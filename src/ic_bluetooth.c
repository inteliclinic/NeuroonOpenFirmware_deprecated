/**
 * @file    ic_bluetooth.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    July, 2016
 * @brief   Bluetooth high-level control.
 */
//---------------------------------------------------------------------------------------------------
#include <stdio.h>
#include "ic_bluetooth.h"
#include "nrf.h"
#include "ble.h"
#include "ble_hci.h"
#include "softdevice_handler.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include <string.h>
#include "ic_delay.h"
#include "ic_cc.h"
#include "ic_cmd_emiter.h"
#include "version.h"

#include "ic_cli.h"
//---------------------------------------------------------------------------------------------------
/** Set of internal configuration macros
 * */
/** UUID ADDRESS BASE */
#define UUID {0x86, 0x08, 0x1C, 0xB8, 0x1C, 0xA1, 0x0C, 0x84, 0xD3, 0xE2, 0x7F, 0xD9, 0x00, 0x00, 0x9E, 0xD0}

/** UUID ADDRESS ALLIAS */
#define UUID_SERVICE                            0x0100

#define UUID_SETTINGS_TX_CHARACTERISTIC 	0x0201
#define UUID_SETTINGS_RX_CHARACTERISTIC 	0x0202
#define UUID_DATA_TX_CHARACTERISTIC 		0x0301
#define UUID_DATA_RX_CHARACTERISTIC 		0x0302
#define UUID_BLE_LOG_TX_CHARACTERISTIC 		0x0401
#define UUID_BLE_LOG_RX_CHARACTERISTIC 		0x0402

#define UUID_DATA_STREAM0_TX_CHARACTERISTIC     0x0201
#define UUID_DATA_STREAM1_TX_CHARACTERISTIC     0x0202
#define UUID_DATA_STREAM2_TX_CHARACTERISTIC     0x0301
#define UUID_RESPONSE_TX_CHARACTERISTIC         0x0302
#define UUID_TEST_TOOL_TX_CHARACTERISTIC        0x0401
#define UUID_TEST_TOOL_RX_CHARACTERISTIC        0x0402
#define UUID_CMD_RX_CHARACTERISTIC              0x0501

#define SECOND_1_25_MS_UNITS                    800 /// Definition of 1 second, when 1 unit is 1.25 ms
#define SECOND_10_MS_UNITS                      100 /// Definition of 1 second, when 1 unit is 10 ms

#define IS_SRVC_CHANGED_CHARACT_PRESENT		1							/// Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device
#define BLE_DEVICE_NAME                     "NeuroOn" 	               	/// Name of device. Will be included in the advertising data

#define MIN_CONN_INTERVAL               	6							/// Minimum acceptable connection interval (0.5 seconds), Connection interval uses 1.25 ms units
#define MAX_CONN_INTERVAL               	32							/// Maximum acceptable connection interval (1 second), Connection interval uses 1.25 ms units

#define SLAVE_LATENCY                   	0                           /// Slave latency
#define CONN_SUP_TIMEOUT                	(6 * SECOND_10_MS_UNITS)	/// Connection supervisory timeout (6 seconds), Supervision Timeout uses 10 ms units

#define SEC_PARAM_TIMEOUT               	30                          /// Timeout for Pairing Request or Security Request (in seconds)
#define SEC_PARAM_BOND                  	0                           /// Perform bonding
#define SEC_PARAM_MITM                  	0                           /// Man In The Middle protection not required
#define SEC_PARAM_IO_CAPABILITIES       	BLE_GAP_IO_CAPS_NONE        /// No I/O capabilities
#define SEC_PARAM_OOB                   	0                           /// Out Of Band data not available
#define SEC_PARAM_MIN_KEY_SIZE          	7                           /// Minimum encryption key size
#define SEC_PARAM_MAX_KEY_SIZE          	16                          /// Maximum encryption key size

#define FIRST_CONN_PARAMS_UPDATE_DELAY  	1 							/// APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) /// Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   	5 							/// APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /// Time between each call to sd_ble_gap_conn_param_update after the first (5 seconds)
#define MAX_CONN_PARAMS_UPDATE_COUNT    	1                           /// Number of attempts before giving up the connection parameter negotiation

#define APP_ADV_INTERVAL                	160                         /// The advertising interval (in units of 0.625 ms. This value corresponds to 1 s)
#define APP_ADV_TIMEOUT_IN_SECONDS      	0x00						    /// The advertising timeout (in units of seconds)
//---------------------------------------------------------------------------------------------------
/** SETTINGS FRAME CMD */
#define		BLE_INTERNAL_CMD_NONE			0x00

#define		BLE_INTERNAL_CMD_STANDBY		0xFF

#define		BLE_INTERNAL_CMD_TASK_SLEEP		0x10		/// use 'sleep' field
#define		BLE_INTERNAL_CMD_TASK_JETLAG	0x11		/// use 'jetlag' field
#define		BLE_INTERNAL_CMD_TASK_NAP		0x12		/// use 'nap' field
#define		BLE_INTERNAL_CMD_TASK_BLT		0x13		/// use 'blt' field

//For task cmd validity test
#define		BLE_INTERNAL_UTILS_TASK_CMD_START		0x10
#define		BLE_INTERNAL_UTILS_TASK_CMD_STOP		0x13

#define		BLE_INTERNAL_CMD_FLASH			0x20		/// use 'flash' field
#define		BLE_INTERNAL_CMD_EPOCH_CLEAR	0x21		/// use 'flash' field

#define		BLE_INTERNAL_CMD_GOTO_TEST		0x30
#define		BLE_INTERNAL_CMD_GOTO_DFU		0x31
//#define		BLE_INTERNAL_CMD_GOTO_RAW		0x32
#define		BLE_INTERNAL_CMD_DEVSEL			0x33
#define		BLE_INTERNAL_CMD_UART_TEST		0x34

//Raw data
#define		BLE_INTERNAL_CMD_RAW_START		0x40
#define		BLE_INTERNAL_CMD_RAW_STOP		0x41
#define		BLE_INTERNAL_CMD_RAW_CLEAR		0x42

//For internal use only
#define		BLE_INTERNAL_CMD_START			0x80
#define		BLE_INTERNAL_CMD_STATUS			0x81
#define		BLE_INTERNAL_CMD_SET_USERID		0x82
#define		BLE_INTERNAL_CMD_GET_SERIAL		0x83

#define		BLE_INTERNAL_CMD_ERROR			0x8F
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
/** SETTINGS ERROR CODES */

#define		BLE_INTERNAL_ERROR_PREV_CMD_NOT_FINISHED		0x01
#define		BLE_INTERNAL_ERROR_CMD_TASK_INVALID				0x02
#define		BLE_INTERNAL_ERROR_CMD_INVALID					0x03
#define		BLE_INTERNAL_ERROR_CMD_RAW_STOP_INVALID_USE		0x04
#define		BLE_INTERNAL_ERROR_CMD_KEY_INVALID				0x05
//---------------------------------------------------------------------------------------------------
BleDevInfo bleDevInfo;
//---------------------------------------------------------------------------------------------------
static BleCmdDesc bleCmdDesc;
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
	uint8_t		cmd;
	uint32_t	key;		///cmd key of dangerous operation
} BleSettingsCmdWithKeyFrame;
//---------------------------------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
	uint16_t		sleepId;
	uint16_t		packetId;
} BleDataFrame;
//---------------------------------------------------------------------------------------------------
static struct {
	uint8_t buf[20];
	bool	valid;
} bleLogFrameUtils;
//---------------------------------------------------------------------------------------------------
static uint8_t	userId[16];
static uint32_t	*timestampPointer = NULL;
//---------------------------------------------------------------------------------------------------
/* Private methods declarations */
static void 	blePrivSoftDeviceEnable();
static void 	blePrivBLEEventHandler(ble_evt_t * bleEvent);
static void 	blePrivSYSEventHandler(uint32_t sys_evt);

static void	blePrivOnReceive(ble_evt_t * bleEvent);
static void	blePrivOnReceiveSettingsFrame(ble_evt_t * bleEvent);
static void     blePrivOnReceiveCMDData(ble_evt_t *bleEvent);
static void     blePrivOnReceiveTestToolData(ble_evt_t *bleEvent);

static void 	blePrivGAPInit();
static void 	blePrivAdvertisingInit();
static void 	blePrivServiceInit();
static void		blePrivConnParamsInit();
static uint32_t	blePrivAddRxCharacteristic(uint16_t charUUID, ble_gatts_char_handles_t *phChar);
static uint32_t	blePrivAddTxCharacteristic(uint16_t charUUID, ble_gatts_char_handles_t *phChar);

static void		blePrivAdvertisingStart();

static uint32_t	blePrivSendPacket(ble_gatts_char_handles_t *hTxChar, uint8_t *buf, uint16_t len);

static void blePrivErrorResp(uint8_t *buf, uint8_t errno);

static void		conn_params_error_handler(uint32_t nrf_error);

static bool		blePrivCheckKey(uint8_t *data, uint32_t key);
//---------------------------------------------------------------------------------------------------
static struct {
	ble_gap_sec_params_t 		secParams;	///	Security parameters

	uint8_t						uuidType;	///	Type of UUID

	uint16_t					hConnect;	///	Handle of main connection

	uint16_t					hService;	/// Handle of main service

	ble_gatts_char_handles_t	hTxCharDataStream0;  /// Handle of Tx Data Stream0 characterisitics
	ble_gatts_char_handles_t	hTxCharDataStream1;  /// Handle of Tx characterisitics
	ble_gatts_char_handles_t	hTxCharDataStream2;  /// Handle of Tx characterisitics

	ble_gatts_char_handles_t	hTxCharResponse;    /// Handle of Tx Data characterisitics

	ble_gatts_char_handles_t	hRxCharCMD;        /// Handle of Rx Log characterisitics

        ble_gatts_char_handles_t        hTxCharTestTool;    /// Handle of Tx Log characterisitics
	ble_gatts_char_handles_t	hRxCharTestTool;    /// Handle of Rx Log characterisitics

} bleConf = {
		.hConnect = BLE_CONN_HANDLE_INVALID
};
//---------------------------------------------------------------------------------------------------
void ble_enableSoftDevice() {
	blePrivSoftDeviceEnable();

	//Unlock command buffer
    bleCmdDesc.valid = false;

    //Unlock log-frame buffer
    bleLogFrameUtils.valid = false;

    //Initialize connection handle to INVALID
	bleConf.hConnect = BLE_CONN_HANDLE_INVALID;
}
//---------------------------------------------------------------------------------------------------
void ble_disableSoftDevice() {
	sd_softdevice_disable();

    //Initialize connection handle to INVALID
	bleConf.hConnect = BLE_CONN_HANDLE_INVALID;
}
//---------------------------------------------------------------------------------------------------
void ble_enableRadioCommunication() {
	uint32_t err_code;
	//Disable SoftDevice ... ensure everything is clear
	sd_softdevice_disable();

	//Enable SoftDevice again
	blePrivSoftDeviceEnable();

	//Enable BLE support
	ble_enable_params_t ble_enable_params;
	memset(&ble_enable_params, 0, sizeof(ble_enable_params));
	ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT; //Include service_change characteristic
	err_code = sd_ble_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	//Set BLE event handler
	err_code = softdevice_ble_evt_handler_set(blePrivBLEEventHandler);
	APP_ERROR_CHECK(err_code);

	//Set SYS event handler
	err_code = softdevice_sys_evt_handler_set(blePrivSYSEventHandler);
    APP_ERROR_CHECK(err_code);

    //Configure GAP connection parameters
    blePrivGAPInit();

    //Configure Services
    blePrivServiceInit();

    //Configure advertising
    blePrivAdvertisingInit();

    //Configure connection parameters
    blePrivConnParamsInit();

    //Configure connection security parameters
    bleConf.secParams.timeout      = SEC_PARAM_TIMEOUT;				/// Timeout for Pairing Request or Security Request (in seconds)
    bleConf.secParams.bond         = SEC_PARAM_BOND;				/// Perform bonding
    bleConf.secParams.mitm         = SEC_PARAM_MITM;				/// Man In The Middle protection not required
    bleConf.secParams.io_caps      = SEC_PARAM_IO_CAPABILITIES;		/// No I/O capabilities
    bleConf.secParams.oob          = SEC_PARAM_OOB;					/// Out Of Band data not available
    bleConf.secParams.min_key_size = SEC_PARAM_MIN_KEY_SIZE;		/// Minimum encryption key size
    bleConf.secParams.max_key_size = SEC_PARAM_MAX_KEY_SIZE;		/// Maximum encryption key size

    //Automatically start advertising
    blePrivAdvertisingStart();

    //Unlock command buffer & clear command
    bleCmdDesc.valid = false;
    bleCmdDesc.type = BLE_INTERNAL_CMD_NONE;

    //Unlock log-frame bufferzaptip
    bleLogFrameUtils.valid = false;

    //Clear userId
    memset(userId, 0, sizeof(userId));

    //Clear timestamp pointer
    timestampPointer = NULL;
    ble_sendName((uint8_t *)"Inteliclinic",12);
    ble_sendFirmRev((uint8_t *)APP_TXT_VERSION, strlen(APP_TXT_VERSION));

    uint8_t tmp[8];
    char buf[20] = {'\0'};
    #ifdef NO_EX_FLASH
    snprintf(buf, 20, "dummy");
    #else
    flash_internal_get_values((uint32_t *)INT_FLASH_SERIAL_HEADER, (uint32_t *)tmp, 2);
    snprintf(buf, 20, "%02d%02d%02d%02d%02d%02d%02d", tmp[2],tmp[1],tmp[0],tmp[7],tmp[6],tmp[5],tmp[4]);
    #endif

    ble_sendSerialNo((uint8_t *)buf, strlen(buf));
}
//---------------------------------------------------------------------------------------------------
void ble_disableRadioCommunication() {
	//Disconnect from remote control
	sd_ble_gap_disconnect(bleConf.hConnect, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	ic_delay_ms(100);
	//Disable SoftDevice ... ensure everything is clear
	sd_softdevice_disable();

	//Enable SoftDevice again
	blePrivSoftDeviceEnable();

    //Initialize connection handle to INVALID
	bleConf.hConnect = BLE_CONN_HANDLE_INVALID;
}
//---------------------------------------------------------------------------------------------------
bool ble_isCentralConnected() {
	if (bleConf.hConnect == BLE_CONN_HANDLE_INVALID)
		return false;

	return true;
}
//---------------------------------------------------------------------------------------------------
bool ble_isValidCmd() {
	return bleCmdDesc.valid;
}
//---------------------------------------------------------------------------------------------------
bool ble_getCommand(BleCmdDesc *bcd) {
	//Check if coping internal bleCmdDesc have any sense...
	if (bleCmdDesc.valid == false || bcd == NULL)
		return false;

	//Copy internal bleCmdDesc to remote memory by provided pointer
	*bcd = bleCmdDesc;

	return true;
}
//---------------------------------------------------------------------------------------------------
void ble_unlockCmdBuffer() {
	if (bleCmdDesc.valid == true) {
		bleCmdDesc.type = BLE_INTERNAL_CMD_NONE;
		bleCmdDesc.valid = false;
	}
}
//---------------------------------------------------------------------------------------------------
static uint32_t blePrivSendPacket(ble_gatts_char_handles_t *hTxChar, uint8_t *buf, uint16_t len) {
	ble_gatts_hvx_params_t hvx_params;

	memset(&hvx_params, 0, sizeof(hvx_params));
	hvx_params.handle = hTxChar->value_handle;
	hvx_params.p_data = buf;
	hvx_params.p_len  = &len;
	hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

	return sd_ble_gatts_hvx(bleConf.hConnect, &hvx_params);
}
//---------------------------------------------------------------------------------------------------
static void blePrivSYSEventHandler(uint32_t sys_evt) {
	//Do nothing...
}
//---------------------------------------------------------------------------------------------------
static void blePrivBLEEventHandler(ble_evt_t * bleEvent) {
	//Universal local error code variable
	uint8_t err_code;
	//This variable is use for store pending connection authentication information.
	static ble_gap_evt_auth_status_t authStatus;
	//Call SDK function to perform basic event service
	ble_conn_params_on_ble_evt(bleEvent);
	//Internal additional event service
	switch (bleEvent->header.evt_id) {
		case BLE_GAP_EVT_CONNECTED: {
			//Save connection handle
			bleConf.hConnect = bleEvent->evt.gap_evt.conn_handle;
                        cc_emit(CC_BLE_CONNECTED, 1);
			//uart_send_u8('C');
			break;
		}
		case BLE_GAP_EVT_DISCONNECTED: {
			//Reset connection handle to invalid state
			bleConf.hConnect = BLE_CONN_HANDLE_INVALID;
                        cc_emit(CC_BLE_DISCONNECTED, 1);
			//Restart advertising (new central can connect to now free device)
			//uart_send_u8('D');
			blePrivAdvertisingStart();
			break;
		}
		case BLE_GATTS_EVT_WRITE: {
			//uart_send_u8('W');
			blePrivOnReceive(bleEvent);
			break;
		}
		case BLE_EVT_TX_COMPLETE: {
			//uart_send_u8('t');
			break;
		}
		case BLE_GAP_EVT_SEC_PARAMS_REQUEST: {
			//For this event it is formal requirements to replay this way (we send our security parameters):
			err_code = sd_ble_gap_sec_params_reply(bleConf.hConnect, BLE_GAP_SEC_STATUS_SUCCESS, &bleConf.secParams);
			APP_ERROR_CHECK(err_code);
			break;
		}
		case BLE_GATTS_EVT_SYS_ATTR_MISSING: {
			//For this event it is formal requirements to replay this way:
			err_code = sd_ble_gatts_sys_attr_set(bleConf.hConnect, 0, 0);
			APP_ERROR_CHECK(err_code);
			break;
		}
		case BLE_GAP_EVT_AUTH_STATUS: {
			//Save authentication status of pending connection
			authStatus = bleEvent->evt.gap_evt.params.auth_status;
			break;
		}
		case BLE_GAP_EVT_SEC_INFO_REQUEST: {
			//For this event it is formal requirements to replay this way:
			//Get pointer to encryption information
			ble_gap_enc_info_t *encInfo = &authStatus.periph_keys.enc_info;
			//Test 'div'
			if (encInfo->div ==  bleEvent->evt.gap_evt.params.sec_info_request.div) {
				err_code = sd_ble_gap_sec_info_reply(bleConf.hConnect, encInfo, 0);
			} else {
				err_code = sd_ble_gap_sec_info_reply(bleConf.hConnect, 0, 0);
			}
			APP_ERROR_CHECK(err_code);
			break;
		}
		case BLE_GAP_EVT_TIMEOUT: {
			break;
		}
		case BLE_GAP_EVT_RSSI_CHANGED: {
			break;
		}
	}
}
//---------------------------------------------------------------------------------------------------
static void blePrivOnReceive(ble_evt_t * bleEvent) {
	ble_gatts_evt_write_t *writeEvent = &(bleEvent->evt.gatts_evt.params.write);
        if (bleConf.hRxCharCMD.value_handle == writeEvent->handle )
          blePrivOnReceiveCMDData(bleEvent);

        if (bleConf.hRxCharTestTool.value_handle == writeEvent->handle )
          blePrivOnReceiveTestToolData(bleEvent);
}
//------------------------------------------------------------------------------

static void blePrivOnReceiveCMDData(ble_evt_t *bleEvent){
  ble_gatts_evt_write_t *event = &bleEvent->evt.gatts_evt.params.write;
  push_data_CMD0Frame(event->data, event->len);
  return;
}

/**
 * @brief Private receiver from CMD1 characteristic (RX_LOG before).
 *
 * Function body has to use old crap because of FT Tester compatibility. Characteristic CMD1 is used
 * for FT Testing purposes.
 *
 * @param bleEvent @ref ble_evt_t
 */
static void blePrivOnReceiveTestToolData(ble_evt_t *bleEvent){
  ble_gatts_evt_write_t *event = &bleEvent->evt.gatts_evt.params.write;

  if (bleLogFrameUtils.valid == false) {
  /*TODO: Old functionality reverted for FT testing purposes*/
    memcpy(bleLogFrameUtils.buf, (uint8_t*)event->data, 20);
    bleLogFrameUtils.valid = true;
  }
  return;
}

//------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
static void blePrivSoftDeviceEnable() {
	//This macro define static array for SoftDevice event buffer storage...
	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);
}
//---------------------------------------------------------------------------------------------------
static void	blePrivConnParamsInit() {
	uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = 0;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}
//---------------------------------------------------------------------------------------------------
static void blePrivGAPInit()
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) BLE_DEVICE_NAME, strlen(BLE_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}
//------------------------------------------------------------------------------
static uint32_t blePrivAddService(uint8_t service_type, uint16_t uuid,
    uint8_t uuid_type, uint16_t *service_handle){

  ble_uuid_t bleUUID = {.uuid = uuid, .type = uuid_type};

  return sd_ble_gatts_service_add(service_type, &bleUUID, service_handle);

}
//------------------------------------------------------------------------------
static uint32_t blePrivAddCharacteristic(ble_uuid_t ble_uuid,
    uint16_t service_handle, ble_gatts_char_handles_t *char_handle,
    uint8_t read){

  ble_gatts_char_md_t char_md;
  ble_gatts_attr_t    attr_char_value;
  ble_gatts_attr_md_t attr_md, cccd_md;

  read = read>0 ? 1 : 0;
  uint8_t notify = read==1 ? 1 : 0;

  if(read)
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  else
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&cccd_md.read_perm);

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vlen    = 0;
  cccd_md.vloc    = BLE_GATTS_VLOC_STACK;
  cccd_md.rd_auth = 0;
  cccd_md.wr_auth = 0;

  char_md.char_props.write            = 1;
  char_md.char_props.read             = read;
  char_md.char_props.notify           = notify;
  char_md.char_props.indicate         = 0;
  char_md.char_props.broadcast        = 0;
  char_md.char_props.auth_signed_wr   = 0;
  char_md.char_props.write_wo_resp    = 0;
  char_md.char_ext_props.reliable_wr  = 0;
  char_md.char_ext_props.wr_aux       = 0;
  char_md.p_char_user_desc            = NULL;
  char_md.char_user_desc_max_size     = 0;
  char_md.char_user_desc_size         = 0;
  char_md.p_char_pf                   = NULL;
  char_md.p_user_desc_md              = NULL;
  char_md.p_cccd_md                   = read == 1 ? &cccd_md : NULL;
  char_md.p_sccd_md                   = NULL;

  if(read)
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  else
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vlen       = 1;
  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 0;
  attr_md.wr_auth    = 0;

  attr_char_value.p_uuid       = &ble_uuid;
  attr_char_value.p_attr_md    = &attr_md;
  attr_char_value.init_len     = 1;
  attr_char_value.init_offs    = 0;
  attr_char_value.max_len      = 20;

  return sd_ble_gatts_characteristic_add(service_handle, &char_md,
      &attr_char_value,
      char_handle);
}
//------------------------------------------------------------------------------
static uint16_t batteryService;
static uint16_t devInfoService;
static ble_gatts_char_handles_t batteryChar;
static ble_gatts_char_handles_t devInfoNameChar;
static ble_gatts_char_handles_t devInfoModelNoChar;
static ble_gatts_char_handles_t devInfoSerialNoChar;
static ble_gatts_char_handles_t devInfoHardRevChar;
static ble_gatts_char_handles_t devInfoFirmRevChar;
static ble_gatts_char_handles_t devInfoSoftRevChar;
static ble_gatts_char_handles_t devInfoSystemIDChar;

static void blePrivServiceInit() {
	uint32_t err_code;

	ble_uuid128_t service_base_uuid = {UUID};

	//Add vendor UUID to SoftDevice BLE stack
	err_code = sd_ble_uuid_vs_add(&service_base_uuid, &(bleConf.uuidType));
	APP_ERROR_CHECK(err_code);

	//Initialize connection handle to INVALID
	bleConf.hConnect = BLE_CONN_HANDLE_INVALID;

	//Add main service
	err_code = blePrivAddService(BLE_GATTS_SRVC_TYPE_PRIMARY, UUID_SERVICE,
            bleConf.uuidType, &bleConf.hService);
	APP_ERROR_CHECK(err_code);


	//Add all characteristics
        err_code = blePrivAddCharacteristic(
            (ble_uuid_t){.uuid = UUID_DATA_STREAM0_TX_CHARACTERISTIC, .type = bleConf.uuidType},
            bleConf.hService, &bleConf.hTxCharDataStream0, 1);
        APP_ERROR_CHECK(err_code);
        err_code = blePrivAddCharacteristic(
            (ble_uuid_t){.uuid = UUID_DATA_STREAM1_TX_CHARACTERISTIC, .type = bleConf.uuidType},
            bleConf.hService, &bleConf.hTxCharDataStream1, 1);
        APP_ERROR_CHECK(err_code);
        err_code = blePrivAddCharacteristic(
            (ble_uuid_t){.uuid = UUID_DATA_STREAM2_TX_CHARACTERISTIC, .type = bleConf.uuidType},
            bleConf.hService, &bleConf.hTxCharDataStream2, 1);
        APP_ERROR_CHECK(err_code);
        err_code = blePrivAddCharacteristic(
            (ble_uuid_t){.uuid = UUID_RESPONSE_TX_CHARACTERISTIC, .type = bleConf.uuidType},
            bleConf.hService, &bleConf.hTxCharResponse, 1);
        APP_ERROR_CHECK(err_code);
        err_code = blePrivAddCharacteristic(
            (ble_uuid_t){.uuid = UUID_TEST_TOOL_TX_CHARACTERISTIC, .type = bleConf.uuidType},
            bleConf.hService, &bleConf.hTxCharTestTool, 1);
        APP_ERROR_CHECK(err_code);
        err_code = blePrivAddCharacteristic(
            (ble_uuid_t){.uuid = UUID_TEST_TOOL_RX_CHARACTERISTIC, .type = bleConf.uuidType},
            bleConf.hService, &bleConf.hRxCharTestTool, 0);
        APP_ERROR_CHECK(err_code);
        err_code = blePrivAddCharacteristic(
            (ble_uuid_t){.uuid = UUID_CMD_RX_CHARACTERISTIC, .type = bleConf.uuidType},
            bleConf.hService, &bleConf.hRxCharCMD, 0);
        APP_ERROR_CHECK(err_code);

        err_code = blePrivAddService(BLE_GATTS_SRVC_TYPE_PRIMARY, 0x180F,
            BLE_UUID_TYPE_BLE, &batteryService);
        APP_ERROR_CHECK(err_code);
        err_code = blePrivAddCharacteristic(
            (ble_uuid_t){.uuid = 0x2A19, .type = BLE_UUID_TYPE_BLE},
            batteryService, &batteryChar, 1);
        APP_ERROR_CHECK(err_code);

        err_code = blePrivAddService(BLE_GATTS_SRVC_TYPE_PRIMARY, 0x180A,
            BLE_UUID_TYPE_BLE, &devInfoService);
        APP_ERROR_CHECK(err_code);
        err_code = blePrivAddCharacteristic(
            (ble_uuid_t){.uuid = 0x2A29, .type = BLE_UUID_TYPE_BLE},
            devInfoService, &devInfoNameChar, 1);
        APP_ERROR_CHECK(err_code);
        err_code = blePrivAddCharacteristic(
            (ble_uuid_t){.uuid = 0x2A25, .type = BLE_UUID_TYPE_BLE},
            devInfoService, &devInfoSerialNoChar, 1);
        APP_ERROR_CHECK(err_code);
        err_code = blePrivAddCharacteristic(
            (ble_uuid_t){.uuid = 0x2A26, .type = BLE_UUID_TYPE_BLE},
            devInfoService, &devInfoFirmRevChar, 1);
        APP_ERROR_CHECK(err_code);
}
//---------------------------------------------------------------------------------------------------
static uint32_t	blePrivAddRxCharacteristic(uint16_t charUUID, ble_gatts_char_handles_t *phChar) {
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.write  = 1;
	char_md.p_char_user_desc  = NULL;
	char_md.p_char_pf         = NULL;
	char_md.p_user_desc_md    = NULL;
	char_md.p_cccd_md         = NULL;
	char_md.p_sccd_md         = NULL;

	ble_uuid.type = bleConf.uuidType;
	ble_uuid.uuid = charUUID;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid       = &ble_uuid;
	attr_char_value.p_attr_md    = &attr_md;
	attr_char_value.init_len     = 1;
	attr_char_value.init_offs    = 0;
	attr_char_value.max_len      = 20;

        return sd_ble_gatts_characteristic_add(	bleConf.hService, &char_md,
            &attr_char_value,
            phChar);
}
//---------------------------------------------------------------------------------------------------
static uint32_t	blePrivAddTxCharacteristic(uint16_t charUUID, ble_gatts_char_handles_t *phChar) {
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md, cccd_md;

	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.write  = 1;
	char_md.char_props.notify  = 1;
    char_md.char_props.read   = 1;
	char_md.p_char_user_desc  = NULL;
	char_md.p_char_pf         = NULL;
	char_md.p_user_desc_md    = NULL;
	char_md.p_cccd_md         = &cccd_md;
	char_md.p_sccd_md         = NULL;

        ble_uuid.type = bleConf.uuidType;
	ble_uuid.uuid = charUUID;

	memset(&attr_md, 0, sizeof(attr_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid       = &ble_uuid;
	attr_char_value.p_attr_md    = &attr_md;
	attr_char_value.init_len     = 1;
	attr_char_value.init_offs    = 0;
	attr_char_value.max_len      = 20;

	return sd_ble_gatts_characteristic_add(	bleConf.hService, &char_md,
											&attr_char_value,
											phChar);
}
//---------------------------------------------------------------------------------------------------
static void blePrivAdvertisingInit() {
	uint32_t      err_code;
	ble_advdata_t advdata;
	ble_advdata_t scanrsp;
	uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


	ble_uuid_t adv_uuids[] = {{UUID_SERVICE,bleConf.uuidType}};

	memset(&advdata, 0, sizeof(advdata));
	advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance      = false;
	advdata.flags.size              = sizeof(flags);
	advdata.flags.p_data            = &flags;

	memset(&scanrsp, 0, sizeof(scanrsp));
	scanrsp.uuids_complete.uuid_cnt =  sizeof(adv_uuids) / sizeof(adv_uuids[0]);
	scanrsp.uuids_complete.p_uuids  = adv_uuids;

	err_code = ble_advdata_set(&advdata, &scanrsp);
	APP_ERROR_CHECK(err_code);
}
//---------------------------------------------------------------------------------------------------
void blePrivAdvertisingStart()
{
	uint32_t				err_code;
	ble_gap_adv_params_t	adv_params;

	memset(&adv_params, 0, sizeof(adv_params));

	adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
	adv_params.p_peer_addr = 0;
	adv_params.fp          = BLE_GAP_ADV_FP_ANY;
	adv_params.interval    = APP_ADV_INTERVAL;
	adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

	err_code = sd_ble_gap_adv_start(&adv_params);
	APP_ERROR_CHECK(err_code);
}
//---------------------------------------------------------------------------------------------------
bool ble_getTestToolCmd(uint8_t *buf, uint16_t len) {
  /*TODO: Old crap used (FT Tester)*/
	if (bleLogFrameUtils.valid) {
		memcpy(buf, bleLogFrameUtils.buf, len);
		bleLogFrameUtils.valid = false;

		return true;
	}

	return false;
}
//---------------------------------------------------------------------------------------------------
#ifdef LEGACY_COMMUNICATION
bool ble_getLogFrame(uint8_t *buf) {
	if (bleLogFrameUtils.valid) {
		memcpy(buf, bleLogFrameUtils.buf, 20);
		bleLogFrameUtils.valid = false;

		return true;
	}

	return false;
}
//---------------------------------------------------------------------------------------------------
void ble_sendLogFrame(uint8_t *buf) {
	blePrivSendPacket(&bleConf.hTxCharLog, buf);
}
//---------------------------------------------------------------------------------------------------
void ble_sendDataPacket(uint8_t *buf) {
	blePrivSendPacket(&bleConf.hTxCharData, buf);
}
#else
//---------------------------------------------------------------------------------------------------
uint32_t ble_sendDataPacketStream0(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&bleConf.hTxCharDataStream0, buf, len);
}
uint32_t ble_sendDataPacketStream1(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&bleConf.hTxCharDataStream1, buf, len);
}
uint32_t ble_sendDataPacketStream2(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&bleConf.hTxCharDataStream2, buf, len);
}

uint32_t ble_sendResponse(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&bleConf.hTxCharResponse, buf, len);
}

uint32_t ble_sendBatteryState(uint8_t *buf, uint16_t len){
  return blePrivSendPacket(&batteryChar, buf, len);
}

uint32_t ble_sendName(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&devInfoNameChar, buf, len);
}
uint32_t ble_sendModelNo(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&devInfoModelNoChar, buf, len);
}
uint32_t ble_sendSerialNo(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&devInfoSerialNoChar, buf, len);
}
uint32_t ble_sendHardRev(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&devInfoHardRevChar, buf, len);
}
uint32_t ble_sendFirmRev(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&devInfoFirmRevChar, buf, len);
}
uint32_t ble_sendSoftRev(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&devInfoSoftRevChar, buf, len);
}
uint32_t ble_sendSystemID(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&devInfoSystemIDChar, buf, len);
}
uint32_t ble_sendTestToolData(uint8_t *buf, uint16_t len) {
	return blePrivSendPacket(&bleConf.hTxCharTestTool, buf, len);
}
//---------------------------------------------------------------------------------------------------
uint32_t ble_sendDataPacket(uint8_t *buf) {
  return 0;
}
#endif
//---------------------------------------------------------------------------------------------------
void ble_getUserId(uint8_t *buf) {
	memcpy(buf, userId, 16);
}
//---------------------------------------------------------------------------------------------------
static bool		blePrivCheckKey(uint8_t *data, uint32_t key) {
	BleSettingsCmdWithKeyFrame *fr = (BleSettingsCmdWithKeyFrame*)data;
	if (fr->key == key)
		return true;

	return false;
}
//---------------------------------------------------------------------------------------------------
/**
 * @fn app_error_handler ()
 * @brief global handler for application errors
 * @param error_code refer to nrf51822/Include/s110/nrf_error.h for error codes
 * @param line_num line number
 * @param p_file_name name of the file
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
	//HardFault_Handler();
	NVIC_SystemReset();
}
//---------------------------------------------------------------------------------------------------
/**
 * @fn assert_nrf_callback ()
 * @brief handler for SoftDevice errors
 * @param line_num line number
 * @param p_file_name name of the file
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}
//---------------------------------------------------------------------------------------------------
/**
 * @fn conn_params_error_handler ()
 * @brief connection parameters error handler
 * @param error code
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
//---------------------------------------------------------------------------------------------------
