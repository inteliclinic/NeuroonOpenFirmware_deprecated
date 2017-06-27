/**
 * @file    ic_bluetooth.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    July, 2016
 * @brief   Bluetooth high-level control.
 */
//---------------------------------------------------------------------------------------------------
#ifndef INC_IC_BLUETOOTH_H_
#define INC_IC_BLUETOOTH_H_
//---------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
/**
 * @brief	Structure for device settings provided by BLE to remote central
 */
typedef struct {
	uint32_t	appVersion;
	uint32_t	dfuVersion;
	uint16_t	bootVersion;

	uint16_t	sleepNumber;
	uint32_t	freePackets;

	uint8_t		batteryState;

	uint32_t	serial[2];
} BleDevInfo;
//--------------------------------------------------------------------------------------------------
/**
 * @brief	Basic information provided by BLE to remote central
 *
 * @note	All information in this object should be provided before call @ref bleEnableRadioCommunication()
 * 			function. During normal BLE radio operation only uint8_t fields can be modified.
 */
extern BleDevInfo bleDevInfo;
//--------------------------------------------------------------------------------------------------
#define		BLE_CMD_STANDBY			0xFF

#define		BLE_CMD_SLEEP			0x10		/// use 'sleep' field
#define		BLE_CMD_JETLAG			0x11		/// use 'jetlag' field
#define		BLE_CMD_NAP				0x12		/// use 'nap' field
#define		BLE_CMD_BLT				0x13		/// use 'blt' field

#define		BLE_CMD_FLASH			0x20		/// use 'flash' field
#define		BLE_CMD_EPOCH_CLEAR		0x21		/// clear epoch data space

#define		BLE_CMD_GOTO_TEST		0x30
#define		BLE_CMD_GOTO_DFU		0x31
//#define		BLE_CMD_GOTO_RAW		0x32
#define		BLE_CMD_DEVSEL			0x33
#define		BLE_CMD_UART_TEST		0x34

//Raw data
#define		BLE_CMD_RAW_START		0x40
#define		BLE_CMD_RAW_STOP		0x41
#define		BLE_CMD_RAW_CLEAR		0x42
//--------------------------------------------------------------------------------------------------
/**
 * @brief	Commands received during BLE communication with remote central
 */
typedef struct {
	bool		valid;			/// other data in structure is valid only if this field is true

	uint8_t		type;			/// type of command

	union {
		struct {
			uint32_t	timestamp;		/// task start timestamp
			uint16_t	timeToJetLag;	/// time to JetLag in minutes
			uint16_t	timeToAlarm;	/// time to alarm (vibration) in minutes
			uint8_t		adLength;		/// artificial daybreak length in minutes
			uint8_t 	intensity_of_ad; /// Intensity of artificial-down, value 10-100
		} jetlag;

		struct {
			uint32_t	timestamp;		/// task start timestamp
			uint16_t	timeToAlarm;	/// time to alarm (vibration) in minutes
			uint8_t		adLength;		/// artificial daybreak length in minutes3
			uint8_t 	intensity_of_ad; /// Intensity of artificial-down, value 10-100
		} sleep;

		struct {
			uint32_t	timestamp;		/// task start timestamp
			uint8_t		napLength;		/// NAP length in minutes
			uint8_t		maxTime;		/// maximum time of nap in minutes, after that alarm should be activated
			uint8_t		adLength;		/// artificial daybreak length in minutes
			 //---------------------------
			uint8_t		contactLevel;
			//---------------------------
			uint8_t		accEventStartLevel;
			uint8_t		accEventStopLevel;
			uint8_t		accEventTailETime;
			//----------------------------
			uint16_t	spindlesStartLevel;
			uint16_t	spindlesStopLevel;
			uint8_t		spindlesTailETime;
			//----------------------------
			uint8_t		asleepWndEStart;
			uint8_t		asleepWndEStop;
			uint8_t		asleepMaxWndTimerDelay;
			//---------------------------
			uint8_t		spindlesStdMALength;
			//---------------------------
			uint8_t 	intensity_of_ad; /// Intensity of artificial-down, value 10-100
		} nap;

		struct {
			uint32_t	timestamp;		/// task start timestamp
			uint8_t		bltLength;		/// BLT length in minutes
		} blt;

		struct {
			uint16_t	sleepId;		/// ID of sleep in flash memory
			uint16_t	packetId;		///	ID of packet in sleep
		} flash;

		struct {
			uint16_t	page;
			bool		single;
			bool		stop;
		} rawdata;
	};
} BleCmdDesc;
//--------------------------------------------------------------------------------------------------
/**
 * @brief	This function enable SoftDevice if it is disabled.
 *
 * @note	Don't use any SoftDevice function before and during this function call (also in interrupts).
 */
void ble_enableSoftDevice();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	This function disable SoftDevice.
 *
 * @note	Don't use any SoftDevice function after and during this function call (also in interrupts).
 */
void ble_disableSoftDevice();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Enable radio communication.
 *
 * @details	Please note that this function reset SoftDevice stack and initialize/reinitialize everything
 * 			(BLE support, services, characteristics and so on).
 *
 * @note	Don't use any SoftDevice function during this function call (especially in interrupts).
 */
void ble_enableRadioCommunication();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Disable radio communication.
 *
 * @details	Please note that this function reset SoftDevice stack and NOT initialize BLE support
 * 			in SoftDevice to ensure no radio activity.
 *
 * @note	Don't use any SoftDevice function during this function call (especially in interrupts).
 */
void ble_disableRadioCommunication();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Test if device is connected to remote central via BLE.
 * @return	true if device is connected, false otherwise
 */
bool ble_isCentralConnected();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Test if there is valid command received via BLE
 *
 * @param	bcd		pointer to structure @ref BleCmdDesc be filled last known command
 *
 * @return	true if command is valid, false otherwise
 */
bool ble_isValidCmd();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Get command form remote control (received via BLE)
 *
 * @param	bcd		pointer to structure @ref BleCmdDesc to be filled last known command
 *
 * @return	true if written command is valid, false otherwise
 */
bool ble_getCommand(BleCmdDesc *bcd);
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Get actual user id.
 *
 * @param	buf Buffer for 16B user id/
 */
void ble_getUserId(uint8_t *buf);
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Unlock internal command buffer.
 *
 * 			In normal operation internal buffer is locked after received a valid command from remote
 * 			central (via BLE). This provide a stable access to received command. After read command
 * 			by @ref bleGetCommand function, internal buffer should by unlock by this function
 * 			or BLE stack should be disabled by @ref bleDisableRadioCommunication() or bleDisableSoftDevice()
 */
void ble_unlockCmdBuffer();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Get command frame from test_tool characteristic.
 *
 * @param[out]	buf	Pointer to buffer for command data from test_tool characteristic.
 * @param[in]	len	Command frame size.
 *
 * @return	true if any valid data has been copied to $buf, false otherwise.
 */
bool ble_getTestToolCmd(uint8_t *buf, uint16_t len);
#ifdef LEGACY_COMMUNICATION
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Get 20B-frame from log-characteristic.
 *
 * @param	buf	Pointer to buffer for 20 bytes data from log-characteristic.
 *
 * @return	true if any valid data has been copied to $buf, false otherwise.
 */
bool ble_getLogFrame(uint8_t *buf);
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Send 20B-frame via log-characteristic.
 *
 * @param	buf	Pointer to 20B-buffer with data to send.
 */
void ble_sendLogFrame(uint8_t *buf);
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Send 20B-frame via data-characteristic.
 *
 * @param	buf	Pointer to 20B-buffer with data to send.
 */
void ble_sendDataPacket(uint8_t *buf);
#else
uint32_t ble_sendDataPacketStream0(uint8_t *buf, uint16_t len);
uint32_t ble_sendDataPacketStream1(uint8_t *buf, uint16_t len);
uint32_t ble_sendDataPacketStream2(uint8_t *buf, uint16_t len);
uint32_t ble_sendResponse(uint8_t *buf, uint16_t len);
uint32_t ble_sendBatteryState(uint8_t *buf, uint16_t len);
uint32_t ble_sendName(uint8_t *buf, uint16_t len);
uint32_t ble_sendModelNo(uint8_t *buf, uint16_t len);
uint32_t ble_sendSerialNo(uint8_t *buf, uint16_t len);
uint32_t ble_sendHardRev(uint8_t *buf, uint16_t len);
uint32_t ble_sendFirmRev(uint8_t *buf, uint16_t len);
uint32_t ble_sendSoftRev(uint8_t *buf, uint16_t len);
uint32_t ble_sendSystemID(uint8_t *buf, uint16_t len);
uint32_t ble_sendTestToolData(uint8_t *buf,uint16_t len);
uint32_t ble_sendDataPacket(uint8_t *buf);
#endif
//---------------------------------------------------------------------------------------------------
#endif /* INC_IC_BLUETOOTH_H_ */
//---------------------------------------------------------------------------------------------------
