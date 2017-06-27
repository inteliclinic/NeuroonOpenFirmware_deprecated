/*
 * ic_state_test_tool.c

 *
 *  Created on: 15 paź 2015
 *      Author: kn
 */
#include "ic_state_test_tool.h"
#include "ic_bluetooth.h"
#include "ic_bq27742.h"
#include "ic_charger.h"
#include "ic_delay.h"
#include "ic_electrotest.h"
#include "ic_flash.h"
#include "ic_flash_map.h"
#include "ic_lis3dh.h"
#include "ic_ltc3220.h"
#include <ic_eeg.h>
#include "ic_timers.h"
#include "string.h"

#define TEST_TOOL_FRAME_SIZE 20

uint8_t buf_state_test_tool_cmd[TEST_TOOL_FRAME_SIZE];
uint8_t buf_state_test_tool_ack[TEST_TOOL_FRAME_SIZE];
uint8_t buf_to_send[TEST_TOOL_FRAME_SIZE];
volatile int32_t test_afe_data[2];

volatile uint8_t repeat=false;

measurement *data, data_accelerometer;

volatile int32_t test_afe_data[2];
volatile measurement test_lis_data;
volatile bool acc_running=false;


uint32_t tmp_serial[2];

volatile uint8_t commadn_received;

extern uint8_t serial_in_ram[8];

static void ptr_replace(uint8_t **ptr1, uint8_t **ptr2){
	uint8_t *tmp;
	tmp=*ptr1;
	*ptr1=*ptr2;
	*ptr2=tmp;
}

state_exit_code state_test_tool()
{
	button_clear_state();
	ResetBit(GPIO_LEDS_ON);
	ResetBit(GPIO_POWER_DIGITAL);
	charger_init();
	ic_delay_ms(10);
	SetBit(GPIO_LEDS_ON);
	SetBit(GPIO_POWER_DIGITAL);
	ic_delay_ms(10);
	TWI_PIN_SELECT(GPIO_SCL_PIN, GPIO_SDA_PIN);
	TWI_Init(K400);
	SetBit(GPIO_POWER_ANALOG);
	LIS3DH_Config(LIS3DH_Set_Mode(1));
	LIS3DH_INT1_Config();
	bq27742_init();
	ads_init();

	SetBit(GPIO_AFE_PDN);
	afe4400_start();
	afe4400_get_config()->led1_current = 0x3A;
	afe4400_get_config()->led2_current = 0x3A;
	afe4400_get_config()->rf_led = 0x05;
	afe4400_get_config()->cf_led = 0x02;
	afe4400_get_config()->gain = 0x04;
	afe4400_init();
	afe4400_stop();

	data = &data_accelerometer;
	diff_p = &diff;

	volatile uint8_t cmd=0;
	state_flag command_flag;
	//LOOP SECTION
	//KN change
	uint8_t kn_buf1[TEST_TOOL_FRAME_SIZE];
	uint8_t kn_buf2[TEST_TOOL_FRAME_SIZE];
	uint8_t *kn_buf_ptr1=kn_buf1;
	uint8_t *kn_buf_ptr2=kn_buf2;
	uint8_t kn_valid1=0, kn_valid2=0;

	while(1)
	{
		while(!kn_valid1)
		{
			kn_valid1=ble_getTestToolCmd(kn_buf_ptr1, TEST_TOOL_FRAME_SIZE);
			WDT_RR();
			if(button_is_pressed())
			{
				button_clear_state();
				state_test_tool_deinit();
				return TEST_TOOL_SM_END_TEST_MESSAGE;
			}
		}
		while(!kn_valid2){
			kn_valid2 = ble_getTestToolCmd(kn_buf_ptr2, TEST_TOOL_FRAME_SIZE);
			WDT_RR();
			if(button_is_pressed())
			{
				button_clear_state();
				state_test_tool_deinit();
				return TEST_TOOL_SM_END_TEST_MESSAGE;
			}

			if (kn_buf_ptr1[0]!=kn_buf_ptr2[0]){
				ptr_replace(&kn_buf_ptr1, &kn_buf_ptr2);
				continue;
			}
			else{
				kn_valid2=true;
			}

		}


		if(kn_valid2)
		{
			cmd=kn_buf_ptr1[0];
			kn_valid1=false;
			kn_valid2=false;
			kn_buf_ptr2[0]=0;
			repeat=true;
			if (cmd!=0x55)
			{
				command_flag=interpret_command_ble(cmd);
				ble_sendTestToolData(kn_buf_ptr1, TEST_TOOL_FRAME_SIZE);
				while(repeat)
				{
					execute_command(command_flag);// execute zmienia repeat na flase, jesli ma się wykonać 1 raz


					if(!kn_valid1)
					{
						kn_valid1=ble_getTestToolCmd(kn_buf_ptr1, TEST_TOOL_FRAME_SIZE);
					}
					if(kn_valid1==true && ble_getTestToolCmd(kn_buf_ptr2, TEST_TOOL_FRAME_SIZE)){
						if (kn_buf_ptr1[0]!=kn_buf_ptr2[0]){
							ptr_replace(&kn_buf_ptr1, &kn_buf_ptr2);
						}
						else{
							kn_valid2=true;
						}
					}
					if (kn_valid2==true)
						break;



//				//	kn_valid1=ble_getLogFrame(kn_buf_ptr1);
//					if(kn_valid1)
//					{
//						//ble_sendLogFrame(kn_buf_ptr1);
//						WDT_RR();
//						break;
//					}
					WDT_RR();
					if(button_is_pressed())
					{
						button_clear_state();
						state_test_tool_deinit();
						return TEST_TOOL_SM_END_TEST_MESSAGE;
					}
				}
				repeat=false;
				WDT_RR();
			}//if cmd!=0x55
			else
			{
				state_test_tool_deinit();
				return TEST_TOOL_SM_END_TEST_MESSAGE;
			}
			WDT_RR();
		}//if(valid)
	}//while(1)
	ble_disableRadioCommunication();
	button_clear_state();
	charger_init();
	state_test_tool_deinit();
	return TEST_TOOL_SM_END_TEST_MESSAGE;
}

void state_test_tool_deinit(void)
{
	ads_deinit();
	afe4400_stop();
	ResetBit(GPIO_AFE_PDN);
	TWI_Deinit();
	ResetBit(GPIO_POWER_DIGITAL);
	ResetBit(GPIO_LEDS_ON);
	SetBit(GPIO_POWER_ANALOG);

}

void execute_command(state_flag current_flag)
{
	WDT_RR();
	switch(current_flag)
	{
		case LEDR1: //61
			SetBit(GPIO_LEDS_ON);
			RGB_left_set(COLOR_RED, 10);
			break;
		case LEDG1: //62
			SetBit(GPIO_LEDS_ON);
			RGB_left_set(COLOR_GREEN, 10);
			break;
		case LEDB1: //63
			SetBit(GPIO_LEDS_ON);
			RGB_left_set(COLOR_BLUE, 10);
			break;
		case LEDR2: //64
			SetBit(GPIO_LEDS_ON);
			RGB_right_set(COLOR_RED, 10);
			break;
		case LEDG2: //65
			SetBit(GPIO_LEDS_ON);
			RGB_right_set(COLOR_GREEN, 10);
			break;
		case LEDB2: //66
			SetBit(GPIO_LEDS_ON);
			RGB_right_set(COLOR_BLUE, 10);
			break;
		case POWER_LED: //67 68
			power_led_flash_test_tool();
			repeat=false;
			break;
		case POWER_LED_NO_REFRESH://6F
			power_led_flash_test();
			ic_delay_ms(5);
			break;
		case LEDS_OFF: //69
			ResetBit(GPIO_LEDS_ON);
			repeat=false;
			break;
		case AFE_ST: //6B
			repeat=false;
			uint32_t tmp;
			tmp = afe4400_SelfTest();
			buf_to_send[0]=0x6B;
			buf_to_send[1]=0x01;
			buf_to_send[2]=((uint8_t)tmp>>8);
			buf_to_send[3]=((uint8_t)tmp);
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case VIBRA_ST: //74
//opcja Y N
//			repeat=false;
//			//volatile int result=test_vibrator_st();
//			volatile int res;
//			volatile uint8_t passed=false;
//			buf_to_send[0]=0x74;
//			buf_to_send[1]=0x01;
//			res=test_vibrator_st();
//			if(res>20000)
//				passed=true;
//			if(passed)
//				buf_to_send[2]=0x59;
//			else
//				buf_to_send[2]=0x4e;
//
//			ble_sendLogFrame(buf_to_send);
//
//			break;
//opcja ze zwracaniem wartości

			repeat=false;
			//volatile int result=test_vibrator_st();
			volatile int res;
			SetBit(GPIO_LEDS_ON);
			buf_to_send[0]=0x74;
			buf_to_send[1]=0x01;
			res=test_vibrator_st();
			buf_to_send[2]=(uint8_t)(res>>24);
			buf_to_send[3]=(uint8_t)(res>>16);
			buf_to_send[4]=(uint8_t)(res>>8);
			buf_to_send[5]=(uint8_t)(res);
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case AFE_ON: //6C
			repeat=false;
			afe4400_start();
			break;
		case AFE_OFF: //6D
			repeat=false;
			afe4400_stop();
			break;
		case AFE_DIODE1: //26
			repeat=false;
			afe4400_TakeData((int32_t*)test_afe_data);
			buf_to_send[0]=0x26;
			buf_to_send[1]=0x01;
			buf_to_send[2]=((uint8_t)(test_afe_data[0]>>24));
			buf_to_send[3]=((uint8_t)(test_afe_data[0]>>16));
			buf_to_send[4]=((uint8_t)(test_afe_data[0]>>8));
			buf_to_send[5]=((uint8_t)(test_afe_data[0]));
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case AFE_DIODE2: //2A
			repeat=false;
			afe4400_TakeData((int32_t*)test_afe_data);
			buf_to_send[0]=0x2A;
			buf_to_send[1]=0x01;
			buf_to_send[2]=((uint8_t)(test_afe_data[1]>>24));
			buf_to_send[3]=((uint8_t)(test_afe_data[1]>>16));
			buf_to_send[4]=((uint8_t)(test_afe_data[1]>>8));
			buf_to_send[5]=((uint8_t)(test_afe_data[1]));
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case BANDWIDTH_INFO: //71
			repeat=false;
			buf_to_send[0]=0x71;
			buf_to_send[1]=0x01;
			//zwraca informacje o udziałach w pasmach
			timer_startSection(SECTION_EEG);
			eeg_clearModule();
			timer_busInit(125);
			volatile int i=0;
			timer_startSection(SECTION_EEG);
			while (eeg_getComputedEpochSamplesNumber()!=500)
			{
				i++;
				if (eeg_computeNextSampleForTesttooling())
				{
					if(eeg_getComputedEpochSamplesNumber()==500)
					{
						EEG_TesttoolingEpoch tmptmp=eeg_getTesttoolingEpoch();
						buf_to_send[2]=tmptmp.partAlpha;
						buf_to_send[3]=tmptmp.partBeta;
						buf_to_send[4]=tmptmp.partDelta;
						buf_to_send[5]=tmptmp.partSpindles;
						buf_to_send[6]=tmptmp.partTheta;
					}
				}
				WDT_RR();
			}
			timer_busDeinit();
			buf_to_send[7]=0x99;
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case VIBRA_ON: //72
			SetBit(GPIO_LEDS_ON);
			ltc3220_set_chargepump(2);
			ltc3220_set_brightness(VIBRATOR1_CHANNEL1,0x3f);
			ltc3220_set_brightness(VIBRATOR1_CHANNEL2,0x3f);
			break;
		case VIBRA_OFF: //73
			ltc3220_set_brightness(VIBRATOR1_CHANNEL1,0x00);
			ltc3220_set_brightness(VIBRATOR1_CHANNEL2,0x00);
			break;
		case ACC_ON: //75
			repeat=false;
			LIS3DH_Start();
			acc_running=true;
			break;
		case ACC_OFF: //77
			repeat=false;
			LIS3DH_Stop();
			acc_running=false;
			break;
		case ACC_X: //78
			//tu skonczyłem, problemsy, nie dziala jak trzeba
			repeat=false;
			buf_to_send[0]=0x78;
			buf_to_send[1]=0x01;
			buf_to_send[2]=data->ACC_DATA_REG[0];
			buf_to_send[3]=data->ACC_DATA_REG[1];
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case ACC_Y: //79
			repeat=false;
			buf_to_send[0]=0x79;
			buf_to_send[1]=0x01;
			buf_to_send[2]=data->ACC_DATA_REG[2];
			buf_to_send[3]=data->ACC_DATA_REG[3];
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case ACC_Z: //7A
			repeat=false;
			buf_to_send[0]=0x7A;
			buf_to_send[1]=0x01;
			buf_to_send[2]=data->ACC_DATA_REG[4];
			buf_to_send[3]=data->ACC_DATA_REG[5];
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case ACC_ST: //6A
			repeat=false;
			buf_to_send[0]=0x6A;
			buf_to_send[1]=0x01;
			uint8_t buffer[18];
			LIS3DH_Self_Test(buffer);
			WDT_RR();
			int licznik=0;
			for(licznik=0;licznik<18;licznik++)
			{
				buf_to_send[licznik+2]=buffer[licznik];
			}
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case BATT_TEMP: //30
			repeat=false;
			volatile uint8_t data_bq[8];
			bq27742_read_measurement_data((uint8_t*)data_bq);
			buf_to_send[0]=0x30;
			buf_to_send[1]=0x01;
			buf_to_send[2]=(uint8_t)data_bq[0];
			buf_to_send[3]=(uint8_t)data_bq[1];
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case GET_SERIAL_RAM: //28
			repeat=false;
			buf_to_send[0]=0x28;
			buf_to_send[1]=0x01;
			for(i=0; i<8; i++)
			{
				buf_to_send[i+2]=serial_in_ram[i];
			}
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case GET_SERIAL_FLASH: //3e
			repeat=false;
			buf_to_send[0]=0x3E;
			buf_to_send[1]=0x01;
			flash_internal_get_values((uint32_t*)INT_FLASH_SERIAL_HEADER,tmp_serial, 2);
			buf_to_send[2]=(uint8_t)(tmp_serial[0]>>24);
			buf_to_send[3]=(uint8_t)(tmp_serial[0]>>16);
			buf_to_send[4]=(uint8_t)(tmp_serial[0]>>8);
			buf_to_send[5]=(uint8_t)(tmp_serial[0]);
			buf_to_send[6]=(uint8_t)(tmp_serial[1]>>24);
			buf_to_send[7]=(uint8_t)(tmp_serial[1]>>16);
			buf_to_send[8]=(uint8_t)(tmp_serial[1]>>8);
			buf_to_send[9]=(uint8_t)(tmp_serial[1]);
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case SEND_SERIAL_RAM_HIDDEN: //51
			repeat=false;
			buf_to_send[0]=0x51;
			buf_to_send[1]=0x01;
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			while(!(buf_state_test_tool_cmd[5]==0xa && buf_state_test_tool_cmd[6]==0xb))
			{
				ble_getTestToolCmd(buf_state_test_tool_cmd, TEST_TOOL_FRAME_SIZE);
				WDT_RR();
			}
			int ii=0;
			for(ii=0;ii<8;ii++)
			{
				serial_in_ram[ii]=buf_state_test_tool_cmd[ii+7];

			}
			RGB_left_set(COLOR_MAGENTA, 10);
			buf_to_send[0]=0x01;
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case SEND_SERIAL_FLASH_HIDDEN: //52
			repeat=false;
			buf_to_send[0]=0x52;
			buf_to_send[1]=0x01;
			 flash_internal_get_values((uint32_t *)INT_FLASH_SERIAL_HEADER,tmp_serial, 2);
			if (tmp_serial[0]!=0xFFFFFFFF || tmp_serial[1]!=0xFFFFFFFF)
				buf_to_send[2]=0x00;
			else
			{
				tmp_serial[0]=((serial_in_ram[0])<<24)+((serial_in_ram[1])<<16)+((serial_in_ram[2])<<8)+((serial_in_ram[3]));
				tmp_serial[1]=((serial_in_ram[4])<<24)+((serial_in_ram[5])<<16)+((serial_in_ram[6])<<8)+(serial_in_ram[7]);
				flash_internal_buf_write((uint32_t *)INT_FLASH_SERIAL_HEADER, tmp_serial, 2);
				ic_delay_ms(1);
				flash_internal_get_values((uint32_t *)INT_FLASH_SERIAL_HEADER,bleDevInfo.serial, 2);
				buf_to_send[2]=0x01;
			}
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case REMOVE_SERIAL_HIDDEN://53
			repeat=false;
			buf_to_send[0]=0x53;
			flash_internal_page_erase((uint32_t *)INT_FLASH_SERIAL_HEADER);
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case ELECTROTEST_ON://5e
			repeat=false;
			buf_to_send[0]=0x5E;
			buf_to_send[1]=0x01;
//opcja Y N ------------------
//			if(diff_p->electrode1>20 && diff_p->electrode2>20)
//				buf_to_send[2]=0x59;
//			else
//				buf_to_send[2]=0x4e;
//---------------------------
// opcja z bajtami
			buf_to_send[2]=(uint8_t)(diff_p->electrode1>>8);
			buf_to_send[3]=(uint8_t)(diff_p->electrode1);
			buf_to_send[4]=(uint8_t)(diff_p->electrode2>>8);
			buf_to_send[5]=(uint8_t)(diff_p->electrode2);
//---------------------
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			break;
		case ELECTRODE_START_MEASUREMENT: //2c
			repeat=false;
			electrotest_init();
			electrotest_start();
			break;
		case ELECTRODE_STOP_MEASUREMENT: //2e
			repeat=false;
			electrotest_stop();
			electrotest_deinit();
			break;
		case BQ_SHUTDOWN: //23
			repeat=false;
			bq27742_shutdown();
			break;
		case BQ_PROGRAMMING: //3a
			repeat=false;
			buf_to_send[0]=0x3a;
			buf_to_send[1]=0x01;
			ble_sendTestToolData(buf_to_send, TEST_TOOL_FRAME_SIZE);
			bq27742_program_flash();
			break;
                case TEST_FINISHED:
                case ALL_ST_CHECK:
                case SEND_SERIAL_RAM:
                case SEND_SERIAL_FLASH:
                case REMOVE_SERIAL:
                case BLUETOOTH_ON:
                case BLUETOOTH_OFF:
                case TEST_BLUETOOTH:
                        break;

	}
}

state_flag interpret_command_ble(uint8_t cmd)
{
	switch(cmd)
	{
		case 'a':
		case 'A':
			return LEDR1;
			break;
		case 'b':
		case 'B':
			return LEDG1;
			break;
		case 'c':
		case 'C':
			return LEDB1;
			break;
		case 'd':
		case 'D':
			return LEDR2;
			break;
		case 'e':
		case 'E':
			return LEDG2;
			break;
		case 'f':
		case 'F':
			return LEDB2;
			break;
		case 'g':
		case 'G':
			return POWER_LED;
			break;
		case 'h':
		case 'H':
			return POWER_LED;
			break;
		case 'i':
		case 'I':
			return LEDS_OFF;
			break;
		case 'j':
		case 'J':
			return ACC_ST;
			break;
		case 'k':
		case 'K':
			return AFE_ST;
			break;
		case 'l':
		case 'L':
			return AFE_ON;
			break;
		case 'm':
		case 'M':
			return AFE_OFF;
			break;
		case 'n':
		case 'N':
			return TEST_FINISHED;
			break;
		case 'o':
		case 'O':
			return POWER_LED_NO_REFRESH;
			break;
		case 'q':
			return BANDWIDTH_INFO;
			break;
		case 'r':
			return VIBRA_ON;
			break;
		case 's':
			return VIBRA_OFF;
			break;
		case 't':
		case 'T':
			return VIBRA_ST;
			break;
		case 'u':
		case 'U':
			return ACC_ON;
			break;
		case 'v':
		case 'V':
			return ALL_ST_CHECK;
			break;
		case 'w':
		case 'W':
			return ACC_OFF;
			break;
		case 'x':
		case 'X':
			return ACC_X;
			break;
		case 'y':
		case 'Y':
			return ACC_Y;
			break;
		case 'z':
		case 'Z':
			return ACC_Z;
			break;
		case '0':
			return BATT_TEMP;
			break;
		case '%':
			return SEND_SERIAL_RAM;
			break;
		case '(':
			return GET_SERIAL_RAM;
			break;
		case 0x51:
			return SEND_SERIAL_RAM_HIDDEN;
			break;
		case 0x52:
			return SEND_SERIAL_FLASH_HIDDEN;
			break;
		case 0x53:
			return REMOVE_SERIAL_HIDDEN;
			break;
		case '>':
			return GET_SERIAL_FLASH;
			break;
		case '<':
			return REMOVE_SERIAL;
			break;
		case '^':
			return ELECTROTEST_ON;
			break;
		case '&':
			return AFE_DIODE1;
			break;
		case '*':
			return AFE_DIODE2;
			break;
		case ',':
			return ELECTRODE_START_MEASUREMENT;
			break;
		case '.':
			return ELECTRODE_STOP_MEASUREMENT;
			break;
		case '#':
			return BQ_SHUTDOWN;
			break;
		case 0x3a:
			return BQ_PROGRAMMING;
			break;
	}
	return 0;
}


