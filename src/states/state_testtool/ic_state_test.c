/*
 * ic_state_test.c
 *
 *  Created on: 7 paź 2015
 *      Author: pb
 */
#include "ic_state_test.h"
#include "ic_uart.h"
#include "ic_electrotest.h"
#include "ic_flash.h"
#include "ic_flash_map.h"
#include "ic_lis3dh.h"
#include "ic_ltc3220.h"
#include <ic_eeg.h>
#include "ic_timers.h"
#include "ic_bluetooth.h"
#include <ic_flash_enclosed.h>

int32_t test_afe_data[2];
volatile measurement test_lis_data;
measurement *data, data_accelerometer;
volatile uint8_t data_bq[8];

volatile uint8_t ble_on=false;
volatile uint8_t serial_in_ram[8]={0,0,0,0,0,0,0,0};
uint32_t tmp_serial[2];


#define CONNECTED_SIGNAL_THRESHOLD 30
#define TEST_BLE_FRAME_SIZE 20

void state_test_init(void)
{
	uart_send_u8(0x49);

	sd_nvic_DisableIRQ(UART0_IRQn);

	TWI_PIN_SELECT(GPIO_SCL_PIN, GPIO_SDA_PIN);
	TWI_Init(K400);
	ResetBit(GPIO_POWER_DIGITAL);
	ic_delay_ms(100);
	SetBit(GPIO_POWER_DIGITAL);
	SetBit(GPIO_POWER_ANALOG);

	while(LIS3DH_Config(LIS3DH_Set_Mode(1))==false)
	{
		uart_send_u8(0x99);
		WDT_RR();
	}
	LIS3DH_INT1_Config();
	bq27742_init();
	FLASH_ENCLOSED_init();

	ads_init();
//	timer_startSection(SECTION_EEG);
//	timer_busInit(125);

	/** AFE4400 init */
	SetBit(GPIO_AFE_PDN);
	afe4400_start();
	afe4400_get_config()->led1_current = 0x3A;
	afe4400_get_config()->led2_current = 0x3A;
	afe4400_get_config()->rf_led = 0x05;
	afe4400_get_config()->cf_led = 0x02;
	afe4400_get_config()->gain = 0x04;
	afe4400_init();
	afe4400_stop();
	electrotest_init();
	electrotest_stop();
	data = &data_accelerometer;
	diff_p = &diff;
}

void state_test_loop(void)
{

	bool acc_running=false;
	bool koniec=false;
	uint8_t buf=(uint8_t)(NRF_UART0->RXD);
	bool repeat=false;
	state_flag current_flag;
	while(1)
	{

		if(NRF_UART0->EVENTS_RXDRDY != 0)
		{
			NRF_UART0->EVENTS_RXDRDY = 0;
			buf = (uint8_t)(NRF_UART0->RXD);
			uart_send_u8(buf);
			ic_delay_ms(10);
			current_flag=interpret_command(buf);
			repeat=true;
		}
		if(repeat)
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
				ic_delay_ms(5);
				uart_send_u8(0x01);
				repeat=false;
				break;
			case POWER_LED_NO_REFRESH://6F
				power_led_flash_test();
				ic_delay_ms(5);
				break;
			case LEDS_OFF: //69
				ResetBit(GPIO_LEDS_ON);
				uart_send_u8(0x01);
				repeat=false;
				break;
			case AFE_ST: //6B
				repeat=false;
				uint32_t tmp;
				tmp = afe4400_SelfTest();
				uart_send_u8((uint8_t) (tmp>>8));
				ic_delay_ms(5);
				uart_send_u8((uint8_t) tmp);
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case VIBRA_ST: //74
//opcja Y N
//				repeat=false;
//				volatile int res;
//				volatile uint8_t passed=false;
//				SetBit(GPIO_LEDS_ON);
//				res=test_vibrator_st();
//				if(res>20000)
//					passed=true;

//				if(passed)
//					uart_send_u8(0x59);//Y
//				else
//					uart_send_u8(0x4e);//N
//				ic_delay_ms(5);
//				uart_send_u8(0x01);
//				break;
//opcja ze zwracaniem wartości
				repeat=false;
				volatile int res=0;
				SetBit(GPIO_LEDS_ON);
				res=test_vibrator_st();
				uart_send_u8((uint8_t)(res>>24));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(res>>16));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(res>>8));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(res));
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case ALL_ST_CHECK: //76
				repeat=false;
				bq27742_read_measurement_data((uint8_t *)data_bq);
				if(data_bq[2]!=0||data_bq[3]!=0)
					if(data_bq[1]!=0)
					{
//						if(LIS3DH_Self_Test()==0)
//						{
							ads_init();
							ads_power_up();
							if(ads_get_value()!=0)
							{
								if(afe4400_SelfTest()==0)
									if(!ltc3220_init())
										if(data_bq[0]!=0||data_bq[1]!=0)
											uart_send_u8(0x59);//Y
										else
										uart_send_u8(0x4E);//N
									else
									uart_send_u8(0x4E);//N
								else
								uart_send_u8(0x4E);//N
								ads_power_down();
							}
							else
							uart_send_u8(0x4E);//N
						}
						else
						uart_send_u8(0x4E);//N
					//else
					//uart_send_u8(0x4E);//N
				else
				uart_send_u8(0x4E);//N

				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case AFE_ON: //6C
				repeat=false;
				afe4400_start();
				uart_send_u8(0x01);
				break;
			case AFE_OFF: //6D
				repeat=false;
				afe4400_stop();
				uart_send_u8(0x01);
				break;
			case AFE_DIODE1: //26
				repeat=false;
				afe4400_TakeData(test_afe_data);
				uart_send_u8((uint8_t)(test_afe_data[0]>>24));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(test_afe_data[0]>>16));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(test_afe_data[0]>>8));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(test_afe_data[0]));
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case AFE_DIODE2: //2A
				repeat=false;
				afe4400_TakeData(test_afe_data);
				uart_send_u8((uint8_t)(test_afe_data[1]>>24));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(test_afe_data[1]>>16));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(test_afe_data[1]>>8));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(test_afe_data[1]));
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case BANDWIDTH_INFO: //71
				repeat=false;
				//zwraca informacje o udziałach w pasmach
				uart_send_u8(0x01);
				eeg_clearModule();
				timer_busInit(125);
				timer_startSection(SECTION_EEG);
				while (eeg_getComputedEpochSamplesNumber()!=500)
				{
					if (eeg_computeNextSampleForTesttooling())
					{
						if(eeg_getComputedEpochSamplesNumber()==500)
						{
							EEG_TesttoolingEpoch tmptmp=eeg_getTesttoolingEpoch();
							uart_send_u8(tmptmp.partAlpha);
							ic_delay_ms(5);
							uart_send_u8(tmptmp.partBeta);
							ic_delay_ms(5);
							uart_send_u8(tmptmp.partDelta);
							ic_delay_ms(5);
							uart_send_u8(tmptmp.partSpindles);
							ic_delay_ms(5);
							uart_send_u8(tmptmp.partTheta);
						}
					}
					WDT_RR();
				}
				timer_busDeinit();
				ic_delay_ms(5);
				uart_send_u8(0x01);
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
				uart_send_u8(0x01);
				break;
			case ACC_OFF: //77
				repeat=false;
				LIS3DH_Stop();
				acc_running=false;
				uart_send_u8(0x01);
				break;
			case ACC_X: //78
				repeat=false;
				ic_delay_ms(5);
				uart_send_u8(data->ACC_DATA_REG[0]);
				ic_delay_ms(5);
				uart_send_u8(data->ACC_DATA_REG[1]);
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case ACC_Y: //79
				repeat=false;
				ic_delay_ms(5);
				uart_send_u8(data->ACC_DATA_REG[2]);
				ic_delay_ms(5);
				uart_send_u8(data->ACC_DATA_REG[3]);
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case ACC_Z: //7A
				repeat=false;
				ic_delay_ms(5);
				uart_send_u8(data->ACC_DATA_REG[4]);
				ic_delay_ms(5);
				uart_send_u8(data->ACC_DATA_REG[5]);
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case ACC_ST: //6A
				repeat=false;
				uint8_t buffer[18]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
				LIS3DH_Self_Test(buffer);
				uint8_t licznik=0;
				for(licznik=0;licznik<18;licznik++)
				{
					ic_delay_ms(5);
					uart_send_u8(buffer[licznik]);
				}
//				if (LIS3DH_Self_Test()==0)
//					uart_send_u8(0x59);//Y
//				else
//					uart_send_u8(0x4E);//N
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case BATT_TEMP: //30
				repeat=false;
				bq27742_read_measurement_data((uint8_t *)data_bq);
				uart_send_u8((uint8_t)(data_bq[0]));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(data_bq[1]));
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case SEND_SERIAL_RAM: // 25
				uart_send_u8(0x01);
				repeat=false;
				int end=0;
				int i=0;
				while(!end)
				{
					if(NRF_UART0->EVENTS_RXDRDY != 0)
					{
						NRF_UART0->EVENTS_RXDRDY = 0;
						serial_in_ram[i++] = (uint8_t)(NRF_UART0->RXD);
					}
					if(i==8)
					end=1;
					WDT_RR();
				}
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case GET_SERIAL_RAM: //28
				repeat=false;
//				ic_delay_ms(5);
//				uart_send_u8(0x01);
				for(i=0; i<8; i++)
				{
					ic_delay_ms(5);
					uart_send_u8( serial_in_ram[i]);
				}
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case SEND_SERIAL_FLASH: //29
				repeat=false;
				 flash_internal_get_values((uint32_t *)INT_FLASH_SERIAL_HEADER,tmp_serial, 2);
				if (tmp_serial[0]!=0xFFFFFFFF || tmp_serial[1]!=0xFFFFFFFF)
				{
					ic_delay_ms(5);
					uart_send_u8(0x00);
				}
				else
				{
					tmp_serial[0]=((serial_in_ram[0])<<24)+((serial_in_ram[1])<<16)+((serial_in_ram[2])<<8)+((serial_in_ram[3]));
					tmp_serial[1]=((serial_in_ram[4])<<24)+((serial_in_ram[5])<<16)+((serial_in_ram[6])<<8)+(serial_in_ram[7]);
					flash_internal_buf_write((uint32_t*)INT_FLASH_SERIAL_HEADER, tmp_serial, 2);
					ic_delay_ms(5);
					uart_send_u8(0x01);
				}
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case GET_SERIAL_FLASH: //3e
				repeat=false;
				flash_internal_get_values((uint32_t*)INT_FLASH_SERIAL_HEADER,tmp_serial, 2);
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(tmp_serial[0]>>24));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(tmp_serial[0]>>16));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(tmp_serial[0]>>8));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(tmp_serial[0]));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(tmp_serial[1]>>24));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(tmp_serial[1]>>16));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(tmp_serial[1]>>8));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(tmp_serial[1]));
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case REMOVE_SERIAL://3C
				repeat=false;
				flash_internal_page_erase((uint32_t*)INT_FLASH_SERIAL_HEADER);
				uart_send_u8(0x01);
				break;
			case ELECTROTEST_ON://5e działa po 2c
				repeat=false;
				ic_delay_ms(5);
				uart_send_u8(0x01);
				ic_delay_ms(5);

//wartości max dla wlącoznych rezystorów były w FT 0x22, 0x23
//włączyć elektrotest!!!
//opcja Y N
//				if(diff_p->electrode1>20 && diff_p->electrode2>20)
//					uart_send_u8(0x59);
//				else
//					uart_send_u8(0x4e);
//----------------------------
//opcja z bajtami ---------------------------
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(diff_p->electrode1>>8));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(diff_p->electrode1));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(diff_p->electrode2>>8));
				ic_delay_ms(5);
				uart_send_u8((uint8_t)(diff_p->electrode2));
//--------------------------
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
			case ELECTRODE_START_MEASUREMENT: //2c
				repeat=false;
				electrotest_init();//ok
				electrotest_start();
				uart_send_u8(0x01);
				break;
			case ELECTRODE_STOP_MEASUREMENT: //2e
				repeat=false;
				electrotest_stop();
				electrotest_deinit();
				uart_send_u8(0x01);
				break;
			case BQ_SHUTDOWN: //23
				repeat=false;
				bq27742_shutdown();
				uart_send_u8(0x01);
				break;
			case TEST_FINISHED:
				repeat=false;
				koniec=true;
				break;
			case BLUETOOTH_ON: //2f
				repeat=false;
				if(!ble_on)
				{
					ble_enableRadioCommunication();
					for(int i =0;i<100;i++)
					{
						if(ble_isCentralConnected())
						{
							ble_on=true;

							break;
						}
						ic_delay_ms(50);
						WDT_RR();
					}
					if(ble_on)
					{
						ic_delay_ms(5);
						uart_send_u8(0x59);//Y
						ic_delay_ms(5);
						uart_send_u8(0x01);
					}
					else
					{
						ic_delay_ms(5);
						uart_send_u8(0x4E);//N
						ic_delay_ms(5);
						uart_send_u8(0x01);
					}
				}
				else
				{
					ic_delay_ms(5);
					uart_send_u8(0x4E);//N
					ic_delay_ms(5);
					uart_send_u8(0x01);
				}
				break;
			case BLUETOOTH_OFF:
				repeat=false;
				if(ble_on)
				{
					ble_disableRadioCommunication();
					for(int i =0;i<100;i++)
					{
						if(!ble_isCentralConnected())
						{
							ble_on=false;
							break;
						}
						ic_delay_ms(15);
					}
					if(!ble_on)
					{
						ic_delay_ms(5);
						uart_send_u8(0x59);//Y
						ic_delay_ms(5);
						uart_send_u8(0x01);
					}
					else
					{
						ic_delay_ms(5);
						uart_send_u8(0x4E);//N
						ic_delay_ms(5);
						uart_send_u8(0x01);
					}
				}
				else
				{
					ic_delay_ms(5);
					uart_send_u8(0x4E);
					ic_delay_ms(5);
					uart_send_u8(0x01);
				}
				break;
			case TEST_BLUETOOTH:
				repeat=false;
				uint8_t buf_state_test_tool_cmd[TEST_BLE_FRAME_SIZE];
				if(ble_on)
				{
					while(!ble_getTestToolCmd(buf_state_test_tool_cmd, TEST_BLE_FRAME_SIZE))
					WDT_RR();

					//buf_state_test_tool_cmd[0]=0x59;
					ble_sendTestToolData(buf_state_test_tool_cmd, TEST_BLE_FRAME_SIZE);
				}
				break;
			case BQ_PROGRAMMING:
				repeat=false;
				bq27742_program_flash();
				ic_delay_ms(5);
				uart_send_u8(0x01);
				break;
                        case SEND_SERIAL_RAM_HIDDEN:
                        case SEND_SERIAL_FLASH_HIDDEN:
                        case REMOVE_SERIAL_HIDDEN:
                                break;
		}
		WDT_RR();
		if(koniec)
		{
			uart_send_u8(0x01);
			return;
		}
		if(acc_running){
			LIS3DH_getTestData(data);
		}
	}
}

int test_vibrator_st(void)
{
		bool error_init;
		volatile int32_t bef, aft, diff;
		do error_init = LIS3DH_Config(LIS3DH_Set_Mode(1));
		while (error_init != true);

		bef = LIS3DH_amplitude(10);
		SetBit(GPIO_LEDS_ON);
		ic_delay_ms(5);
		ltc3220_set_chargepump(2);
		ltc3220_set_brightness(VIBRATOR1_CHANNEL1,0x3f);
		ltc3220_set_brightness(VIBRATOR1_CHANNEL2,0x3f);
		ic_delay_ms(500);
		aft = LIS3DH_amplitude(10);
		ResetBit(GPIO_LEDS_ON);
		diff=aft-bef;
		diff=(diff>0 ? diff : -diff);
		return diff;
}

state_flag interpret_command(uint8_t command)
{
	switch(command)
	{
		case 'a':
		case 'A':
			uart_send_u8(0x01);
			return LEDR1;
			break;
		case 'b':
		case 'B':
			uart_send_u8(0x01);
			return LEDG1;
			break;
		case 'c':
		case 'C':
			uart_send_u8(0x01);
			return LEDB1;
			break;
		case 'd':
		case 'D':
			uart_send_u8(0x01);
			return LEDR2;
			break;
		case 'e':
		case 'E':
			uart_send_u8(0x01);
			return LEDG2;
			break;
		case 'f':
		case 'F':
			uart_send_u8(0x01);
			return LEDB2;
			break;
		case 'g':
		case 'G':
			//uart_send_u8(0x01);
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
			uart_send_u8(0x01);
			return ACC_ST;
			break;
		case 'k':
		case 'K':
			uart_send_u8(0x01);
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
			uart_send_u8(0x01);
			return POWER_LED_NO_REFRESH;
			break;
		case 'q':
		case 'Q':
			return BANDWIDTH_INFO;
			break;
		case 'r':
		case 'R':
			uart_send_u8(0x01);
			return VIBRA_ON;
			break;
		case 's':
		case 'S':
			uart_send_u8(0x01);
			return VIBRA_OFF;
			break;
		case 't':
		case 'T':
			uart_send_u8(0x01);
			return VIBRA_ST;
			break;
		case 'u':
		case 'U':
			return ACC_ON;
			break;
		case 'v':
		case 'V':
			uart_send_u8(0x01);
			return ALL_ST_CHECK;
			break;
		case 'w':
		case 'W':
			return ACC_OFF;
			break;
		case 'x':
		case 'X':
			uart_send_u8(0x01);
			return ACC_X;
			break;
		case 'y':
		case 'Y':
			uart_send_u8(0x01);
			return ACC_Y;
			break;
		case 'z':
		case 'Z':
			uart_send_u8(0x01);
			return ACC_Z;
			break;
		case '0':
			uart_send_u8(0x01);
			return BATT_TEMP;
			break;
		case '%':
			return SEND_SERIAL_RAM;
			break;
		case '(':
			uart_send_u8(0x01);
			return GET_SERIAL_RAM;
			break;
		case ')':
			uart_send_u8(0x01);
			return SEND_SERIAL_FLASH;
			break;
		case '>':
			uart_send_u8(0x01);
			return GET_SERIAL_FLASH;
			break;
		case '<':
			return REMOVE_SERIAL;
			break;
		case '^':
			return ELECTROTEST_ON;
			break;
		case '&':
			uart_send_u8(0x01);
			return AFE_DIODE1;
			break;
		case '*':
			uart_send_u8(0x01);
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
		case '/': //2f
			uart_send_u8(0x01);
			return BLUETOOTH_ON;
			break;
		case '\\':
			uart_send_u8(0x01);
			return BLUETOOTH_OFF;
			break;
		case '|':
			uart_send_u8(0x01);
			return TEST_BLUETOOTH;
			break;
		case ':':
			return BQ_PROGRAMMING;
			break;
	}
	return 0;
}

void state_test_deinit(void)
{
	sd_nvic_SystemReset();
}

