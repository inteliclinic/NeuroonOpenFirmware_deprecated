/**
 * @file
 * @brief LIS3DH accelerometer driver methods
 */

#include "ic_lis3dh.h"
#ifdef USE_IC_LIS3DH
#ifndef USE_IC_I2C
	#define USE_IC_I2C
#endif

/**
 * @struct test_measurement
 * @brief measurement data structure for lis3dh testing
 */


/** global auxiliary variables */
static lis3dh_config acc_config;
static uint8_t clear_data = 0x00;

/**
 * @fn LIS3DH_Set_Mode ()
 * @brief set accelerometer module mode
 * @param mode configuration:
 * 0 - CLICK-CLICK MODE
 * 1 - MEASUREMENT_MODE
 * 2 - MEASUREMENT WITH FIFO MODE
 * 3 and >5 - MOVEMENT THRESHOLD MODE
 * 4 - like mode 1 but with SelfTest0
 * 5 - like mode 1 but with SelfTest1
 */

static int16_t mod(int16_t x) {
	return x>=0? x :-x;
}

lis3dh_config* LIS3DH_Set_Mode(uint8_t mode)
{
	if (mode == 0) {

		acc_config.reg1 = 0x8F;								/// Low power mode (1.6 KHz)
		acc_config.reg2 = 0x8C;								/// High pass filter normal mode for CLICK
		acc_config.reg3 = 0x80;								/// Interrupt with CLICK detection enabled
		acc_config.reg4 = 0x05;								/// FS = 2g; I2C interface
		acc_config.reg5 = 0x00;

		acc_config.fifo_ctrl = 0x00;

		acc_config.click_cfg = CLICK_CFG;					/// Single click detection on XYZ
		acc_config.click_threshold = CLICK_THS;				/// Single/Double click threshold
		acc_config.click_time_limit =CLICK_TIME_LIMIT;		/// Single/Double click time limit
		acc_config.click_time_window = CLICK_TIME_WINDOW;	/// CLICK-CLICK time window

		acc_config.int1_cfg = 0x00;
		acc_config.int1_duration = 0x00;
		acc_config.int1_threshold = 0x00;

		return &acc_config;
	}
	if (mode == 1) {

		acc_config.reg1 = 0x77;								/// Normal power mode (400 Hz)
		acc_config.reg2 = 0x00;								/// High-pass filter normal mode
		acc_config.reg3 = 0x00;								/// All interrupt disable
		acc_config.reg4 = 0x01;//HERE								/// FS = 2g; I2C interface
		acc_config.reg5 = 0x00;

		acc_config.fifo_ctrl = 0x00;

		acc_config.click_cfg = 0x00;
		acc_config.click_threshold = 0x00;
		acc_config.click_time_limit = 0x00;
		acc_config.click_time_window = 0x00;

		acc_config.int1_cfg = 0x00;
		acc_config.int1_duration = 0x00;
		acc_config.int1_threshold = 0x00;

		return &acc_config;
	}
	if (mode == 2) {

		acc_config.reg1 = 0x17;							/// Normal power mode (1 Hz)
		acc_config.reg2 = 0x00;							/// High-pass filter normal mode
		acc_config.reg3 = 0x04;							/// FIFO Watermark interrupt
		acc_config.reg4 = 0x01;							/// FS = 2g; I2C interface;
		acc_config.reg5 = 0x40;							/// FIFO enable

		acc_config.fifo_ctrl = 0x5D;					/// FIFO mode; Watermark = 29

		acc_config.click_cfg = 0x00;
		acc_config.click_threshold = 0x00;
		acc_config.click_time_limit = 0x00;
		acc_config.click_time_window = 0x00;

		acc_config.int1_cfg = 0x00;
		acc_config.int1_duration = 0x00;
		acc_config.int1_threshold = 0x00;

		return &acc_config;
	}
	if (mode == 3 ) {
		acc_config.reg1 = 0x57;						/// Normal power mode (100 Hz)
		acc_config.reg2 = 0x01;						/// High pass filter normal mode for AOI int1
		acc_config.reg3 = 0x40;						/// Interrupt with AOI1 detection enabled
		acc_config.reg4 = 0x01;						/// FS = 2g; I2C interface //??????WHY THIS 1?????????????
		acc_config.reg5 = 0x00;

		acc_config.fifo_ctrl = 0x00;

		acc_config.click_cfg = 0x00;
		acc_config.click_threshold = 0x00;
		acc_config.click_time_limit = 0x00;
		acc_config.click_time_window = 0x00;

		acc_config.int1_cfg = INT1_CFG;
		acc_config.int1_duration = INT1_THS;
		acc_config.int1_threshold = INT1_DURATION;

		return &acc_config;
	}
	if (mode == 4) {

		acc_config.reg1 = 0x77;								/// Normal power mode (400 Hz)
		acc_config.reg2 = 0x00;								/// High-pass filter normal mode
		acc_config.reg3 = 0x00;								/// All interrupt disable
		acc_config.reg4 = 0x03;//HERE								/// FS = 2g; I2C interface
		acc_config.reg5 = 0x00;

		acc_config.fifo_ctrl = 0x00;

		acc_config.click_cfg = 0x00;
		acc_config.click_threshold = 0x00;
		acc_config.click_time_limit = 0x00;
		acc_config.click_time_window = 0x00;

		acc_config.int1_cfg = 0x00;
		acc_config.int1_duration = 0x00;
		acc_config.int1_threshold = 0x00;

		return &acc_config;
	}
	if (mode == 5) {

		acc_config.reg1 = 0x77;								/// Normal power mode (400 Hz)
		acc_config.reg2 = 0x00;								/// High-pass filter normal mode
		acc_config.reg3 = 0x00;								/// All interrupt disable
		acc_config.reg4 = 0x05;//HERE								/// FS = 2g; I2C interface
		acc_config.reg5 = 0x00;

		acc_config.fifo_ctrl = 0x00;

		acc_config.click_cfg = 0x00;
		acc_config.click_threshold = 0x00;
		acc_config.click_time_limit = 0x00;
		acc_config.click_time_window = 0x00;

		acc_config.int1_cfg = 0x00;
		acc_config.int1_duration = 0x00;
		acc_config.int1_threshold = 0x00;

		return &acc_config;
	}
	if (mode == 6) {

		acc_config.reg1 = 0x47;	//77						/// Normal power mode (400 Hz)
		acc_config.reg2 = 0x00;								/// High-pass filter normal mode
		acc_config.reg3 = 0x80;
		acc_config.reg4 = 0x01;//HERE								/// FS = 2g; I2C interface
		acc_config.reg5 = 0x00;

		acc_config.fifo_ctrl = 0x00;

		acc_config.click_cfg = CLICK_CFG;					/// Single click detection on XYZ
		acc_config.click_threshold = CLICK_THS;				/// Single/Double click threshold
		acc_config.click_time_limit =CLICK_TIME_LIMIT;		/// Single/Double click time limit
		acc_config.click_time_latency=CLICK_TIME_LATENCY;
		acc_config.click_time_window = CLICK_TIME_WINDOW;	/// CLICK-CLICK time window


		acc_config.int1_cfg = 0x00;
		acc_config.int1_duration = 0x00;
		acc_config.int1_threshold = 0x00;

//		acc_config.reg2 = 0x8C;								/// High pass filter normal mode for CLICK

		return &acc_config;
	}
	else {
		acc_config.reg1 = 0x57;						/// Normal power mode (100 Hz)
		acc_config.reg2 = 0x01;						/// High pass filter normal mode for AOI int1
		acc_config.reg3 = 0x40;						/// Interrupt with AOI1 detection enabled
		acc_config.reg4 = 0x01;						/// FS = 2g; I2C interface //??????WHY THIS 1?????????????
		acc_config.reg5 = 0x00;

		acc_config.fifo_ctrl = 0x00;

		acc_config.click_cfg = 0x00;
		acc_config.click_threshold = 0x00;
		acc_config.click_time_limit = 0x00;
		acc_config.click_time_window = 0x00;

		acc_config.int1_cfg = INT1_CFG;
		acc_config.int1_duration = INT1_THS;
		acc_config.int1_threshold = INT1_DURATION;

		return &acc_config;
	}
}

/**
 * @fn LIS3DH_Config()
 * @brief accelerometer module initialization via I2C
 * @param pointer do configuration data
 * @return true if communication correct
 */
bool LIS3DH_Config(lis3dh_config *data)
{
	bool lis_error = false;
	uint8_t who;


	/** Check communication I2C */
	TWI_ReadReg(TWI_ACC_ADDRESS,LIS3DH_WHO_AM_I,&who,1);

		if (who == 0x33) {
			lis_error = true;
		}
		else {
			lis_error = false;
		}

	//Registers configuration
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_CTRL_REG1, &data->reg1,1);					// Power mode, date rate selection; axis enable
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_CTRL_REG2, &data->reg2,1);					// Filter configuration

	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_CTRL_REG3, &data->reg3,1);					// Interrupt configuration (clear all interrupt)
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_CTRL_REG4, &data->reg4,1);					// Scale, interface mode selection
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_CTRL_REG5, &data->reg5,1);					// FIFO enable

	//TODO HACK init fifo
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_FIFO_CTRL_REG, &clear_data,1);				// FIFO configuration
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_FIFO_CTRL_REG, &data->fifo_ctrl,1);			// FIFO configuration

	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_CLICK_CFG, &data->click_cfg,1);				// Click-Click configuration
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_CLICK_THS, &data->click_threshold,1);		// CLICK-CLICK click threshold
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_TIME_LIMIT, &data->click_time_limit,1);		// CLICK-CLICK click time limit
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_TIME_LIMIT, &data->click_time_latency,1);		// CLICK-CLICK click time limit
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_TIME_WINDOW, &data->click_time_window,1);	// CLICK-CLICK time window


	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_INT1_CFG, &data->int1_cfg,1);				// Movement configuration
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_INT1_DURATION, &data->int1_duration,1);		// Movement event duration
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_INT1_THS, &data->int1_threshold,1);			// Movement sensitivity

	return lis_error;
}

/**
 * @fn LIS3DH_INT1_Config ()
 * @brief interrupt 1 configuration as GPIOTE
 */
void LIS3DH_INT1_Config ()
{
	/** GIPO->INT 1 */
	NRF_GPIO->PIN_CNF[GPIO_INT1_ACC_PIN] =
				(GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos)
		      |	(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
		      | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
		      | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
		      | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIOTE->EVENTS_PORT = 0;
	//NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Set << GPIOTE_INTENSET_PORT_Pos;
}

/**
 * @fn LIS3DH_INT1_Deinit ()
 * @brief interrupt 1 deinit
 */
void LIS3DH_INT1_Deinit ()
{
	NRF_GPIO->PIN_CNF[GPIO_INT1_ACC_PIN] = 0x00;
	//NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_PORT_Clear << GPIOTE_INTENCLR_PORT_Pos;
}

/**
 * @fn LIS3DH_Start ()
 * @brief accelerometer module start measurement by interrupt enable
 */
void LIS3DH_Start ()
{
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_CTRL_REG3, &acc_config.reg3,1);
}

/**
 * @fn LIS3DH_Stop ()
 * @brief accelerometer module stop measurement by interrupt disable
 */
void LIS3DH_Stop ()
{
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_CTRL_REG3, &clear_data,1);
}

/**
 * @fn LIS3DH_Fifo_Clear ()
 * @brief clear internal FIFO
 */
void LIS3DH_Fifo_Clear ()
{
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_FIFO_CTRL_REG, &clear_data,1);
	TWI_WriteReg(TWI_ACC_ADDRESS,LIS3DH_FIFO_CTRL_REG, &acc_config.fifo_ctrl,1);
}

/**
* @fn app_lis3dh_config ()
* @brief getter to acc config structure
* @return acc config structure pointer
*/
lis3dh_config_p app_lis3dh_config ()
{
	return &acc_config;
}

/** @fn LIS3DH_getTestData()
* @brief read data from axis X,Y,Z
* @return measurement data structure
*/
void LIS3DH_getTestData(measurement *acc_axis) {
	TWI_ReadReg(TWI_ACC_ADDRESS, LIS3DH_X_LSB, &(acc_axis->ACC_DATA_REG[0]),1);
	TWI_ReadReg(TWI_ACC_ADDRESS, LIS3DH_X_MSB, &(acc_axis->ACC_DATA_REG[1]),1);

	TWI_ReadReg(TWI_ACC_ADDRESS, LIS3DH_Y_LSB, &(acc_axis->ACC_DATA_REG[2]),1);
	TWI_ReadReg(TWI_ACC_ADDRESS, LIS3DH_Y_MSB, &(acc_axis->ACC_DATA_REG[3]),1);

	TWI_ReadReg(TWI_ACC_ADDRESS, LIS3DH_Z_LSB, &(acc_axis->ACC_DATA_REG[4]),1);
	TWI_ReadReg(TWI_ACC_ADDRESS, LIS3DH_Z_MSB, &(acc_axis->ACC_DATA_REG[5]),1);

	acc_axis->ACC_X = (int16_t)(acc_axis->ACC_DATA_REG[1]<<8) | (int16_t)(acc_axis->ACC_DATA_REG[0]);
	acc_axis->ACC_Y = (int16_t)(acc_axis->ACC_DATA_REG[3]<<8) | (int16_t)(acc_axis->ACC_DATA_REG[2]);
	acc_axis->ACC_Z = (int16_t)(acc_axis->ACC_DATA_REG[5]<<8) | (int16_t)(acc_axis->ACC_DATA_REG[4]);
}

static int16_t mean5(int16_t *data){
	return (data[0]+data[1]+data[2]+data[3]+data[4])/5;
}

int16_t LIS3DH_Self_Test(uint8_t *buffer){
#define MESURE_LENGTH 5
	bool error_init = false;
	measurement *data, tmp;
	int16_t x_NM[5],y_NM[5],z_NM[5], x_ST0[5],y_ST0[5],z_ST0[5], x_ST1[5],y_ST1[5],z_ST1[5];
	int16_t x_mean_NM=0, y_mean_NM=0, z_mean_NM=0, x_mean_ST0=0, y_mean_ST0=0, z_mean_ST0=0, x_mean_ST1=0, y_mean_ST1=0, z_mean_ST1=0;
	volatile uint8_t i=0;
	data = &tmp;
	volatile uint8_t timeout_count=0;
	bool test_failed_1=true;
	bool test_failed_4=true;
	bool test_failed_5=true;

	for(i=0;i<5;i++)
	{
		x_NM[i]=0;
		y_NM[i]=0;
		z_NM[i]=0;
		x_ST0[i]=0;
		y_ST0[i]=0;
		z_ST0[i]=0;
		x_ST0[i]=0;
		y_ST0[i]=0;
		z_ST0[i]=0;
	}

	while (error_init != true && timeout_count<=10)
	{
		error_init = LIS3DH_Config(LIS3DH_Set_Mode(1));
		timeout_count++;
		if(error_init==true)
			test_failed_1=false;
	}
	timeout_count=0;

	if(test_failed_1==false)
	{
		ic_delay_ms(1);
		for (i=0; i<MESURE_LENGTH; ++i){
			ic_delay_ms(4);
			LIS3DH_getTestData(data);
			x_NM[i] = data->ACC_X;
			y_NM[i] = data->ACC_Y;
			z_NM[i] = data->ACC_Z;
		}
	}

	error_init=false;
	while (error_init != true && timeout_count<=10)
	{
		error_init = LIS3DH_Config(LIS3DH_Set_Mode(4));
		timeout_count++;
		if(error_init==true)
			test_failed_4=false;
	}
	timeout_count=0;

	if(test_failed_4==false)
	{
		ic_delay_ms(1);
		for (i=0; i<MESURE_LENGTH; ++i){
			ic_delay_ms(4);
			LIS3DH_getTestData(data);
			x_ST0[i] = data->ACC_X;
			y_ST0[i] = data->ACC_Y;
			z_ST0[i] = data->ACC_Z;
		}
	}

	error_init=false;
	while (error_init != true && timeout_count<=10)
	{
		error_init = LIS3DH_Config(LIS3DH_Set_Mode(5));
		timeout_count++;
		if(error_init==true)
			test_failed_5=false;
	}
	timeout_count=0;

	if(test_failed_5==false)
	{
		ic_delay_ms(1);
		for (i=0; i<MESURE_LENGTH; ++i){
			ic_delay_ms(4);
			LIS3DH_getTestData(data);
			x_ST1[i] = data->ACC_X;
			y_ST1[i] = data->ACC_Y;
			z_ST1[i] = data->ACC_Z;
		}
	}

	x_mean_NM=mean5(x_NM);
	y_mean_NM=mean5(y_NM);
	z_mean_NM=mean5(z_NM);

	x_mean_ST0=mean5(x_ST0);
	y_mean_ST0=mean5(y_ST0);
	z_mean_ST0=mean5(z_ST0);

	x_mean_ST1=mean5(x_ST1);
	y_mean_ST1=mean5(y_ST1);
	z_mean_ST1=mean5(z_ST1);

//	x_mean_NM=0;
//	y_mean_NM=0;
//	z_mean_NM=0;
//
//	x_mean_ST0=0;
//	y_mean_ST0=0;
//	z_mean_ST0=0;
//
//	x_mean_ST1=0;
//	y_mean_ST1=0;
//	z_mean_ST1=0;


	if(test_failed_1==false)
	{
		if(x_mean_ST0>x_mean_NM && x_mean_NM>x_mean_ST1)
		{
			buffer[0]=x_mean_NM>>8;
			buffer[1]=x_mean_NM;
			buffer[2]=x_mean_ST0>>8;
			buffer[3]=x_mean_ST0;
			buffer[4]=x_mean_ST1>>8;
			buffer[5]=x_mean_ST1;
		}
		else
		{//10000
			buffer[0]=0x27;
			buffer[1]=0x10;
			buffer[2]=0;
			buffer[3]=0;
			buffer[4]=0;
			buffer[5]=0;
		}
	}
	else
	{// 11111; 0; 0;
		buffer[0]=0x2b;
		buffer[1]=0x67;
		buffer[2]=0;
		buffer[3]=0;
		buffer[4]=0;
		buffer[5]=0;
	}
	if(test_failed_4==false)
	{
		if(y_mean_ST0>y_mean_NM && y_mean_NM>y_mean_ST1)
		{
			buffer[6]=y_mean_NM>>8;
			buffer[7]=y_mean_NM;
			buffer[8]=y_mean_ST0>>8;
			buffer[9]=y_mean_ST0;
			buffer[10]=y_mean_ST1>>8;
			buffer[11]=y_mean_ST1;
		}
		else
		{//20000
			buffer[6]=0x4E;
			buffer[7]=0x20;
			buffer[8]=0;
			buffer[9]=0;
			buffer[10]=0;
			buffer[11]=0;
		}
	}
	else
	{// 22222; 0; 0;
		buffer[6]=0x56;
		buffer[7]=0xce;
		buffer[8]=0;
		buffer[9]=0;
		buffer[10]=0;
		buffer[11]=0;
	}

	if(test_failed_5==false)
	{
		if(z_mean_ST0>z_mean_NM && z_mean_NM>z_mean_ST1)
		{
			buffer[12]=z_mean_NM>>8;
			buffer[13]=z_mean_NM;
			buffer[14]=z_mean_ST0>>8;
			buffer[15]=z_mean_ST0;
			buffer[16]=z_mean_ST1>>8;
			buffer[17]=z_mean_ST1;
		}
		else
		{//30000
			buffer[12]=0x75;
			buffer[13]=0x30;
			buffer[14]=0;
			buffer[15]=0;
			buffer[16]=0;
			buffer[17]=0;
		}
	}
	else
	{// 32000; 0; 0;
		buffer[12]=0x7d;
		buffer[13]=0x00;
		buffer[14]=0;
		buffer[15]=0;
		buffer[16]=0;
		buffer[17]=0;
	}

	return 0;
	//x_NM x_ST0 x_ST1   xxxyyyzzz
}

int32_t LIS3DH_amplitude(int length){
	measurement *data, tmp;
	data=&tmp;
	int16_t x,y,z;
	int i;
	int32_t minx=INT_LEAST32_MAX, maxx=0;
	int32_t miny=INT_LEAST32_MAX, maxy=0;
	int32_t minz=INT_LEAST32_MAX, maxz=0;

	for (i=0; i<length;++i){
		LIS3DH_getTestData(data);
		x = data->ACC_X;
		y = data->ACC_Y;
		z = data->ACC_Z;
		if (x<minx) minx=x;
		if (x>maxx) maxx=x;
		if (y<miny) miny=y;
		if (y>maxy) maxy=y;
		if (z<minz) minz=z;
		if (z>maxz) maxz=z;
	}
	return (maxx-minx)+(maxy-miny)+(maxz-minz);
}

#endif
