/**
 * @file
 * @brief I2C communication methods
 */

#include "ic_i2c.h"
#ifdef USE_IC_I2C
#ifndef USE_IC_DELAY
	#define USE_IC_DELAY
	#include "ic_delay.h"
#endif


/** general I2C pin struct */
twi_pin_config pin_select;

/** global twi frequency value */
twi_clk_freq twi_freq_global;

/**
 * @fn TWI_PIN_SELECT ()
 * @brief Select GPIO pin for i2c bus
 * @param scl_pin
 * @param sda_pin
 */
void TWI_PIN_SELECT (uint8_t scl_pin, uint8_t sda_pin){
	pin_select.gpio_scl_pin = scl_pin;
	pin_select.gpio_sda_pin = sda_pin;
}

/**
 * @fn TWI_Init()
 * @brief initialization I2C bus
 */
void TWI_Init (twi_clk_freq clk_freq)
{
	///GPIO->SCL
	NRF_GPIO->PIN_CNF[pin_select.gpio_scl_pin] =
				(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
		      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
		      | (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)
		      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
		      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);
	///GPIO->SDA
	NRF_GPIO->PIN_CNF[pin_select.gpio_sda_pin] =
		        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
		      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
		      | (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)
		      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
		      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);

	TWI_I2C->POWER        	  = 1;
	TWI_I2C->EVENTS_RXDREADY = 0;
	TWI_I2C->EVENTS_TXDSENT  = 0;
	TWI_I2C->EVENTS_ERROR = 0;

	TWI_I2C->PSELSCL = pin_select.gpio_scl_pin; ///SCL SIGNAL
	TWI_I2C->PSELSDA = pin_select.gpio_sda_pin; ///SD SIGNAL

	twi_freq_global = clk_freq;
	if(clk_freq == K100)
		TWI_I2C->FREQUENCY = (TWI_FREQUENCY_FREQUENCY_K100<<TWI_FREQUENCY_FREQUENCY_Pos);
	else if(clk_freq == K250)
		TWI_I2C->FREQUENCY = (TWI_FREQUENCY_FREQUENCY_K250<<TWI_FREQUENCY_FREQUENCY_Pos);
	if(clk_freq == K400)
		TWI_I2C->FREQUENCY = (TWI_FREQUENCY_FREQUENCY_K400<<TWI_FREQUENCY_FREQUENCY_Pos);

	TWI_I2C->ENABLE = (TWI_ENABLE_ENABLE_Enabled<<TWI_ENABLE_ENABLE_Pos);

#ifdef USE_SOFTDEVICE
	sd_ppi_channel_assign(PPI_I2C_CHANNEL, &TWI_I2C->EVENTS_BB, &TWI_I2C->TASKS_SUSPEND);
	sd_ppi_channel_enable_clr(PPI_CHENCLR_CH0_Msk);
#else
	NRF_PPI->CH[PPI_I2C_CHANNEL].EEP = TWI_I2C->EVENTS_BB;
	NRF_PPI->CH[PPI_I2C_CHANNEL].TEP = TWI_I2C->TASKS_SUSPEND;
	NRF_PPI->CHENCLR |= PPI_CHENCLR_CH0_Enabled << PPI_CHENCLR_CH0_Msk;
#endif
}

/**
 * @fn TWI_Deinit()
 * @brief deinitialization I2C bus
 */
void TWI_Deinit ()
{
	TWI_I2C->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;

	NRF_GPIO->PIN_CNF[pin_select.gpio_scl_pin] = GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos;
//			  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
//	    	| (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
//	        | (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
//	        | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
//	        | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);
//
	NRF_GPIO->PIN_CNF[pin_select.gpio_sda_pin] = GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos;
//			  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
//	    	| (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
//	     	| (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
//	     	| (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
//	     	| (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);

//	NRF_GPIO->OUTSET = (1<<pin_select.gpio_sda_pin);
//	NRF_GPIO->OUTSET = (1<<pin_select.gpio_scl_pin);

    TWI_I2C->POWER = 0;
}

/**
 * @fn TWI_Clear_Bus()
 * @brief clear I2C bus
 * @return true if bus is clear
 */
bool TWI_Clear_Bus ()
{
    uint32_t twi_state;
    uint32_t clk_pin_config;
    uint32_t data_pin_config;
    bool bus_clear;

    twi_state = TWI_I2C->ENABLE;
    TWI_I2C->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    clk_pin_config = NRF_GPIO->PIN_CNF[pin_select.gpio_scl_pin];
    data_pin_config = NRF_GPIO->PIN_CNF[pin_select.gpio_sda_pin];

    NRF_GPIO->PIN_CNF[pin_select.gpio_scl_pin] =
    			  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
    	    	| (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
    	        | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
    	        | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
    	        | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);
    NRF_GPIO->PIN_CNF[pin_select.gpio_sda_pin] =
    			  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
    		    | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
    		    | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
    		    | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
    		    | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);

    NRF_GPIO->OUTSET = (1<<pin_select.gpio_sda_pin);
    NRF_GPIO->OUTSET = (1<<pin_select.gpio_scl_pin);
    ic_delay_ms(1);

    if ((((NRF_GPIO->IN >> pin_select.gpio_sda_pin) & 0x1UL) == 1) && (((pin_select.gpio_scl_pin) & 0x1UL) == 1)) {
    	bus_clear = true;
    }
    else {
    	bus_clear = TWI_Hard_Clear_Bus();
    }

    NRF_GPIO->PIN_CNF[pin_select.gpio_scl_pin] = clk_pin_config;
    NRF_GPIO->PIN_CNF[pin_select.gpio_sda_pin]  = data_pin_config;
    TWI_I2C->ENABLE = twi_state;

    return bus_clear;
}

bool TWI_Hard_Clear_Bus (){
	uint8_t i,j;
    for (i=18; i--;)
    {
        ResetBit(pin_select.gpio_scl_pin);
        for(j=25;j--;)
        	__NOP();
        SetBit(pin_select.gpio_scl_pin);
        for(j=25;j--;)
            __NOP();
        if ((((NRF_GPIO->IN >> pin_select.gpio_sda_pin) & 0x1UL) == 1) && (((pin_select.gpio_scl_pin) & 0x1UL) == 1))
        {
            return true;
        }
    }
    return false;
}

/**
 * @fn TWI_Reset_Bus(uint8_t events_error)
 * @brief reset I2C bus after error events
 */
void TWI_Reset_Bus(void)
{
	TWI_I2C->EVENTS_ERROR = 0;
    TWI_I2C->ENABLE       = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    TWI_I2C->POWER        = 0;
    ic_delay_ms(1);
    TWI_I2C->POWER        = 1;
    TWI_I2C->ENABLE       = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

    TWI_Init(twi_freq_global);
}

/**
 * @fn TWI_WriteReg(uint8_t slave_address, uint8_t address, uint8_t *data, uint8_t data_length)
 * @brief write value to device register
 * @param slave_address
 * @param register_address
 * @param data: pointer to the data to write
 * @param data_length (in byte)
 * @return o if data is write
 */
uint8_t TWI_WriteReg(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint32_t data_length)
{
	if (data_length == 0) {
		return 0;
	}
	//TODO Sprawdzic kiedy twi wylacza sie po TWI_Init()
	TWI_I2C->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;
	TWI_Clear_Bus();
	TWI_I2C->ADDRESS = slave_address;
	TWI_I2C->EVENTS_TXDSENT = 0;
	TWI_I2C->TXD = register_address;
	TWI_I2C->TASKS_STARTTX = 1;

	volatile uint32_t timeout = TIMEOUT_VAL;

	while(TWI_I2C->EVENTS_TXDSENT == 0 && TWI_I2C->EVENTS_ERROR == 0 && (--timeout)>0)
		__NOP();

	if (TWI_I2C->EVENTS_ERROR != 0 || timeout == 0) {
		TWI_Reset_Bus();
		return 1;
	}

	TWI_I2C->EVENTS_TXDSENT = 0;
	while (data_length-- != 0) {
		TWI_I2C->TXD = *data++;
		timeout = TIMEOUT_VAL;
		while(TWI_I2C->EVENTS_TXDSENT == 0 && TWI_I2C->EVENTS_ERROR == 0 && (--timeout)>0)
			__NOP();

		if (TWI_I2C->EVENTS_ERROR != 0 || timeout == 0) {
			TWI_Reset_Bus();
			return 1;
		}

		TWI_I2C->EVENTS_TXDSENT = 0;
	}

	TWI_I2C->EVENTS_STOPPED = 0;
	TWI_I2C->TASKS_STOP=1;
	timeout = TIMEOUT_VAL;
	while (TWI_I2C->EVENTS_STOPPED == 0 && (--timeout)>0 ){
			__NOP();
	}

	if(timeout==0) {
		TWI_Reset_Bus();
		return 1;
	}

	TWI_I2C->EVENTS_STOPPED = 0;

	return 0;
}

/**
 * @fn TWI_ReadReg(uint8_t slave_address, uint8_t address, uint8_t *data, uint8_t data_length)
 * @brief read value from device register
 * @param slave_address
 * @param register_address
 * @param data: pointer to the variable when data are read
 * @param data_length (in byte)
 * @return o if data is read
 */
uint8_t TWI_ReadReg(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint32_t data_length)
{
	//TODO Sprawdzic kiedy twi wylacza sie po TWI_Init()
	TWI_I2C->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;
	TWI_Clear_Bus();
	TWI_I2C->ADDRESS = slave_address;
	TWI_I2C->EVENTS_TXDSENT = 0;
	TWI_I2C->TXD = register_address;
	TWI_I2C->TASKS_STARTTX = 1;

	uint32_t timeout = TIMEOUT_VAL;

	while(TWI_I2C->EVENTS_TXDSENT == 0 && TWI_I2C->EVENTS_ERROR == 0 && (--timeout)>0)
		__NOP();

	if (TWI_I2C->EVENTS_ERROR != 0 || timeout == 0) {
		TWI_Reset_Bus();
		return 1;
	}

	TWI_I2C->EVENTS_TXDSENT = 0;

	if (data_length == 0) {
		return 1;
	}

	else if (data_length == 1) {
#ifdef USE_SOFTDEVICE
		sd_ppi_channel_assign(PPI_I2C_CHANNEL, &TWI_I2C->EVENTS_BB, &TWI_I2C->TASKS_STOP);
#else
		NRF_PPI->CH[PPI_I2C_CHANNEL].EEP = TWI_I2C->EVENTS_BB;
		NRF_PPI->CH[PPI_I2C_CHANNEL].TEP = TWI_I2C->TASKS_STOP;
#endif

	}
	else {
#ifdef USE_SOFTDEVICE
		sd_ppi_channel_assign(PPI_I2C_CHANNEL, &TWI_I2C->EVENTS_BB, &TWI_I2C->TASKS_SUSPEND);
#else
		NRF_PPI->CH[PPI_I2C_CHANNEL].EEP = TWI_I2C->EVENTS_BB;
		NRF_PPI->CH[PPI_I2C_CHANNEL].TEP = TWI_I2C->TASKS_SUSPEND;
#endif
	}
#ifdef USE_SOFTDEVICE
	sd_ppi_channel_enable_set(PPI_CHENSET_CH0_Msk);
#else
		NRF_PPI->CHENSET |= PPI_CHENSET_CH0_Enabled << PPI_CHENSET_CH0_Msk;

#endif

	TWI_I2C->EVENTS_RXDREADY = 0;
	TWI_I2C->TASKS_STARTRX = 1;

	while(data_length-- != 0) {

		timeout = TIMEOUT_VAL;

		while(TWI_I2C->EVENTS_RXDREADY == 0 && TWI_I2C->EVENTS_ERROR == 0 && (--timeout)>0)
			__NOP();

		if (TWI_I2C->EVENTS_ERROR != 0 || timeout==0) {
			TWI_Reset_Bus();
			return 1;
		}

		TWI_I2C->EVENTS_RXDREADY = 0;
		*data++ = TWI_I2C->RXD;

		if (data_length == 1) {


#ifdef USE_SOFTDEVICE
			sd_ppi_channel_assign(PPI_I2C_CHANNEL, &TWI_I2C->EVENTS_BB, &TWI_I2C->TASKS_STOP);
#else
			NRF_PPI->CH[PPI_I2C_CHANNEL].EEP = TWI_I2C->EVENTS_BB;
			NRF_PPI->CH[PPI_I2C_CHANNEL].TEP = TWI_I2C->TASKS_STOP;
#endif
		}

		if (data_length == 0) {
			break;
		}

		TWI_I2C->TASKS_RESUME = 1;
	}

	timeout = TIMEOUT_VAL;

	while (TWI_I2C->EVENTS_STOPPED == 0 && (--timeout)>0){
		__NOP();
	}

	if(timeout==0) {
		TWI_Reset_Bus();
		return 1;
	}

	TWI_I2C->EVENTS_STOPPED = 0;

#ifdef USE_SOFTDEVICE
		sd_ppi_channel_enable_clr(PPI_CHENCLR_CH0_Msk);
#else
		NRF_PPI->CHENCLR |= PPI_CHENCLR_CH0_Enabled << PPI_CHENCLR_CH0_Msk;

#endif

	return 0;
}
#endif
