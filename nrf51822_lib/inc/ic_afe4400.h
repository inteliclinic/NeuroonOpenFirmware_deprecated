/**
 * @file    ic_afe4400.h
 * @Author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   AFE4400 driver header
 *
 * Description
 */

#ifndef __IC_AFE4400__H
#define __IC_AFE4400__H

#include "ic_lib_config.h"
#include "ic_frame_handle.h"

/** @defgroup MOD_TI_AFE44x0_LIB modyfication (standard Neuroon values) of macros from TI's AFE44x0.h
 *
 * @{
 */
// CONTROL0 - Write Only register
#define CONTROL0        0x00
#define    CONTROL0_VAL     (0x000000ul)
#define    CONTROL0_STD_VAL (DIAG_EN)
#define    SPI_WRITE        (0x000000ul)
#define    SPI_READ         (0x000001ul)        //SPI read
#define    TIM_CNT_RST      (0x000002ul)        //Timer counter reset
#define    DIAG_EN          (0x000004ul)        //Diagnostic enable
#define    SW_RST           (0x000008ul)        //Software reset

// CONTROL1 - Read/Write register
#define CONTROL1        0x1E
#define    CONTROL1_VAL                         (0x000002ul)
#define    CONTROL1_STD_VAL                     (TIMEREN)
#define    SAMPLE_LED2_SAMPLE_LED1              (0x000000ul)        //Clocks on ALM pins
#define    LED2_PULSE_LED1_PULSE                (0x000200ul)        //Clocks on ALM pins
#define    SAMPLE_LED2_SAMPLE_LED1_PULSE        (0x000400ul)        //Clocks on ALM pins
#define    LED2_CONVERT_LED1_CONVERT            (0x000600ul)        //Clocks on ALM pins
#define    LED2_AMBIENT_LED1_AMBIENT            (0x000800ul)        //Clocks on ALM pins
#define    NO_OUTPUT_NO_OUTPUT                  (0x000A00ul)        //Clocks on ALM pins
#define    TIMEREN                              (0x000100ul)//Timer enable

#define TIAGAIN         0x20 /*Register for factory use*/
#define    TIAGAIN_VAL                 (0x000000ul)
#define    RF_LED1_500K                (0x000000ul)        //Program RF for LED1
#define    RF_LED1_250K                (0x000001ul)        //Program RF for LED1
#define    RF_LED1_100K                (0x000002ul)        //Program RF for LED1
#define    RF_LED1_50K                 (0x000003ul)        //Program RF for LED1
#define    RF_LED1_25K                 (0x000004ul)        //Program RF for LED1
#define    RF_LED1_10K                 (0x000005ul)        //Program RF for LED1
#define    RF_LED1_1M                  (0x000006ul)        //Program RF for LED1
#define    RF_LED1_NONE                (0x000007ul)        //Program RF for LED1

#define    CF_LED1_5P                  (0x000000ul)        //Program CF for LED1
#define    CF_LED1_5P_5P               (0x000008ul)        //Program CF for LED1
#define    CF_LED1_15P_5P              (0x000010ul)        //Program CF for LED1
#define    CF_LED1_20P_5P              (0x000018ul)        //Program CF for LED1
#define    CF_LED1_25P_5P              (0x000020ul)        //Program CF for LED1
#define    CF_LED1_30P_5P              (0x000028ul)        //Program CF for LED1
#define    CF_LED1_40P_5P              (0x000030ul)        //Program CF for LED1
#define    CF_LED1_45P_5P              (0x000038ul)        //Program CF for LED1
#define    CF_LED1_50P_5P              (0x000040ul)        //Program CF for LED1
#define    CF_LED1_55P_5P              (0x000048ul)        //Program CF for LED1
#define    CF_LED1_65P_5P              (0x000050ul)        //Program CF for LED1
#define    CF_LED1_70P_5P              (0x000058ul)        //Program CF for LED1
#define    CF_LED1_75P_5P              (0x000060ul)        //Program CF for LED1
#define    CF_LED1_80P_5P              (0x000068ul)        //Program CF for LED1
#define    CF_LED1_90P_5P              (0x000070ul)        //Program CF for LED1
#define    CF_LED1_95P_5P              (0x000078ul)        //Program CF for LED1
#define    CF_LED1_150P_5P             (0x000080ul)        //Program CF for LED1
#define    CF_LED1_155P_5P             (0x000088ul)        //Program CF for LED1
#define    CF_LED1_165P_5P             (0x000090ul)        //Program CF for LED1
#define    CF_LED1_170P_5P             (0x000098ul)        //Program CF for LED1
#define    CF_LED1_175P_5P             (0x0000A0ul)        //Program CF for LED1
#define    CF_LED1_180P_5P             (0x0000A8ul)        //Program CF for LED1
#define    CF_LED1_190P_5P             (0x0000B0ul)        //Program CF for LED1
#define    CF_LED1_195P_5P             (0x0000B8ul)        //Program CF for LED1
#define    CF_LED1_200P_5P             (0x0000C0ul)        //Program CF for LED1
#define    CF_LED1_205P_5P             (0x0000C8ul)        //Program CF for LED1
#define    CF_LED1_215P_5P             (0x0000D0ul)        //Program CF for LED1
#define    CF_LED1_220P_5P             (0x0000D8ul)        //Program CF for LED1
#define    CF_LED1_225P_5P             (0x0000E0ul)        //Program CF for LED1
#define    CF_LED1_230P_5P             (0x0000E8ul)        //Program CF for LED1
#define    CF_LED1_240P_5P             (0x0000F0ul)        //Program CF for LED1
#define    CF_LED1_245P_5P             (0x0000F8ul)        //Program CF for LED1

#define    STG2GAIN_LED1_0DB           0x000000ul        //Stage 2 gain setting for LED1
#define    STG2GAIN_LED1_3DB           0x000100ul        //Stage 2 gain setting for LED1
#define    STG2GAIN_LED1_6DB           0x000200ul        //Stage 2 gain setting for LED1
#define    STG2GAIN_LED1_9DB           0x000300ul        //Stage 2 gain setting for LED1
#define    STG2GAIN_LED1_12DB          0x000400ul        //Stage 2 gain setting for LED1
#define    STG2GAIN_LED1               0x000700ul        //Stage 2 gain setting for LED1

#define    STAGE2EN_LED1               0x004000ul        //Stage 2 enable for LED1
#define    ENSEPGAIN                   0x008000ul


#define TIA_AMB_GAIN    0x21
#define    TIA_AMB_GAIN_VAL            0x000000ul
#define    TIA_AMB_GAIN_STD_VAL        (STAGE2EN_LED2 +\
                                        STG2GAIN_LED2_12DB +\
                                        CF_LED2_15P_5P +\
                                        RF_LED2_10K)
#define    RF_LED2_500K                0x000000ul        //Program RF for LED2
#define    RF_LED2_250K                0x000001ul        //Program RF for LED2
#define    RF_LED2_100K                0x000002ul        //Program RF for LED2
#define    RF_LED2_50K                 0x000003ul        //Program RF for LED2
#define    RF_LED2_25K                 0x000004ul        //Program RF for LED2
#define    RF_LED2_10K                 0x000005ul        //Program RF for LED2
#define    RF_LED2_1M                  0x000006ul        //Program RF for LED2
#define    RF_LED2_NONE                0x000007ul        //Program RF for LED2

#define    CF_LED2_5P                  0x000000ul        //Program CF for LED2
#define    CF_LED2_5P_5P               0x000008ul        //Program CF for LED2
#define    CF_LED2_15P_5P              0x000010ul        //Program CF for LED2
#define    CF_LED2_20P_5P              0x000018ul        //Program CF for LED2
#define    CF_LED2_25P_5P              0x000020ul        //Program CF for LED2
#define    CF_LED2_30P_5P              0x000028ul        //Program CF for LED2
#define    CF_LED2_40P_5P              0x000030ul        //Program CF for LED2
#define    CF_LED2_45P_5P              0x000038ul        //Program CF for LED2
#define    CF_LED2_50P_5P              0x000040ul        //Program CF for LED2
#define    CF_LED2_55P_5P              0x000048ul        //Program CF for LED2
#define    CF_LED2_65P_5P              0x000050ul        //Program CF for LED2
#define    CF_LED2_70P_5P              0x000058ul        //Program CF for LED2
#define    CF_LED2_75P_5P              0x000060ul        //Program CF for LED2
#define    CF_LED2_80P_5P              0x000068ul        //Program CF for LED2
#define    CF_LED2_90P_5P              0x000070ul        //Program CF for LED2
#define    CF_LED2_95P_5P              0x000078ul        //Program CF for LED2
#define    CF_LED2_150P_5P             0x000080ul        //Program CF for LED2
#define    CF_LED2_155P_5P             0x000088ul        //Program CF for LED2
#define    CF_LED2_165P_5P             0x000090ul        //Program CF for LED2
#define    CF_LED2_170P_5P             0x000098ul        //Program CF for LED2
#define    CF_LED2_175P_5P             0x0000A0ul        //Program CF for LED2
#define    CF_LED2_180P_5P             0x0000A8ul        //Program CF for LED2
#define    CF_LED2_190P_5P             0x0000B0ul        //Program CF for LED2
#define    CF_LED2_195P_5P             0x0000B8ul        //Program CF for LED2
#define    CF_LED2_200P_5P             0x0000C0ul        //Program CF for LED2
#define    CF_LED2_205P_5P             0x0000C8ul        //Program CF for LED2
#define    CF_LED2_215P_5P             0x0000D0ul        //Program CF for LED2
#define    CF_LED2_220P_5P             0x0000D8ul        //Program CF for LED2
#define    CF_LED2_225P_5P             0x0000E0ul        //Program CF for LED2
#define    CF_LED2_230P_5P             0x0000E8ul        //Program CF for LED2
#define    CF_LED2_240P_5P             0x0000F0ul        //Program CF for LED2
#define    CF_LED2_245P_5P             0x0000F8ul        //Program CF for LED2

#define    STG2GAIN_LED2_0DB           0x000000ul        //Stage 2 gain setting for LED2
#define    STG2GAIN_LED2_3DB           0x000100ul        //Stage 2 gain setting for LED2
#define    STG2GAIN_LED2_6DB           0x000200ul        //Stage 2 gain setting for LED2
#define    STG2GAIN_LED2_9DB           0x000300ul        //Stage 2 gain setting for LED2
#define    STG2GAIN_LED2_12DB          0x000400ul        //Stage 2 gain setting for LED2
#define    STG2GAIN_LED2               0x000700ul        //Stage 2 gain setting for LED2

#define    STAGE2EN_LED2               0x004000ul        //Stage 2 enable for LED2

#define    AMBDAC_0uA                  0x000000ul        //Ambient DAC value
#define    AMBDAC_1uA                  0x010000ul        //Ambient DAC value
#define    AMBDAC_2uA                  0x020000ul        //Ambient DAC value
#define    AMBDAC_3uA                  0x030000ul        //Ambient DAC value
#define    AMBDAC_4uA                  0x040000ul        //Ambient DAC value
#define    AMBDAC_5uA                  0x050000ul        //Ambient DAC value
#define    AMBDAC_6uA                  0x060000ul        //Ambient DAC value
#define    AMBDAC_7uA                  0x070000ul        //Ambient DAC value
#define    AMBDAC_8uA                  0x080000ul        //Ambient DAC value
#define    AMBDAC_9uA                  0x090000ul        //Ambient DAC value
#define    AMBDAC_10uA                 0x0A0000ul        //Ambient DAC value

#define LEDCNTRL        0x22
#define    LEDCNTRL_VAL                (0x010000ul)
#define    LEDCNTRL_STD_VAL            (LED2_STD_CURRENT +\
                                        LED1_STD_CURRENT)
#define    LED2_CURRENT                (0x0000FFul)        //Program LED current for LED2 signal
#define    LED1_CURRENT                (0x00FF00ul)        //Program LED current for LED1 signal
#define    LED2_STD_CURRENT            (0x00003Aul)        //Program standard Neuroon LED current for LED2 signal
#define    LED1_STD_CURRENT            (0x000300ul)        //Program standard Neuroon LED current for LED1 signal

#define CONTROL2        0x23
#define    CONTROL2_VAL                (0x020100ul)
#define    CONTROL2_STD_VAL            (PDN_AFE_OFF)
#define    PDN_AFE_OFF                 (0x000000ul)        //AFE power-down (Powered on)
#define    PDN_AFE_ON                  (0x000001ul)        //AFE power-down (Powered off)

#define    PDN_RX_OFF                  (0x000000ul)        //Rx power-down (Powered on)
#define    PDN_RX_ON                   (0x000002ul)        //Rx power-down (Powered off)

#define    PDN_TX_OFF                  (0x000000ul)        //Tx power-down (Powered on)
#define    PDN_TX_ON                   (0x000004ul)        //Tx power-down (Powered off)

#define    EN_FAST_DIAG                (0x000000ul)        //Fast diagnostics mode enable
#define    EN_SLOW_DIAG                (0x000100ul)        //Slow diagnostics mode enable

#define    XTAL_ENABLE                 (0x000000ul)        //The crystal module is enabled
#define    XTAL_DISABLE                (0x000200ul)        //The crystal module is disabled

#define    DIGOUT_TRISTATE_DISABLE     (0x000000ul)        //Digital tristate disabled
#define    DIGOUT_TRISTATE_ENABLE      (0x000400ul)        //Digital tristate enabled

#define    TXBRGMOD_H_BRIDGE           (0x000000ul)        //Tx bridge mode
#define    TXBRGMOD_PUSH_PULL          (0x000800ul)        //Tx bridge mode

#define ALARM           0x29
#define    ALARM_VAL                    (0x000000ul)
#define    ALARM_STD_VAL                (ALMPINCLKEN)
#define    ALMPINCLKEN                  (0x000080ul)        //Alarm pin clock enable (Enables CLKALMPIN)

// Read only registers
#define LED2VAL         0x2A
#define ALED2VAL        0x2B
#define LED1VAL         0x2C
#define ALED1VAL        0x2D
#define LED2_ALED2VAL   0x2E
#define LED1_ALED1VAL   0x2F
#define DIAG            0x30
// End of Read only registers

#define PRPCOUNT              0x1D
#define PRPCOUNT_STD_VAL      7999

#define LED2STC               0x01
#define LED2STC_STD_VAL       6050

#define LED2ENDC              0x02
#define LED2ENDC_STD_VAL      7998

#define LED2LEDSTC            0x03
#define LED2LEDSTC_STD_VAL    6000

#define LED2LEDENDC           0x04
#define LED2LEDENDC_STD_VAL   7999

#define ALED2STC              0x05
#define ALED2STC_STD_VAL      50

#define ALED2ENDC             0x06
#define ALED2ENDC_STD_VAL     1998

#define LED1STC               0x07
#define LED1STC_STD_VAL       2050

#define LED1ENDC              0x08
#define LED1ENDC_STD_VAL      3998

#define LED1LEDSTC            0x09
#define LED1LEDSTC_STD_VAL    2000

#define LED1LEDENDC           0x0A
#define LED1LEDENDC_STD_VAL   3999

#define ALED1STC              0x0B
#define ALED1STC_STD_VAL      4050

#define ALED1ENDC             0x0C
#define ALED1ENDC_STD_VAL     5998

#define LED2CONVST            0x0D
#define LED2CONVST_STD_VAL    4

#define LED2CONVEND           0x0E
#define LED2CONVEND_STD_VAL   1999

#define ALED2CONVST           0x0F
#define ALED2CONVST_STD_VAL   2004

#define ALED2CONVEND          0x10
#define ALED2CONVEND_STD_VAL  3999

#define LED1CONVST            0x11
#define LED1CONVST_STD_VAL    4004

#define LED1CONVEND           0x12
#define LED1CONVEND_STD_VAL   5999

#define ALED1CONVST           0x13
#define ALED1CONVST_STD_VAL   6004

#define ALED1CONVEND          0x14
#define ALED1CONVEND_STD_VAL  7999

#define ADCRSTSTCT0           0x15
#define ADCRSTSTCT0_STD_VAL   0

#define ADCRSTENDCT0          0x16
#define ADCRSTENDCT0_STD_VAL  3

#define ADCRSTSTCT1           0x17
#define ADCRSTSTCT1_STD_VAL   2000

#define ADCRSTENDCT1          0x18
#define ADCRSTENDCT1_STD_VAL  2003

#define ADCRSTSTCT2           0x19
#define ADCRSTSTCT2_STD_VAL   4000

#define ADCRSTENDCT2          0x1A
#define ADCRSTENDCT2_STD_VAL  4003

#define ADCRSTSTCT3           0x1B
#define ADCRSTSTCT3_STD_VAL   6000

#define ADCRSTENDCT3          0x1C
#define ADCRSTENDCT3_STD_VAL  6003

/** @} */ // end of MOD_TI_AFE44x0_LIB

/**
 * @struct afe4400_config
 * @brief configuration structure
 */
typedef struct {
	uint32_t led1_current;
	uint32_t led2_current;
	uint32_t gain;
	uint32_t rf_led;
	uint32_t cf_led;
} afe4400_config_t, *afe4400_config_p;

/** DEVICE DEPENDENT FUNCTIONS */

/** READ/WRITE REGISTER */ /*TODO: read/write functions should be private for ic_afe4400 module (no
                             time to do this)*/
uint32_t afe4400_ReadReg(uint8_t reg);
void afe4400_WriteReg(uint8_t reg, uint32_t data);

/** CONTROL */
void afe4400_hdw_init(void);
void afe4400_std_val_init(void);
void afe4400_powerdown_on(void);
void afe4400_powerdown_off(void);

void afe4400_init(void);
void afe4400_start(void);
void afe4400_stop(void);

/** @defgroup AFE4400_REG_CONF_FUNC global variables with current AFE4400 registers configuration
 *
 * @{
 */
void afe4400_set_control0_r(t_afe4400RegisterConf reg_val);
void afe4400_set_control1_r(t_afe4400RegisterConf reg_val);
void afe4400_set_tia_amb_gain_r(t_afe4400RegisterConf reg_val);

/**
 * @brief
 * LED current = ((LEDx)/256)*50mA
 *
 * @param reg_val
 */
void afe4400_set_ledcntrl_r(t_afe4400RegisterConf reg_val);
void afe4400_set_control2_r(t_afe4400RegisterConf reg_val);
void afe4400_set_alarm_r(t_afe4400RegisterConf reg_val);
void afe4400_set_selected_timer_module_r(t_afe4400Register reg, t_afe4400RegisterConf reg_val);

t_afe4400RegisterConf afe4400_get_selected_r(t_afe4400Register reg);

void afe4400_read_selected_r(t_afe4400Register reg);
/** @} */ //end of AFE4400_REGS_CONF

/** RED LED CONTROL */
void afe4400_red_off ();
void afe4400_red_on ();

/** READ DATA */
void afe4400_TakeData(int32_t *data);
void afe4400_TakeData_LED1(int32_t *data);
void afe4400_TakeData_LED2(int32_t *data);

/** CONFIG GETTERS */
afe4400_config_p afe4400_get_config(void);

/** SELFTEST */
int afe4400_SelfTest();

#endif
