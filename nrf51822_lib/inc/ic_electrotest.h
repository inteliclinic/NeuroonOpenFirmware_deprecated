#ifndef IC_ELECTROTEST_H_
#define IC_ELECTROTEST_H_

/**
 * @file
 * @brief electrode contact test header
 */
#include "ic_lib_config.h"

typedef struct differance_min_max_of_signal_s
{
	uint16_t electrode1;
	uint16_t electrode2;
} differance_min_max_of_signal_t, *differance_min_max_of_signal_p;


differance_min_max_of_signal_t *diff_p, diff;

/** APPLICATION FUNCTIONS */
void electrotest_init(void);
void electrotest_deinit(void);
void electrotest_start(void);
void electrotest_stop(void);

uint64_t electrotest_get_mean_power_e1(void);
uint64_t electrotest_get_mean_power_e2(void);

void electrotest_adc_rdy_handler(uint32_t values_left);

void set_differance_min_max_of_signal(uint16_t diff);
void set_next_electrode_on_test();
#endif /* IC_ELECTROTEST_H_ */
