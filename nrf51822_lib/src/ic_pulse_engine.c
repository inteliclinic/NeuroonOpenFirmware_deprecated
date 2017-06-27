#include "ic_pulse_engine.h"
#include <ic_cbuffer.h>
#include <ic_log.h>
///defines------------------------------------------------------------------
#define NO_OF_SAMPLES_TO_START 7
#define ONE_MINUTE 60000 //in ms
//--------------------------------------------------------------------------

///variables----------------------------------------------------------------
///counts the first seven samples, which will not be used for differentiate
static uint8_t diff_sample_cnt = 0;
///No. of samples for last complete event
static uint32_t no_of_samples_for_last_event = 0;
///counts samples after first event
static uint32_t after_first_event_sample_cnt = 0;
///number of pulses for averaging
static uint8_t pulse_event_cnt = 0;

///differentiate register is full - YES, WE CAN (differentiate)!
volatile uint8_t start_diff_flag;

///differentiate register
volatile PE_SAMPLE_TYPE samples_for_diff[7];
///previous value of 7-point differentiation
static PE_SAMPLE_TYPE prev_diff_val;
///current value of 7-point differentiation
static volatile PE_SAMPLE_TYPE diff_val = 0;
//--------------------------------------------------------------------------
/* FIR(100, [0.50, 3.50]/12.50) den=32000 */
//static const int32_t ledIRfir[101] = {0,-10,-10,3,23,39,40,25,4,-1,21,66,107,113,76,19,-13,17,101,183,193,102,-44,-150,-134,0,149,172,-2,-306,-558,-585,-365,-68,35,-228,-777,-1288,-1401,-988,-306,112,-198,-1259,-2510,-3035,-2081,439,3765,6588,7694,6588,3765,439,-2081,-3035,-2510,-1259,-198,112,-306,-988,-1401,-1288,-777,-228,35,-68,-365,-585,-558,-306,-2,172,149,0,-134,-150,-44,102,193,183,101,17,-13,19,76,113,107,66,21,-1,4,25,40,39,23,3,-10,-10,0};

/* FIR(150, [0.83, 1.83]/12.50) den=32000 */
//static const int32_t ledIRfir[151] = {0,2,4,3,0,-5,-12,-19,-26,-31,-33,-29,-21,-8,9,26,41,52,56,52,42,27,12,0,-5,0,14,35,57,73,76,60,22,-35,-106,-180,-243,-283,-290,-258,-190,-95,13,114,193,234,232,190,123,52,0,-10,38,145,300,475,631,726,720,585,312,-84,-562,-1061,-1508,-1828,-1955,-1850,-1502,-935,-211,585,1352,1988,2409,2556,2409,1988,1352,585,-211,-935,-1502,-1850,-1955,-1828,-1508,-1061,-562,-84,312,585,720,726,631,475,300,145,38,-10,0,52,123,190,232,234,193,114,13,-95,-190,-258,-290,-283,-243,-180,-106,-35,22,60,76,73,57,35,14,0,-5,0,12,27,42,52,56,52,41,26,9,-8,-21,-29,-33,-31,-26,-19,-12,-5,0,3,4,2,0};

/* FIR(100, [0.83, 1.83]/12.50) den=32000 */
static const int32_t ledIRfir[101] = {0,4,9,14,18,20,16,6,-11,-34,-61,-88,-109,-118,-112,-87,-46,6,61,107,136,140,120,81,35,0,-7,28,111,235,381,518,609,616,510,276,-76,-514,-983,-1414,-1733,-1872,-1787,-1461,-917,-208,579,1343,1980,2403,2550,2403,1980,1343,579,-208,-917,-1461,-1787,-1872,-1733,-1414,-983,-514,-76,276,510,616,609,518,381,235,111,28,-7,0,35,81,120,140,136,107,61,6,-46,-87,-112,-118,-109,-88,-61,-34,-11,6,16,20,18,14,9,4,0};

static int32_t __buf_led_id[101];
static CBi32 cbLedIR = CBUFFER_INIT(__buf_led_id, 101);
//--------------------------------------------------------------------------

///local functions----------------------------------------------------------
/**
 * @brief Add sample to differentiate register. When there will be at least 7,
 *		  we can start differentiation.
 */
static void add_sample_to_diff_reg(PE_SAMPLE_TYPE sample_to_add) {
	///auxiliary counter
	uint8_t i;
	if (diff_sample_cnt < NO_OF_SAMPLES_TO_START) {
		start_diff_flag = 0;
		samples_for_diff[diff_sample_cnt] = sample_to_add;
		diff_sample_cnt++;
	} else {
		start_diff_flag = 1;
		for (i = 0; i < 6; i++) {
			samples_for_diff[i] = samples_for_diff[i+1];
		}
		samples_for_diff[6] = sample_to_add;
	}
}

/**
 * @brief 7 point differentiation without division.
 */
static void not_completely_differentiation() {
	if(diff_sample_cnt <= 7) {
		diff_val = (-samples_for_diff[0]) + (9*samples_for_diff[1]) - (45*samples_for_diff[2]) + (45*samples_for_diff[4]) - (9*samples_for_diff[5]) + (samples_for_diff[6]);
		prev_diff_val = diff_val;
		diff_sample_cnt = 8;
	} else {
		prev_diff_val = diff_val;
		diff_val = (-samples_for_diff[0]) + (9*samples_for_diff[1]) - (45*samples_for_diff[2]) + (45*samples_for_diff[4]) - (9*samples_for_diff[5]) + (samples_for_diff[6]);
	}
}

//--------------------------------------------------------------------------

void PE_ClearAndInit() {
	diff_sample_cnt = 0;
	pulse_event_cnt = 0;
	start_diff_flag = 0;
	prev_diff_val = 0;
	diff_val = 0;
	no_of_samples_for_last_event = 0;
	after_first_event_sample_cnt = 0;
}

void PE_AddSample(PE_SAMPLE_TYPE sample) {

//	if (cbLedIR.count == 97) {
//		volatile int a = 0;
//		a++;
//	}
	add_sample_to_diff_reg(sample);
	cbi32_add(&cbLedIR, sample);
	cbi32_weightedSumFromNewest(&cbLedIR, ledIRfir);
	//sam  = cbi32_getFastSum(&cbLedIR)/cbLedIR.len;

	//log_redled_byfifo(sample);
	//log_irled_byfifo(sam);

	if (start_diff_flag) {
		not_completely_differentiation();
		if((prev_diff_val >= 0) && (diff_val < 0)) {
			pulse_event_cnt++;
			no_of_samples_for_last_event = after_first_event_sample_cnt;
		}
		if (pulse_event_cnt >= 1) {
			after_first_event_sample_cnt++;
		}
	}
}

uint16_t PE_GetNumberOfCapturedPulseIncident() {
	if (pulse_event_cnt > 1) {
		return pulse_event_cnt;
	}
	return 0;
}

int16_t PE_GetAveragePulse() {
	if (pulse_event_cnt > 1) {
		return ONE_MINUTE/(no_of_samples_for_last_event*PE_SAMPLE_PERIOD/(pulse_event_cnt-1)); //pulse_event_cnt must be decreased by 1, because no_of_samples_for_last_event is counted after first event
	}
	return -1;
}

#ifdef PULSE_ENG_MODULE_TEST
int main(){
//	PE_SAMPLE_TYPE sinus[] = {0,1018,2016,2974,3876,4703,5440,6075,6596,6996,7270,7416,7434,7330,7110,6783,6362,5861,5296,4684,4045,3397,2759,2149,1585,1084,659,324,88,-42,-62,31,235,544,952,1449,2022,2656,3334,4041,4755,5459,6134,6760,7320,7799,8180,8453,8608,8637,8536,8304,7942,7457,6856,6148,5348,4470,3531,2550,1545,537,-454,-1410,-2312,-3143,-3888,-4534,-5069,-5487,-5782,-5952,-5999,-5925,-5739,-5449,-5068,-4610,-4090,-3527,-2939,-2344,-1761,-1210,-706,-266,94,364,534,595,545,381,106,-275,-756,-1326,-1973,-2680,-3433,-4212,-5000,-5777,-6523,-7220,-7850,-8397,-8847,-9185,-9404,-9495,-9455,-9282,-8977,-8546,-7996,-7338,-6584,-5750,-4852,-3908,-2939,-1963,-1000,-70,810,1622,2351,2985,3512,3925,4218,4390,4441,4376,4202,3928,3566,3130,2636,2102,1545,985,441,-70,-530,-923,-1235,-1453,-1568,-1573,-1464,-1240,-903,-457,90,728,1443,2220,3044,3895,4755,5605,6425,7195,7899,8520,9043,9454,9745,9908,9938,9835,9598,9234,8749,8155,7463,6689,5849,4961,4045,3120,2205,1321,484,-288,-981,-1580,-2076,-2461,-2730,-2881,-2915,-2835,-2650,-2368,-2002,-1565,-1075,-547,-0,547,1075,1565,2002,2368,2650,2835,2915,2881,2730,2461,2076,1580,981,288,-484,-1321,-2205,-3120,-4045,-4961,-5849,-6689,-7463,-8155,-8749,-9234,-9598,-9835,-9938,-9908,-9745,-9454,-9043,-8520,-7899,-7195,-6425,-5605,-4755,-3895,-3044,-2220,-1443,-728,-90,457,903,1240,1464,1573,1568,1453,1235,923,530,70,-441,-985,-1545,-2102,-2636,-3130,-3566,-3928,-4202,-4376,-4441,-4390,-4218,-3925,-3512,-2985,-2351,-1622,-810,70,1000,1963,2939,3908,4852,5750,6584,7338,7996,8546,8977,9282,9455,9495,9404,9185,8847,8397,7850,7220,6523,5777,5000,4212,3433,2680,1973,1326,756,275,-106,-381,-545,-595,-534,-364,-94,266,706,1210,1761,2344,2939,3527,4090,4610,5068,5449,5739,5925,5999,5952,5782,5487,5069,4534,3888,3143,2312,1410,454,-537,-1545,-2550,-3531,-4470,-5348,-6148,-6856,-7457,-7942,-8304,-8536,-8637,-8608,-8453,-8180,-7799,-7320,-6760,-6134,-5459,-4755,-4041,-3334,-2656,-2022,-1449,-952,-544,-235,-31,62,42,-88,-324,-659,-1084,-1585,-2149,-2759,-3397,-4045,-4684,-5296,-5861,-6362,-6783,-7110,-7330,-7434,-7416,-7270,-6996,-6596,-6075,-5440,-4703,-3876,-2974,-2016,-1018,-0}; //for 25 ms period
	PE_SAMPLE_TYPE sinus[] = {0,815,1620,2405,3160,3876,4544,5157,5707,6188,6596,6926,7176,7344,7430,7434,7360,7211,6991,6706,6362,5966,5528,5056,4558,4045,3526,3012,2510,2032,1585,1178,819,514,268,88,-25,-67,-38,63,235,474,778,1141,1558,2022,2525,3059,3615,4183,4755,5320,5869,6391,6878,7320,7710,8040,8303,8494,8608,8641,8592,8458,8241,7942,7564,7110,6585,5995,5348,4651,3913,3142,2350,1545,738,-61,-842,-1596,-2312,-2983,-3601,-4159,-4650,-5069,-5413,-5679,-5865,-5971,-5999,-5949,-5826,-5635,-5380,-5068,-4707,-4304,-3869,-3411,-2939,-2463,-1992,-1536,-1104,-706,-349,-40,214,406,534,592,579,493,335,106,-191,-553,-974,-1450,-1973,-2534,-3127,-3742,-4370,-5000,-5623,-6229,-6809,-7352,-7850,-8295,-8679,-8996,-9239,-9404,-9488,-9487,-9402,-9231,-8977,-8642,-8229,-7745,-7194,-6584,-5922,-5217,-4479,-3716,-2939,-2158,-1382,-623,111,810,1466,2071,2617,3099,3512,3851,4115,4301,4410,4441,4398,4284,4104,3862,3566,3222,2839,2426,1991,1545,1097,655,231,-167,-530,-851,-1121,-1334,-1484,-1568,-1581,-1522,-1389,-1182,-903,-554,-140,335,865,1443,2060,2710,3382,4067,4755,5437,6102,6740,7342,7899,8403,8846,9221,9523,9745,9886,9942,9913,9798,9598,9317,8957,8524,8024,7463,6849,6192,5498,4780,4045,3305,2569,1847,1149,484,-140,-714,-1232,-1688,-2076,-2393,-2637,-2805,-2897,-2915,-2860,-2736,-2548,-2301,-2002,-1658,-1276,-867,-438,0,438,867,1276,1658,2002,2301,2548,2736,2860,2915,2897,2805,2637,2393,2076,1688,1232,714,140,-484,-1149,-1847,-2569,-3305,-4045,-4780,-5498,-6192,-6849,-7463,-8024,-8524,-8957,-9317,-9598,-9798,-9913,-9942,-9886,-9745,-9523,-9221,-8846,-8403,-7899,-7342,-6740,-6102,-5437,-4755,-4067,-3382,-2710,-2060,-1443,-865,-335,140,554,903,1182,1389,1522,1581,1568,1484,1334,1121,851,530,167,-231,-655,-1097,-1545,-1991,-2426,-2839,-3222,-3566,-3862,-4104,-4284,-4398,-4441,-4410,-4301,-4115,-3851,-3512,-3099,-2617,-2071,-1466,-810,-111,623,1382,2158,2939,3716,4479,5217,5922,6584,7194,7745,8229,8642,8977,9231,9402,9487,9488,9404,9239,8996,8679,8295,7850,7352,6809,6229,5623,5000,4370,3742,3127,2534,1973,1450,974,553,191,-106,-335,-493,-579,-592,-534,-406,-214,40,349,706,1104,1536,1992,2463,2939,3411,3869,4304,4707,5068,5380,5635,5826,5949,5999,5971,5865,5679,5413,5069,4650,4159,3601,2983,2312,1596,842,61,-738,-1545,-2350,-3142,-3913,-4651,-5348,-5995,-6585,-7110,-7564,-7942,-8241,-8458,-8592,-8641,-8608,-8494,-8303,-8040,-7710,-7320,-6878,-6391,-5869,-5320,-4755,-4183,-3615,-3059,-2525,-2022,-1558,-1141,-778,-474,-235,-63,38,67,25,-88,-268,-514,-819,-1178,-1585,-2032,-2510,-3012,-3526,-4045,-4558,-5056,-5528,-5966,-6362,-6706,-6991,-7211,-7360,-7434,-7430,-7344,-7176,-6926,-6596,-6188,-5707,-5157,-4544,-3876,-3160,-2405,-1620,-815,0}; //for 20 ms period
	uint16_t no_of_pulses;
	int16_t	avg_pulse;
	int i = 0;

	//for (i = 0; i<4096; i++) {
	//	sinus[i] = (PE_SAMPLE_TYPE)(sin(2.0*3.141592*(double)(i)*0.025)*10000.0);
	//}

	PE_ClearAndInit();

	for (i = 0; i<500; i++) {
		PE_AddSample(sinus[i]);
	}

	no_of_pulses = PE_GetNumberOfCapturedPulseIncident();
	avg_pulse = PE_GetAveragePulse();

	printf("Zarejestrowana liczba zdarzen: %d \n", no_of_pulses);
	printf("Sredni puls: %d \n", avg_pulse);

	return 0;
}
#endif
