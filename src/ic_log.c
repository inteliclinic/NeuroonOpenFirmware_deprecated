//---------------------------------------------------------------------------------------------------
#define LOG_FRAME_TEMPLATE_DEF
#include <ic_log.h>
#include <ic_crc8.h>
#include <string.h>
#include <ic_fifo.h>
#include <global_conf.h>
//---------------------------------------------------------------------------------------------------
static uint8_t	stringFrameFC 	= 0;
static uint8_t	rawPageFrameFC	= 0;

static uint32_t	reportSec		= 0;
static uint8_t	reportFC		= 0;
static uint8_t	reportHeader[]	= {0xD4, 0x2D};
//---------------------------------------------------------------------------------------------------
bool logf_send(LOGFuni *lf) {
	return logf_sendex(lf, lf->payload);
}
//---------------------------------------------------------------------------------------------------
bool logf_sendex(LOGFuni *lf, void *payload) {
	//Test free space
	if (UARTLOG_getFreeSpace() < (uint16_t)(3 + lf->psize))
		return false;

	//Compute crc8
	uint8_t crc8 = 0x00;
	crc8 = crc8_next(crc8, lf->id);
	crc8 = crc8_next(crc8, lf->fc);
	crc8 = crc8_tabex(crc8, payload, lf->psize);

	//Insert to uart-log sender
	UARTLOG_sendByte(lf->id);
	UARTLOG_sendByte(lf->fc);
	UARTLOG_send(payload, lf->psize);
	UARTLOG_sendByte(crc8);

	//Increment frame number
	lf->fc++;

	return true;
}
//---------------------------------------------------------------------------------------------------
bool logf_sendStr(const char* txt) {
	//frame: <ID><frame-counter><length><string><crc8>
	//test string
	if (txt == 0 || txt[0] == 0)
		return false;

	//crc8 and length computation
	uint8_t crc8 = 0x00;
	crc8 = crc8_next(crc8, LOG_FRAME_ID_STRING);
	crc8 = crc8_next(crc8, stringFrameFC);
	int len = strlen(txt);
	if (len > 255)
		len = 255;

	crc8 = crc8_next(crc8, (uint8_t) len);
	for (int i = 0; i < len; i++) {
		crc8 = crc8_next(crc8, txt[i]);
	}
	//Test free space
	if (UARTLOG_getFreeSpace() < (uint16_t) (4 + len))
		return false;

	//Insert to uart-log sender
	UARTLOG_sendByte(LOG_FRAME_ID_STRING);
	UARTLOG_sendByte(stringFrameFC);
	UARTLOG_sendByte((uint8_t)len);
	UARTLOG_send((const uint8_t *)txt, len);
	UARTLOG_sendByte(crc8);
	stringFrameFC++;
	return true;
}
//---------------------------------------------------------------------------------------------------
void logf_clearStrFC() {
	stringFrameFC = 0;
}
//---------------------------------------------------------------------------------------------------
void log_initSecondReport(uint32_t sec) {
	reportSec = sec;
	reportFC = 0;
}
//---------------------------------------------------------------------------------------------------
bool log_secondReport() {
	uint8_t crc8 = 0x00;
	crc8 = crc8_next(0x00, LOG_FRAME_ID_REPORT_SECOND);
	crc8 = crc8_next(crc8, reportFC);
	crc8 = crc8_tabex(crc8, reportHeader, sizeof(reportHeader));
	crc8 = crc8_tabex(crc8, (uint8_t*)&reportSec, 4);

	UARTLOG_sendByte(LOG_FRAME_ID_REPORT_SECOND);
	UARTLOG_sendByte(reportFC);
	UARTLOG_send(reportHeader, sizeof(reportHeader));
	UARTLOG_send((uint8_t*)&reportSec, 4);
	UARTLOG_sendByte(crc8);

	reportSec++;
	reportFC++;

	return true;
}
//---------------------------------------------------------------------------------------------------
bool logf_sendRawPage(LogRawPageStr *frame) {
	//frame: <ID><frame-counter><page-data><page-id><crc8>
	//Test free space
	if (UARTLOG_getFreeSpace() < (uint16_t) (3 + 258))
		return false;

	//crc8 and length computation
	uint8_t crc8 = 0x00;
	crc8 = crc8_next(crc8, LOG_FRAME_ID_RAW_PAGE);
	crc8 = crc8_next(crc8, stringFrameFC);
	crc8 = crc8_tabex(crc8, (uint8_t*)frame, 258);

	//Insert to uart-log sender
	UARTLOG_sendByte(LOG_FRAME_ID_RAW_PAGE);
	UARTLOG_sendByte(stringFrameFC);
	UARTLOG_send((uint8_t*)frame, 258);
	UARTLOG_sendByte(crc8);
	stringFrameFC++;

	return true;
}
//---------------------------------------------------------------------------------------------------
void logf_clearRawPageFC() {
	rawPageFrameFC = 0;
}
//---------------------------------------------------------------------------------------------------






//---------------------------------------------------------------------------------------------------
static bool logch[LOG_CHANNELS_NUMBER];
//---------------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------------
void log_enable(int code) {
	if (code >= LOG_CHANNELS_NUMBER)
		return;
	logch[code] = true;
}
//---------------------------------------------------------------------------------------------------
void log_disable(int code) {
	if (code >= LOG_CHANNELS_NUMBER)
		return;
	logch[code] = false;
}
//---------------------------------------------------------------------------------------------------
bool log_isActive(int code) {
	if (code >= LOG_CHANNELS_NUMBER)
		return false;
	return logch[code];
}
//---------------------------------------------------------------------------------------------------





//---------------------------------------------------------------------------------------------------
#ifdef LOG_USE_IRLED
	static LOGFi32 	lf_irled = LOG_SIGNAL_FRAME_INIT(LOG_FRAME_ID_RAW_IRLED);
	static int32_t buf_irled[25];
	static FIFO_i32	fifo_irled = FIFO_INIT(buf_irled, sizeof(buf_irled)/sizeof(int32_t));
#endif //LOG_USE_IRLED
//---------------------------------------------------------------------------------------------------
void log_irled_clear() {
#ifdef LOG_USE_IRLED
	if (logch[LOG_IRLED]) {
		logfi32_clear(&lf_irled);
		fifoi32_clear(&fifo_irled);
	}
#endif //LOG_USE_IRLED
}
//---------------------------------------------------------------------------------------------------
bool log_irled(int32_t sample) {
#ifdef LOG_USE_IRLED
	if (logch[LOG_IRLED])
		return logfi32_addSample(&lf_irled, sample);
#endif //LOG_USE_IRLED
	return false;
}
//---------------------------------------------------------------------------------------------------
bool log_irled_byfifo(int32_t sample) {
#ifdef LOG_USE_IRLED
	if (logch[LOG_IRLED])
		return fifoi32_sadd(&fifo_irled, sample);
#endif //LOG_USE_IRLED
	return false;
}
//---------------------------------------------------------------------------------------------------
bool log_irled_flushFifo() {
#ifdef LOG_USE_IRLED
	if (logch[LOG_IRLED]) {
		while (fifoi32_sisEmpty(&fifo_irled) == false) {
			logfi32_addSample(&lf_irled, fifoi32_sget(&fifo_irled));
		}
		return true;
	}
#endif //LOG_USE_IRLED
	return false;
}
//---------------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------------
#ifdef LOG_USE_REDLED
	static LOGFi32 	lf_redled = LOG_SIGNAL_FRAME_INIT(LOG_FRAME_ID_RAW_REDLED);
	static int32_t buf_redled[25];
	static FIFO_i32	fifo_redled = FIFO_INIT(buf_redled, sizeof(buf_redled)/sizeof(int32_t));
#endif //LOG_USE_REDLED
//---------------------------------------------------------------------------------------------------
void log_redled_clear() {
#ifdef LOG_USE_REDLED
	if (logch[LOG_REDLED]) {
		logfi32_clear(&lf_redled);
		fifoi32_clear(&fifo_redled);
	}
#endif //LOG_USE_REDLED
}
//---------------------------------------------------------------------------------------------------
bool log_redled(int32_t sample) {
#ifdef LOG_USE_REDLED
	if (logch[LOG_REDLED])
		return logfi32_addSample(&lf_redled, sample);
#endif //LOG_USE_REDLED
	return false;
}
//---------------------------------------------------------------------------------------------------
bool log_redled_byfifo(int32_t sample) {
#ifdef LOG_USE_REDLED
	if (logch[LOG_REDLED])
		return fifoi32_sadd(&fifo_redled, sample);
#endif //LOG_USE_REDLED
	return false;
}
//---------------------------------------------------------------------------------------------------
bool log_redled_flushFifo() {
#ifdef LOG_USE_REDLED
	if (logch[LOG_REDLED]) {
		while (fifoi32_sisEmpty(&fifo_redled) == false) {
			logfi32_addSample(&lf_redled, fifoi32_sget(&fifo_redled));
		}
		return true;
	}
#endif //LOG_USE_REDLED
	return false;
}
//---------------------------------------------------------------------------------------------------





//---------------------------------------------------------------------------------------------------
#ifdef LOG_USE_NN
	static LOGFi32 	lf_nn = LOG_SIGNAL_FRAME_INIT(LOG_FRAME_ID_RAW_NN);
	static int32_t buf_nn[25];
	static FIFO_i32	fifo_nn = FIFO_INIT(buf_nn, sizeof(buf_nn)/sizeof(int32_t));
#endif //LOG_USE_REDLED
//---------------------------------------------------------------------------------------------------
void log_nn_clear() {
#ifdef LOG_USE_NN
	if (logch[LOG_NN]) {
		logfi32_clear(&lf_nn);
		fifoi32_clear(&fifo_nn);
	}
#endif //LOG_USE_NN
}
//---------------------------------------------------------------------------------------------------
bool log_nn(int32_t sample) {
#ifdef LOG_USE_NN
	if (logch[LOG_NN])
		return logfi32_addSample(&lf_nn, sample);
#endif //LOG_USE_NN
	return false;
}
//---------------------------------------------------------------------------------------------------
bool log_nn_byfifo(int32_t sample) {
#ifdef LOG_USE_NN
	if (logch[LOG_NN])
		return fifoi32_sadd(&fifo_nn, sample);
#endif //LOG_USE_NN
	return false;
}
//---------------------------------------------------------------------------------------------------
bool log_nn_flushFifo() {
#ifdef LOG_USE_NN
	if (logch[LOG_NN]) {
		while (fifoi32_sisEmpty(&fifo_nn) == false) {
			logfi32_addSample(&lf_nn, fifoi32_sget(&fifo_nn));
		}
		return true;
	}
#endif //LOG_USE_NN
	return false;
}
//---------------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------------
#ifdef LOG_USE_BUSTIMER
	static LOGFu16 	lf_bustimer = LOG_SIGNAL_FRAME_INIT(LOG_FRAME_ID_BUSTIMER);
	static uint16_t buf_bustimer[25];
	static FIFO_u16	fifo_bustimer = FIFO_INIT(buf_bustimer, sizeof(buf_bustimer)/sizeof(uint16_t));
#endif //LOG_USE_BUSTIMER
//---------------------------------------------------------------------------------------------------
void log_bustimer_clear() {
#ifdef LOG_USE_BUSTIMER
	if (logch[LOG_BUSTIMER]) {
		logfu16_clear(&lf_bustimer);
		fifou16_clear(&fifo_bustimer);
	}
#endif //LOG_USE_BUSTIMER
}
//---------------------------------------------------------------------------------------------------
bool log_bustimer(uint16_t sample) {
#ifdef LOG_USE_BUSTIMER
	if (logch[LOG_BUSTIMER])
		return logfu16_addSample(&lf_bustimer, sample);
#endif //LOG_USE_BUSTIMER
	return false;
}
//---------------------------------------------------------------------------------------------------
bool log_bustimer_byfifo(uint16_t sample) {
#ifdef LOG_USE_BUSTIMER
	if (logch[LOG_BUSTIMER])
		return fifou16_sadd(&fifo_bustimer, sample);
#endif //LOG_USE_BUSTIMER
	return false;
}
//---------------------------------------------------------------------------------------------------
bool log_bustimer_flushFifo() {
#ifdef LOG_USE_BUSTIMER
	if (logch[LOG_BUSTIMER]) {
		while (fifou16_sisEmpty(&fifo_bustimer) == false) {
			logfu16_addSample(&lf_bustimer, fifou16_sget(&fifo_bustimer));
		}
		return true;
	}
#endif //LOG_USE_BUSTIMER
	return false;
}
//---------------------------------------------------------------------------------------------------





//---------------------------------------------------------------------------------------------------
#ifdef LOG_USE_EEG_FIFO_COUNT
	static LOGFu16 	lf_eegFIFOcount = LOG_SIGNAL_FRAME_INIT(LOG_FRAME_ID_EEG_FIFO_COUNT);
	static uint16_t buf_eegFIFOcount[25];
	static FIFO_u16	fifo_eegFIFOcount = FIFO_INIT(buf_eegFIFOcount, sizeof(buf_eegFIFOcount)/sizeof(uint16_t));
#endif //LOG_USE_EEG_FIFO_COUNT
//---------------------------------------------------------------------------------------------------
void log_eegFIFOcount_clear() {
#ifdef LOG_USE_EEG_FIFO_COUNT
	if (logch[LOG_EEG_FIFO_COUNT]) {
		logfu16_clear(&lf_eegFIFOcount);
		fifou16_clear(&fifo_eegFIFOcount);
	}
#endif //LOG_USE_EEG_FIFO_COUNT
}
//---------------------------------------------------------------------------------------------------
bool log_eegFIFOcount(uint16_t sample) {
#ifdef LOG_USE_EEG_FIFO_COUNT
	if (logch[LOG_EEG_FIFO_COUNT])
		return logfu16_addSample(&lf_eegFIFOcount, sample);
#endif //LOG_USE_EEG_FIFO_COUNT
	return false;
}
//---------------------------------------------------------------------------------------------------
bool log_eegFIFOcount_byfifo(uint16_t sample) {
#ifdef LOG_USE_EEG_FIFO_COUNT
	if (logch[LOG_EEG_FIFO_COUNT])
		return fifou16_sadd(&fifo_eegFIFOcount, sample);
#endif //LOG_USE_EEG_FIFO_COUNT
	return false;
}
//---------------------------------------------------------------------------------------------------
bool log_eegFIFOcount_flushFifo() {
#ifdef LOG_USE_EEG_FIFO_COUNT
	if (logch[LOG_EEG_FIFO_COUNT]) {
		while (fifou16_sisEmpty(&fifo_eegFIFOcount) == false) {
			logfu16_addSample(&lf_eegFIFOcount, fifou16_sget(&fifo_eegFIFOcount));
		}
		return true;
	}
#endif //LOG_USE_EEG_FIFO_COUNT
	return false;
}
//---------------------------------------------------------------------------------------------------


