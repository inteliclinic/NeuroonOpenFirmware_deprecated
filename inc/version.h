//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------

#ifndef INC_VERSION_H_
#define INC_VERSION_H_

#define MAKE_UINT32(hh, hl, lh, ll) (((0xFF&hh)<<24) | ((0xFF&hl)<<16) | ((0xFF&lh)<<8) | (0xFF&ll))

#define APP_VERSION		MAKE_UINT32(2, 1, 1, 5)
#define APP_TXT_VERSION	NEURON_FIRM_VERSION

#endif /* INC_VERSION_H_ */
