//---------------------------------------------------------------------------------------------------
#ifndef INC_FIFO_TEMPLATE_H_
#define INC_FIFO_TEMPLATE_H_
//---------------------------------------------------------------------------------------------------
#include <synchronized.h>
#include <stdint.h>
#include <stdbool.h>
//---------------------------------------------------------------------------------------------------
/**
 *	HACK:
 *	Example implementation of header and source in files ic_fifo.h and ic_fifo.c
 *
 *	To implement FIFO for specified argument type you have to:
 *	1. Create header file (e.g. ic_fifo.h)
 *		a) make list of definitions:
 *			#define FIFO_UNDERFLOW_RET			[value returned in case of call get method when fifo is empty e.g. 0]
 *			#define FIFO_TEMPLATE_DEC			[inform template that we want to use declaration template]
 *			#define FIFO_TEMPLATE_FUN_PREFIX	[set function name prefix for specified type e.g. 'fifoi16' for int16_t fifo implementation]
 *			#define FIFO_TEMPLATE_TYPE			[set a fifo element type e.g. int16_t]
 *			#define	FIFO_TEMPLATE_STR_NAME		[set fifo structure type name associated with specified type e.g. 'FIFO_i16']
 *		b) include ic_fifo_template.h
 *			#include <ic_fifo_template.h>
 *		c) undef all definition from point 1.a
 *			#undef FIFO_UNDERFLOW_RET
 *			#undef FIFO_TEMPLATE_DEC
 *			#undef FIFO_TEMPLATE_FUN_PREFIX
 *			#undef FIFO_TEMPLATE_TYPE
 *			#undef FIFO_TEMPLATE_STR_NAME
 *		d) make points a, b and c for all needed fifo element types
 *	2. Create compilation source file (e.g. ic_fifo.c)
 *		Use template:
 *			#define FIFO_TEMPLATE_DEF
 *			#include "ic_fifo.h"
 *			#undef FIFO_TEMPLATE_DEF
 *	3. In every source file you want to use fifos include your header file (e.g. #include <ic_fifo.h>)
 *	4. To make fifo use template (e.g. for fifo with int16_t elements):
 *		a) In global scope:
 *			*) as normal variable
 *				int16_t buf[10];
 *				FIFO_i16 fifo = FIFO_INIT(buf, 10);
 *			*) as pointer
 				int16_t buf[10];
 *				FIFO_i16 *fifo = &(FIFO_i16)FIFO_INIT(buf, 10);
 *		b) In function scope
 *			int16_t buf[10];
 *			FIFO_i16 fifo = fifoi16_init(buf, 10);
 */
//---------------------------------------------------------------------------------------------------
/**
 * This macros is for valid function name creation.
 */
#define FIFO_TEMPLATE_FUN_NAME_INT1(FUN_NAME, NAME)		FUN_NAME##_##NAME
#define FIFO_TEMPLATE_FUN_NAME_INT2(FUN_NAME, NAME)		FIFO_TEMPLATE_FUN_NAME_INT1(FUN_NAME, NAME)
#define FIFO_TEMPLATE_FUN_NAME(NAME)					FIFO_TEMPLATE_FUN_NAME_INT2(FIFO_TEMPLATE_FUN_PREFIX, NAME)
//---------------------------------------------------------------------------------------------------
#endif /* INC_FIFO_TEMPLATE_H_ */
//---------------------------------------------------------------------------------------------------
//###################################################################################################
//###################################################################################################
//###################################################################################################
//###################################################################################################
//###################################################################################################
//###################################################################################################
/**
 * In this block there are all generic declarations
 */
//---------------------------------------------------------------------------------------------------
#if defined(FIFO_TEMPLATE_DEC) && defined(FIFO_TEMPLATE_FUN_PREFIX) && defined(FIFO_TEMPLATE_TYPE) && defined(FIFO_TEMPLATE_STR_NAME)
/**
 * FIFO main structure
 */
typedef struct {
	FIFO_TEMPLATE_TYPE		*arr;		///< pointer to array buffer
	uint16_t				len;		///< buffer array length
	uint16_t				count;		///< number of items in fifo
	uint16_t				pin, pout;	///< pointer in buffer for insert and withdraw location
} FIFO_TEMPLATE_STR_NAME;
//---------------------------------------------------------------------------------------------------
//Not synchronized methods & macros
//---------------------------------------------------------------------------------------------------
#ifndef FIFO_INIT
/**
 * @brief init/reinit fifo especially outside function (e.g. in global scope)
 * @param buffer	pointer to buffer array
 * @param length 	length of buffer array
 */
#define FIFO_INIT(buffer, length)	\
{					 				\
	.arr	= (buffer),				\
	.len	= (length),				\
	.count	= 0,					\
	.pin	= 0,					\
	.pout	= 0						\
}
#endif //FIFO_INIT
//---------------------------------------------------------------------------------------------------
/**
 * @brief init/reinit fifo inside function (no sync)
 * @param buffer	pointer to buffer array
 * @param len 		length of buffer array
 * @return valid filed fifo structure
 */
FIFO_TEMPLATE_STR_NAME	FIFO_TEMPLATE_FUN_NAME(init)		(FIFO_TEMPLATE_TYPE *buffer, uint16_t len);
//---------------------------------------------------------------------------------------------------
/**
 * @brief insert value to fifo (no sync)
 * @param fifo pointer to fifo instance
 * @param val value inserted to fifo
 * @return true if no error, false if fifo overflow
 */
bool				FIFO_TEMPLATE_FUN_NAME(add)		(FIFO_TEMPLATE_STR_NAME *fifo, const FIFO_TEMPLATE_TYPE val);
//---------------------------------------------------------------------------------------------------
/**
 * @brief insert array to fifo (no sync)
 * @param fifo pointer to fifo instance
 * @param array array inserted to fifo
 * @param len length of array
 * @return true if no error, false if fifo has not enough free space.
 */
bool				FIFO_TEMPLATE_FUN_NAME(addArray)		(FIFO_TEMPLATE_STR_NAME *fifo, const FIFO_TEMPLATE_TYPE *array, uint16_t len);
//---------------------------------------------------------------------------------------------------
/**
 * @brief get value from fifo (no sync)
 * @param fifo pointer to fifo instance
 * @return next value from fifo or FIFO_UNDERFLOW_RET if fifo is underflow
 */
FIFO_TEMPLATE_TYPE	FIFO_TEMPLATE_FUN_NAME(get)		(FIFO_TEMPLATE_STR_NAME *fifo);
//---------------------------------------------------------------------------------------------------
/**
 * @brief get value from fifo (no sync)
 * @param fifo pointer to fifo instance
 * @param ret pointer to free location for element from FIFO
 *
 * @return true if element was copied to ret address, false otherwise (when FIFO is empty)
 */
bool				FIFO_TEMPLATE_FUN_NAME(getSecure)	(FIFO_TEMPLATE_STR_NAME *fifo, FIFO_TEMPLATE_TYPE *ret);
//---------------------------------------------------------------------------------------------------
/**
 * @brief clear fifo (after this method fifo is empty) (no sync)
 * @param fifo pointer to fifo instance
 */
void				FIFO_TEMPLATE_FUN_NAME(clear)	(FIFO_TEMPLATE_STR_NAME *fifo);
//---------------------------------------------------------------------------------------------------
/**
 * @brief get number of emelents in fifo (no sync)
 * @param fifo pointer to fifo instance
 * @return number of elements in fifo
 */
uint16_t			FIFO_TEMPLATE_FUN_NAME(getCount)(FIFO_TEMPLATE_STR_NAME *fifo);
//---------------------------------------------------------------------------------------------------
/**
 * @brief check if fifo is empty (no sync)
 * @param fifo pointer to fifo instance
 * @return true if there is no element in fifo, false otherwise
 */
bool				FIFO_TEMPLATE_FUN_NAME(isEmpty)	(FIFO_TEMPLATE_STR_NAME *fifo);
//---------------------------------------------------------------------------------------------------
/**
 * @brief check if fifo is full (no sync)
 * @param fifo pointer to fifo instance
 * @return true if there is no free space in fifo, false otherwise
 */
bool				FIFO_TEMPLATE_FUN_NAME(isFull)	(FIFO_TEMPLATE_STR_NAME *fifo);
//---------------------------------------------------------------------------------------------------
//Synchronized methods
//---------------------------------------------------------------------------------------------------
/**
 * @brief insert value to fifo (sync)
 * @param fifo pointer to fifo instance
 * @param val value insertet to fifo
 * @return true if no error, false if fifo overflow
 */
synchronized bool				FIFO_TEMPLATE_FUN_NAME(sadd)		(FIFO_TEMPLATE_STR_NAME *fifo, const FIFO_TEMPLATE_TYPE val);
//---------------------------------------------------------------------------------------------------
/**
 * @brief insert array to fifo (no sync)
 * @param fifo pointer to fifo instance
 * @param array array inserted to fifo
 * @param len length of array
 * @return true if no error, false if fifo has not enough free space.
 */
synchronized bool				FIFO_TEMPLATE_FUN_NAME(saddArray)		(FIFO_TEMPLATE_STR_NAME *fifo, const FIFO_TEMPLATE_TYPE *array, uint16_t len);
//---------------------------------------------------------------------------------------------------
/**
 * @brief get value from fifo (sync)
 * @param fifo pointer to fifo instance
 * @return next value from fifo or FIFO_UNDERFLOW_RET if fifo is underflow
 */
synchronized FIFO_TEMPLATE_TYPE	FIFO_TEMPLATE_FUN_NAME(sget)		(FIFO_TEMPLATE_STR_NAME *fifo);
//---------------------------------------------------------------------------------------------------
/**
 * @brief get value from fifo (sync)
 * @param fifo pointer to fifo instance
 * @param ret pointer to free location for element from FIFO
 *
 * @return true if element was copied to ret address, false otherwise (when FIFO is empty)
 */
bool				FIFO_TEMPLATE_FUN_NAME(sgetSecure)	(FIFO_TEMPLATE_STR_NAME *fifo, FIFO_TEMPLATE_TYPE *ret);
//---------------------------------------------------------------------------------------------------
/**
 * @brief clear fifo (after this method fifo is empty) (sync)
 * @param fifo pointer to fifo instance
 */
synchronized void				FIFO_TEMPLATE_FUN_NAME(sclear)		(FIFO_TEMPLATE_STR_NAME *fifo);
//---------------------------------------------------------------------------------------------------
/**
 * @brief get number of emelents in fifo (sync)
 * @param fifo pointer to fifo instance
 * @return number of elements in fifo
 */
synchronized uint16_t			FIFO_TEMPLATE_FUN_NAME(sgetCount)	(FIFO_TEMPLATE_STR_NAME *fifo);
//---------------------------------------------------------------------------------------------------
/**
 * @brief check if fifo is empty (sync)
 * @param fifo pointer to fifo instance
 * @return true if there is no element in fifo, false otherwise
 */
synchronized bool				FIFO_TEMPLATE_FUN_NAME(sisEmpty)	(FIFO_TEMPLATE_STR_NAME *fifo);
//---------------------------------------------------------------------------------------------------
/**
 * @brief check if fifo is full (sync)
 * @param fifo pointer to fifo instance
 * @return true if there is no free space in fifo, false otherwise
 */
synchronized bool				FIFO_TEMPLATE_FUN_NAME(sisFull)		(FIFO_TEMPLATE_STR_NAME *fifo);
//---------------------------------------------------------------------------------------------------
#endif /* FIFO_TEMPLATE_DEC && FIFO_TEMPLATE_FUN_PREFIX && FIFO_TEMPLATE_TYPE && FIFO_TEMPLATE_STR_NAME */
//---------------------------------------------------------------------------------------------------
//###################################################################################################
//###################################################################################################
//###################################################################################################
//###################################################################################################
//###################################################################################################
//###################################################################################################
//---------------------------------------------------------------------------------------------------
#if defined(FIFO_TEMPLATE_DEF) && defined(FIFO_TEMPLATE_FUN_PREFIX) && defined(FIFO_TEMPLATE_TYPE) && defined(FIFO_TEMPLATE_STR_NAME)
//---------------------------------------------------------------------------------------------------
#include <nrf.h>	//for interrupt enable & disable in synchronized methods
//---------------------------------------------------------------------------------------------------
FIFO_TEMPLATE_STR_NAME	FIFO_TEMPLATE_FUN_NAME(init)		(FIFO_TEMPLATE_TYPE *buffer, uint16_t len) {
	FIFO_TEMPLATE_STR_NAME tmp = {
			.arr	= buffer,
			.len	= len,
			.count	= 0,
			.pin	= 0,
			.pout	= 0
	};

	return tmp;
}
//---------------------------------------------------------------------------------------------------
bool				FIFO_TEMPLATE_FUN_NAME(add)		(FIFO_TEMPLATE_STR_NAME *fifo, const FIFO_TEMPLATE_TYPE val) {
	if (fifo->count == fifo->len) {
		return false;
	}

	fifo->arr[fifo->pin] = val;
	if (++(fifo->pin) >= fifo->len)
		fifo->pin = 0;
	fifo->count++;

	return true;
}
//---------------------------------------------------------------------------------------------------
bool				FIFO_TEMPLATE_FUN_NAME(addArray)		(FIFO_TEMPLATE_STR_NAME *fifo, const FIFO_TEMPLATE_TYPE *array, uint16_t len) {
	if (fifo->count + len > fifo->len) {
		return false;
	}
	for (uint16_t u = 0; u < len; u++) {
		fifo->arr[fifo->pin] = *array;
		array++;
		//this can operate quite faster... no need to check this every time
		if (++(fifo->pin) >= fifo->len)
			fifo->pin = 0;
	}
	fifo->count += len;
	return true;
}
//---------------------------------------------------------------------------------------------------
FIFO_TEMPLATE_TYPE	FIFO_TEMPLATE_FUN_NAME(get)		(FIFO_TEMPLATE_STR_NAME *fifo) {
	if (fifo->count == 0)
		return FIFO_UNDERFLOW_RET;

	fifo->count--;
	int16_t tmp = fifo->pout;
	if (++(fifo->pout) >= fifo->len)
		fifo->pout = 0;

	return fifo->arr[tmp];
}
//---------------------------------------------------------------------------------------------------
bool 				FIFO_TEMPLATE_FUN_NAME(getSecure)	(FIFO_TEMPLATE_STR_NAME *fifo, FIFO_TEMPLATE_TYPE *ret) {
	if (fifo->count == 0)
		return false;

	fifo->count--;
	int16_t tmp = fifo->pout;
	if (++(fifo->pout) >= fifo->len)
		fifo->pout = 0;

	*ret = fifo->arr[tmp];

	return true;
}
//---------------------------------------------------------------------------------------------------
void				FIFO_TEMPLATE_FUN_NAME(clear)	(FIFO_TEMPLATE_STR_NAME *fifo) {
	fifo->count	= 0;
	fifo->pin	= 0;
	fifo->pout	= 0;
}
//---------------------------------------------------------------------------------------------------
uint16_t			FIFO_TEMPLATE_FUN_NAME(getCount)(FIFO_TEMPLATE_STR_NAME *fifo) {
	return fifo->count;
}
//---------------------------------------------------------------------------------------------------
bool				FIFO_TEMPLATE_FUN_NAME(isEmpty)	(FIFO_TEMPLATE_STR_NAME *fifo) {
	if (fifo->count == 0)
		return true;
	return false;
}
//---------------------------------------------------------------------------------------------------
bool				FIFO_TEMPLATE_FUN_NAME(isFull)	(FIFO_TEMPLATE_STR_NAME *fifo) {
	if (fifo->count == fifo->len)
		return true;
	return false;
}
//---------------------------------------------------------------------------------------------------
synchronized bool				FIFO_TEMPLATE_FUN_NAME(sadd)		(FIFO_TEMPLATE_STR_NAME *fifo, const FIFO_TEMPLATE_TYPE val) {
	bool tmp;
	__disable_irq();
	tmp = FIFO_TEMPLATE_FUN_NAME(add)(fifo, val);
	__enable_irq();
	return tmp;
}
//---------------------------------------------------------------------------------------------------
bool				FIFO_TEMPLATE_FUN_NAME(saddArray)		(FIFO_TEMPLATE_STR_NAME *fifo, const FIFO_TEMPLATE_TYPE *array, uint16_t len) {
	if (FIFO_TEMPLATE_FUN_NAME(sgetCount)(fifo) + len > fifo->len) {
		return false;
	}
	for (uint16_t u = 0; u < len; u++) {
		FIFO_TEMPLATE_FUN_NAME(sadd)(fifo, array[u]);
	}
	return true;
}
//---------------------------------------------------------------------------------------------------
synchronized FIFO_TEMPLATE_TYPE	FIFO_TEMPLATE_FUN_NAME(sget)		(FIFO_TEMPLATE_STR_NAME *fifo) {
	FIFO_TEMPLATE_TYPE tmp;
	__disable_irq();
	tmp = FIFO_TEMPLATE_FUN_NAME(get)(fifo);
	__enable_irq();
	return tmp;
}
//---------------------------------------------------------------------------------------------------
synchronized bool	FIFO_TEMPLATE_FUN_NAME(sgetSecure)		(FIFO_TEMPLATE_STR_NAME *fifo, FIFO_TEMPLATE_TYPE *ret) {
	bool tmp;
	__disable_irq();
	tmp = FIFO_TEMPLATE_FUN_NAME(getSecure)(fifo, ret);
	__enable_irq();
	return tmp;
}
//---------------------------------------------------------------------------------------------------
synchronized void				FIFO_TEMPLATE_FUN_NAME(sclear)		(FIFO_TEMPLATE_STR_NAME *fifo) {
	__disable_irq();
	FIFO_TEMPLATE_FUN_NAME(clear)(fifo);
	__enable_irq();
}
//---------------------------------------------------------------------------------------------------
synchronized uint16_t			FIFO_TEMPLATE_FUN_NAME(sgetCount)	(FIFO_TEMPLATE_STR_NAME *fifo) {
	bool tmp;
	__disable_irq();
	tmp = FIFO_TEMPLATE_FUN_NAME(getCount)(fifo);
	__enable_irq();
	return tmp;
}
//---------------------------------------------------------------------------------------------------
synchronized bool				FIFO_TEMPLATE_FUN_NAME(sisEmpty)	(FIFO_TEMPLATE_STR_NAME *fifo) {
	bool tmp;
	__disable_irq();
	tmp = FIFO_TEMPLATE_FUN_NAME(isEmpty)(fifo);
	__enable_irq();
	return tmp;
}
//---------------------------------------------------------------------------------------------------
synchronized bool				FIFO_TEMPLATE_FUN_NAME(sisFull)		(FIFO_TEMPLATE_STR_NAME *fifo) {
	bool tmp;
	__disable_irq();
	tmp = FIFO_TEMPLATE_FUN_NAME(isFull)(fifo);
	__enable_irq();
	return tmp;
}
//---------------------------------------------------------------------------------------------------
#endif /* FIFO_TEMPLATE_DEF && FIFO_TEMPLATE_FUN_PREFIX && FIFO_TEMPLATE_TYPE && FIFO_TEMPLATE_STR_NAME */
//---------------------------------------------------------------------------------------------------
