//---------------------------------------------------------------------------------------------------
#ifndef NRF51822_LIB_INC_IC_CBUFFER_TEMPLATE_H_
#define NRF51822_LIB_INC_IC_CBUFFER_TEMPLATE_H_
//---------------------------------------------------------------------------------------------------
#include <synchronized.h>
#include <stdint.h>
#include <stdbool.h>
//---------------------------------------------------------------------------------------------------
/**
 *	HACK:
 *	Example implementation of header and source in files ic_cbuffer.h and ic_buffer.c
 *
 *	1.	Create header file (e.g. ic_cbuffer.h)
 *		a) make list of definitions:
 *			#define CBUFFER_TEMPLATE_DEC				[inform template that we want to use declaration template]
 *			#define CBUFFER_TEMPLATE_FUN_PREFIX			[set function name prefix for specified type e.g. 'cbi16' for int16_t cbuffer implementation]
 *			#define CBUFFER_TEMPLATE_TYPE				[set a cbuffer element type e.g. int16_t]
 *			#define	CBUFFER_TEMPLATE_STR_NAME				[set cbuffer structure type name associated with specified type e.g. 'CBi16']
 *			#define CBUFFER_DEFAULT_RET					[set a default return value for get & clear method]
 *			#define CBUFFER_TEMPLATE_SUM_TYPE			[optional: use for fast-sum implementation; see description below]
 *			#define CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE	[optional: use for weighted-sum implementation; see description below]
 *		b) include ic_cbuffer_template.h
 *			#include <ic_cbuffer_template.h>
 *		c) there is no need to undef above definitions; this template contains all needed undefs (at this file end)
 *		d) make (a) and (b) for all needed cyclic-buffer types
 *		e) struct element type
 *			For struct element type please not define #CBUFFER_TEMPLATE_SUM_TYPE and #CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE.
 *			For example structure type:
 *
 *			typedef struct {
 *				uint32_t a, b, c;
 *			} MyType;
 *
 *			can be use in cyclic-buffer by defining:
 *
 *			#define CBUFFER_TEMPLATE_DEC
 *			#define CBUFFER_TEMPLATE_FUN_PREFIX			myType
 *			#define CBUFFER_TEMPLATE_TYPE				MyType
 *			#define	CBUFFER_TEMPLATE_STR_NAME			CB_MyType
 *			#define CBUFFER_DEFAULT_RET					(MyType){0, 0, 0}
 *
 *			or for #CBUFFER_DEFAULT_RET:
 *
 *			#define CBUFFER_DEFAULT_RET					(MyType){.a = 0, .b = 0, .c = 0}
 *
 *	2.	Create compilation source file (e.g. ic_cbuffer.c)
 *		Just use simple template:
 *			#define CBUFFER_TEMPLATE_DEF
 *			#include <ic_buffer.h>
 *			#undef CBUFFER_TEMPLATE_DEF
 *	3.	In every source file you want to use cbuffer include your header file (e.g. #include <ic_cbuffer.h>)
 *	4.	To make cbuffer use template (e.g. for cbuffer with int16_t elements):
 *		*) as normal variable
 *			int16_t	raw_buf[10];
 *			CBi16	cbuf = CBUFFER_INIT(raw_buf, 10);
 *		*) as pointer variable
 *			int16_t	raw_buf[10];
 *			CBi16	*pcbuf = &(CBi16)CBUFFER_INIT(raw_buf, 10);
 *		*) using special macro
 *			CBi16 *pcbuf = CBUFFER_ALLOC(CBi16, 10);
 *	5.	Flag CBUFFER_TEMPLATE_SUM_TYPE is used to enable fast-sum feature. Fast-sum update sum of element in buffer
 *		by analyze inserted and deleted element during 'add' method. During flag definition man must specify sum
 *		type. E.g.
 *			#define CBUFFER_TEMPLATE_SUM_TYPE int32_t
 *	6.	Flag CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE is used to enable feature of calculating weighted sum
 *		of element in cyclic buffer.
 */
//---------------------------------------------------------------------------------------------------
#define CBUFFER_TEMPLATE_FUN_NAME_INT1(FUN_NAME, NAME)		FUN_NAME##_##NAME
#define CBUFFER_TEMPLATE_FUN_NAME_INT2(FUN_NAME, NAME)		CBUFFER_TEMPLATE_FUN_NAME_INT1(FUN_NAME, NAME)
#define CBUFFER_TEMPLATE_FUN_NAME(NAME)						CBUFFER_TEMPLATE_FUN_NAME_INT2(CBUFFER_TEMPLATE_FUN_PREFIX, NAME)
//---------------------------------------------------------------------------------------------------
#endif /* NRF51822_LIB_INC_IC_CBUFFER_TEMPLATE_H_ */
//---------------------------------------------------------------------------------------------------
//Un-comment undermentioned definition only during testing/changing/developing. Have to be commented during normal operation!
//#define CBUFFER_DEVELOPING
#ifdef CBUFFER_DEVELOPING
	#define CBUFFER_TEMPLATE_DEC
	#define CBUFFER_TEMPLATE_DEF
	#define CBUFFER_TEMPLATE_FUN_PREFIX			cbi16
	#define CBUFFER_TEMPLATE_TYPE				int16_t
	#define CBUFFER_TEMPLATE_STR_NAME			CBi16
	#define	CBUFFER_DEFAULT_RET					0
	#define CBUFFER_TEMPLATE_SUM_TYPE			int32_t
	#define CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE	int32_t
#endif //CBUFFER_DEVELOPING
//---------------------------------------------------------------------------------------------------
//#############################################################################################################################################################
//---------------------------------------------------------------------------------------------------
#if defined(CBUFFER_TEMPLATE_DEC) && defined(CBUFFER_TEMPLATE_FUN_PREFIX) && defined(CBUFFER_TEMPLATE_TYPE) && defined(CBUFFER_TEMPLATE_STR_NAME)
	//---------------------------------------------------------------------------------------------------
	/**
	 * CYCLIC BUFFER main structure
	 */
	typedef struct {
		CBUFFER_TEMPLATE_TYPE			*buf;		///< pointer to array buffer
		uint16_t						len;		///< buffer array length
		uint16_t						pin;		///< pointer in buffer for insert location
		uint32_t						count;		///< number of items in fifo
		#ifdef CBUFFER_TEMPLATE_SUM_TYPE
			CBUFFER_TEMPLATE_SUM_TYPE	fastSum;	///< fast sum - for equal weights of input values
		#endif //CBUFFER_TEMPLATE_SUM_TYPE
	} CBUFFER_TEMPLATE_STR_NAME;
	//---------------------------------------------------------------------------------------------------
	#ifndef CBUFFER_INIT
		//---------------------------------------------------------------------------------------------------
		/**
		 * @brief	init/reinit cyclic buffer especially outside function (e.g. in global scope)
		 * @param	buffer	pointer to buffer array
		 * @param	length	length of buffer array
		 */
		#define CBUFFER_INIT(buffer, length)	\
		{					 					\
			.buf		= (buffer),				\
			.len		= (length)				\
		}

		//---------------------------------------------------------------------------------------------------
	#endif //CBUFFER_INIT
	//---------------------------------------------------------------------------------------------------
	#ifndef CBUFFER_ALLOC
		//---------------------------------------------------------------------------------------------------
		/**
		 * @brief	Create circular-buffer and return pointer to it.
		 * 			Can be used in global and local scope. Initialize buffer with zeros.
		 * @param	type	type of circular-buffer
		 * @param	length	length of buffer array
		 */
		#define CBUFFER_ALLOC(type, length)	\
		&(type){					 					\
			.buf		= (void*)(uint8_t[]){[0 ... (length*sizeof(*((type*)0)->buf)-1)] = 0},				\
			.len		= (length)				\
		}

		//---------------------------------------------------------------------------------------------------
	#endif //CBUFFER_ALLOC
	//---------------------------------------------------------------------------------------------------
	/**
	 * @brief	clear cyclic-buffer (also clear internal buffer)
	 *
	 * @param	cb		pointer to instance
	 */
	void
	CBUFFER_TEMPLATE_FUN_NAME(clear)			(CBUFFER_TEMPLATE_STR_NAME *cb);

	synchronized void
	CBUFFER_TEMPLATE_FUN_NAME(sclear)			(CBUFFER_TEMPLATE_STR_NAME *cb);
	//---------------------------------------------------------------------------------------------------
		/**
	 * @brief	clear cyclic-buffer (also clear internal buffer) with specified value
	 *
	 * @param	cb		pointer to instance
	 */
	void
	CBUFFER_TEMPLATE_FUN_NAME(clearex)			(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE val);

	synchronized void
	CBUFFER_TEMPLATE_FUN_NAME(sclearex)			(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE val);
	//---------------------------------------------------------------------------------------------------
	/**
	 * @brief	resize circulat-buffer (also clear internal buffer)
	 * 			If newSize param is zero, function do nothing.
	 *
	 * @param	cb		pointer to instance
	 * @param	newSize new circular-buffer size
	 */
	void
	CBUFFER_TEMPLATE_FUN_NAME(resize)			(CBUFFER_TEMPLATE_STR_NAME *cb, uint16_t newSize);

	synchronized void
	CBUFFER_TEMPLATE_FUN_NAME(sresize)			(CBUFFER_TEMPLATE_STR_NAME *cb, uint16_t newSize);
	//---------------------------------------------------------------------------------------------------
	/**
	 * @brief	add element to cyclic buffer
	 *
	 * @param	cb		pointer to instance
	 * @param	val 	value inserted to cyclic buffer
	 */
	void
	CBUFFER_TEMPLATE_FUN_NAME(add)				(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE val);

	synchronized void
	CBUFFER_TEMPLATE_FUN_NAME(sadd)				(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE val);
	//---------------------------------------------------------------------------------------------------
	/**
	 * @brief	get element from cyclic buffer
	 *
	 * @param	cb		pointer to instance
	 * @param	pos 	element position in buffer
	 *
	 * @return value of pointed element or #CBUFFER_DEFAULT_RET if $pos is out of range
	 */
	CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(get)				(CBUFFER_TEMPLATE_STR_NAME *cb, uint16_t pos);

	synchronized CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(sget)				(CBUFFER_TEMPLATE_STR_NAME *cb, uint16_t pos);
	//---------------------------------------------------------------------------------------------------
	/**
	 * @brief	get last added element from cyclic buffer
	 *
	 * @param	cb		pointer to instance
	 *
	 * @return value of last added element or #CBUFFER_DEFAULT_RET if cyclic buffer is empty
	 */
	CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(getNewestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb);

	synchronized CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(sgetNewestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb);
	//---------------------------------------------------------------------------------------------------
	/**
	 * @brief	get the oldest element from cyclic buffer
	 *
	 * @param	cb		pointer to instance
	 *
	 * @return value of the oldest element or #CBUFFER_DEFAULT_RET if cyclic buffer is empty
	 */
	CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(getOldestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb);

	synchronized CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(sgetOldestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb);
	//---------------------------------------------------------------------------------------------------
	/**
	 * @brief	get N last elements from cyclic buffer
	 *
	 * @param	cb		pointer to instance
	 * @param	array	pointer to array of structure of data, that will be written by function
	 * @param	n 		number of last elements
	 */
	//CBUFFER_TEMPLATE_TYPE*
	void
	CBUFFER_TEMPLATE_FUN_NAME(getNNewestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE* array, uint16_t n);

	synchronized void //CBUFFER_TEMPLATE_TYPE*
	CBUFFER_TEMPLATE_FUN_NAME(sgetNNewestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE* array, uint16_t n);
	//---------------------------------------------------------------------------------------------------
	/**
	 * @brief	get number of elements inserted into cyclic buffer
	 *
	 * @param	cb		pointer to instance
	 *
	 * @return number of elements every inserted into cyclic buffer
	 */
	uint32_t
	CBUFFER_TEMPLATE_FUN_NAME(getCounter)		(CBUFFER_TEMPLATE_STR_NAME *cb);

	synchronized uint32_t
	CBUFFER_TEMPLATE_FUN_NAME(sgetCounter)		(CBUFFER_TEMPLATE_STR_NAME *cb);
	//---------------------------------------------------------------------------------------------------
	#ifdef CBUFFER_TEMPLATE_SUM_TYPE
		//---------------------------------------------------------------------------------------------------
		/**
		 * @brief	reset fastSum of inserted elements
		 *
		 * @param	cb		pointer to instance
		 * @param	fSum	new fastSum value
		 *
		 * @see		recalcFastSum(...)
		 */
		void
		CBUFFER_TEMPLATE_FUN_NAME(resetFastSum)		(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_SUM_TYPE fSum);

		synchronized void
		CBUFFER_TEMPLATE_FUN_NAME(sresetFastSum)	(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_SUM_TYPE fSum);
		//---------------------------------------------------------------------------------------------------
		/**
		 * @brief	recalculate fastSum value of all elements in buffer
		 *
		 * @param	cb		pointer to instance
		 *
		 * @return	new value of recalculated fast sum
		 */
		CBUFFER_TEMPLATE_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(recalcFastSum)	(CBUFFER_TEMPLATE_STR_NAME *cb);

		synchronized CBUFFER_TEMPLATE_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(srecalcFastSum)	(CBUFFER_TEMPLATE_STR_NAME *cb);
		//---------------------------------------------------------------------------------------------------
		/**
		 * @brief	get fastSum value
		 *
		 * @param	cb		pointer to instance
		 *
		 * @return	value of fastSum
		 */
		CBUFFER_TEMPLATE_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(getFastSum)		(CBUFFER_TEMPLATE_STR_NAME *cb);

		synchronized CBUFFER_TEMPLATE_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(sgetFastSum)		(CBUFFER_TEMPLATE_STR_NAME *cb);
		//---------------------------------------------------------------------------------------------------
		/**
		 * @brief	get raw mean value
		 * 			This function return value of fastSum divided by circular-buffer length.
		 *
		 * @param	cb		pointer to instance
		 *
		 * @return	value of raw mean
		 */
		CBUFFER_TEMPLATE_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(getRawMean)		(CBUFFER_TEMPLATE_STR_NAME *cb);

		synchronized CBUFFER_TEMPLATE_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(sgetRawMean)		(CBUFFER_TEMPLATE_STR_NAME *cb);
		//---------------------------------------------------------------------------------------------------
		/**
		 * @brief	get real mean value
		 * 			This function return value of fastSum divided by real number of elements in circular-buffer.
		 * 			If circular-buffer is empty, then return value is equal to #CBUFFER_DEFAULT_RET.
		 *
		 * @param	cb		pointer to instance
		 *
		 * @return	value of real mean
		 */
		CBUFFER_TEMPLATE_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(getRealMean)		(CBUFFER_TEMPLATE_STR_NAME *cb);

		synchronized CBUFFER_TEMPLATE_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(sgetRealMean)		(CBUFFER_TEMPLATE_STR_NAME *cb);
		//---------------------------------------------------------------------------------------------------
	#endif //CBUFFER_TEMPLATE_SUM_TYPE
	//---------------------------------------------------------------------------------------------------
	#ifdef CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE
		//---------------------------------------------------------------------------------------------------
		/**
		 * @brief	calculate weighted sum of elements in buffer
		 *
		 * @param	cb		pointer to instance
		 * @param	weights	pointer to buffer (element type #CBUFFER_TEMPLATE_TYPE) containing weights,
		 * 					buffer have to have length equal to internal cyclic buffer length
		 */
		CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(weightedSumFromNewest)		(CBUFFER_TEMPLATE_STR_NAME *cb, const CBUFFER_TEMPLATE_TYPE *weights);

		synchronized CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(sweightedSumFromNewest)		(CBUFFER_TEMPLATE_STR_NAME *cb, const CBUFFER_TEMPLATE_TYPE *weights);
		//---------------------------------------------------------------------------------------------------
	#endif //CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE
	//---------------------------------------------------------------------------------------------------
#endif //defined(CBUFFER_TEMPLATE_DEC) && defined(CBUFFER_TEMPLATE_FUN_PREFIX) && defined(CBUFFER_TEMPLATE_TYPE) && defined(CBUFFER_TEMPLATE_STR_NAME)
//---------------------------------------------------------------------------------------------------
//#############################################################################################################################################################
//---------------------------------------------------------------------------------------------------
#if defined(CBUFFER_TEMPLATE_DEF) && defined(CBUFFER_TEMPLATE_FUN_PREFIX) && defined(CBUFFER_TEMPLATE_TYPE) && defined(CBUFFER_TEMPLATE_STR_NAME)
	//---------------------------------------------------------------------------------------------------
	#include <nrf.h>	//for interrupt enable & disable in synchronized methods
	//---------------------------------------------------------------------------------------------------
	void
	CBUFFER_TEMPLATE_FUN_NAME(clear)			(CBUFFER_TEMPLATE_STR_NAME *cb) {
		cb->count = 0;
		cb->pin = 0;
		#ifdef CBUFFER_TEMPLATE_SUM_TYPE
			cb->fastSum = 0;
		#endif //CBUFFER_TEMPLATE_SUM_TYPE
		CBUFFER_TEMPLATE_TYPE *element = cb->buf;
		for (uint16_t u = 0; u < cb->len; u++) {
			*element = CBUFFER_DEFAULT_RET;
			element++;
		}
	}

	synchronized void
	CBUFFER_TEMPLATE_FUN_NAME(sclear)			(CBUFFER_TEMPLATE_STR_NAME *cb) {
		__disable_irq();
		CBUFFER_TEMPLATE_FUN_NAME(clear)(cb);
		__enable_irq();
	}
	//---------------------------------------------------------------------------------------------------
	void
	CBUFFER_TEMPLATE_FUN_NAME(clearex)			(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE val) {
		cb->count = 0;
		cb->pin = 0;
		#ifdef CBUFFER_TEMPLATE_SUM_TYPE
			cb->fastSum = (cb->len)*val;
		#endif //CBUFFER_TEMPLATE_SUM_TYPE
		CBUFFER_TEMPLATE_TYPE *element = cb->buf;
		for (uint16_t u = 0; u < cb->len; u++) {
			*element = val;
			element++;
		}
	}

	synchronized void
	CBUFFER_TEMPLATE_FUN_NAME(sclearex)			(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE val) {
		__disable_irq();
		CBUFFER_TEMPLATE_FUN_NAME(clearex)(cb, val);
		__enable_irq();
	}
	//---------------------------------------------------------------------------------------------------
	void
	CBUFFER_TEMPLATE_FUN_NAME(resize)			(CBUFFER_TEMPLATE_STR_NAME *cb, uint16_t newSize) {
		if (newSize == 0)
			return;
		cb->len = newSize;
		CBUFFER_TEMPLATE_FUN_NAME(clear)(cb);
	}

	synchronized void
	CBUFFER_TEMPLATE_FUN_NAME(sresize)			(CBUFFER_TEMPLATE_STR_NAME *cb, uint16_t newSize) {
		__disable_irq();
		CBUFFER_TEMPLATE_FUN_NAME(resize)(cb, newSize);
		__enable_irq();
	}
	//---------------------------------------------------------------------------------------------------
	void
	CBUFFER_TEMPLATE_FUN_NAME(add)				(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE val) {
		#ifdef CBUFFER_TEMPLATE_SUM_TYPE
			cb->fastSum -= cb->buf[cb->pin];
			cb->fastSum += val;
		#endif //CBUFFER_TEMPLATE_SUM_TYPE
		cb->buf[cb->pin] = val;
		if (++(cb->pin) >= cb->len)
			cb->pin = 0;
		cb->count++;
	}

	synchronized void
	CBUFFER_TEMPLATE_FUN_NAME(sadd)				(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE val) {
		__disable_irq();
		CBUFFER_TEMPLATE_FUN_NAME(add)(cb, val);
		__enable_irq();
	}
	//---------------------------------------------------------------------------------------------------
	CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(get)				(CBUFFER_TEMPLATE_STR_NAME *cb, uint16_t pos) {
		if (pos >= cb->len)
			return CBUFFER_DEFAULT_RET;

		return cb->buf[pos];
	}

	synchronized CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(sget)				(CBUFFER_TEMPLATE_STR_NAME *cb, uint16_t pos)  {
		CBUFFER_TEMPLATE_TYPE tmp;
		__disable_irq();
		tmp = CBUFFER_TEMPLATE_FUN_NAME(get)(cb, pos);
		__enable_irq();
		return tmp;
	}
	//---------------------------------------------------------------------------------------------------
	CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(getNewestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb) {
		if (cb->count < 1)
			return CBUFFER_DEFAULT_RET;
		else if ((cb->pin) > 0)
			return cb->buf[(cb->pin)-1];
		else
			return cb->buf[(cb->len)-1];
	}

	synchronized CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(sgetNewestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb)  {
		CBUFFER_TEMPLATE_TYPE tmp;
		__disable_irq();
		tmp = CBUFFER_TEMPLATE_FUN_NAME(getNewestVal)(cb);
		__enable_irq();
		return tmp;
	}
	//---------------------------------------------------------------------------------------------------
	CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(getOldestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb) {
		if (cb->count < 1) {
			return CBUFFER_DEFAULT_RET;
		} else if (cb->count < cb->len) {
			if (cb->pin >= cb->count) {
				return cb->buf[(cb->pin) - (cb->count)];
			} else {
				return cb->buf[(cb->len) - ((cb->count) - (cb->pin))];
			}
		} else {
			return cb->buf[cb->pin];
		}
	}

	synchronized CBUFFER_TEMPLATE_TYPE
	CBUFFER_TEMPLATE_FUN_NAME(sgetOldestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb)  {
		CBUFFER_TEMPLATE_TYPE tmp;
		__disable_irq();
		tmp = CBUFFER_TEMPLATE_FUN_NAME(getOldestVal)(cb);
		__enable_irq();
		return tmp;
	}
	//---------------------------------------------------------------------------------------------------
	void

CBUFFER_TEMPLATE_FUN_NAME(getNNewestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE* array, uint16_t n) {
		//CBUFFER_TEMPLATE_TYPE tmp[n];
		uint32_t tmp_count, m;
		int32_t i, j=0, k=0;

		if(cb->count > cb->len)
			tmp_count = cb->len;
		else
			tmp_count = cb->count;

		if(n > tmp_count) {
			for(i = (int32_t)n-1; i >= (int32_t)tmp_count; i--)
				array[i] = CBUFFER_DEFAULT_RET;
			m = tmp_count;
		} else {
			m = n;
		}

		//if (m > 0) {
			j = cb->pin;
			k = cb->len;
			for(i = (int64_t)m-1; i>= 0; i--) {
				if (j > 0) {
					array[i] = cb->buf[j-1];
					j--;
				} else {
					array[i] = cb->buf[k-1];
					k--;
				}
			}
		//}
	}

	synchronized void//CBUFFER_TEMPLATE_TYPE*
	CBUFFER_TEMPLATE_FUN_NAME(sgetNNewestVal)				(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_TYPE* array, uint16_t n)  {
		//CBUFFER_TEMPLATE_TYPE* tmp;
		__disable_irq();
		CBUFFER_TEMPLATE_FUN_NAME(getNNewestVal)(cb, array, n);
		__enable_irq();
		//return tmp;
	}
	//---------------------------------------------------------------------------------------------------
	uint32_t
	CBUFFER_TEMPLATE_FUN_NAME(getCounter)		(CBUFFER_TEMPLATE_STR_NAME *cb) {
		return cb->count;
	}

	synchronized uint32_t
	CBUFFER_TEMPLATE_FUN_NAME(sgetCounter)		(CBUFFER_TEMPLATE_STR_NAME *cb) {
		uint32_t tmp;

		__disable_irq();
		tmp = CBUFFER_TEMPLATE_FUN_NAME(getCounter)(cb);
		__enable_irq();
		return tmp;
	}
	//---------------------------------------------------------------------------------------------------
	#ifdef CBUFFER_TEMPLATE_SUM_TYPE
		//---------------------------------------------------------------------------------------------------
		void
		CBUFFER_TEMPLATE_FUN_NAME(resetFastSum)		(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_SUM_TYPE fSum) {
			cb->fastSum = fSum;
		}

		synchronized void
		CBUFFER_TEMPLATE_FUN_NAME(sresetFastSum)	(CBUFFER_TEMPLATE_STR_NAME *cb, CBUFFER_TEMPLATE_SUM_TYPE fSum) {
			__disable_irq();
			CBUFFER_TEMPLATE_FUN_NAME(resetFastSum)(cb, fSum);
			__enable_irq();
		}
		//---------------------------------------------------------------------------------------------------
		CBUFFER_TEMPLATE_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(recalcFastSum)	(CBUFFER_TEMPLATE_STR_NAME *cb) {
			CBUFFER_TEMPLATE_SUM_TYPE sum = 0;
			CBUFFER_TEMPLATE_TYPE *element =cb->buf;
			for (uint16_t u = 0; u < cb->len; u++) {
				sum += *element;
				element++;
			}
			cb->fastSum = sum;
			return sum;
		}

		synchronized CBUFFER_TEMPLATE_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(srecalcFastSum)	(CBUFFER_TEMPLATE_STR_NAME *cb) {
			CBUFFER_TEMPLATE_SUM_TYPE tmp;
			__disable_irq();
			tmp = CBUFFER_TEMPLATE_FUN_NAME(recalcFastSum)(cb);
			__enable_irq();
			return tmp;
		}
		//---------------------------------------------------------------------------------------------------
		CBUFFER_TEMPLATE_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(getFastSum)		(CBUFFER_TEMPLATE_STR_NAME *cb) {
			return cb->fastSum;
		}

		synchronized CBUFFER_TEMPLATE_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(sgetFastSum)		(CBUFFER_TEMPLATE_STR_NAME *cb) {
			CBUFFER_TEMPLATE_SUM_TYPE tmp;
			__disable_irq();
			tmp = CBUFFER_TEMPLATE_FUN_NAME(getFastSum)(cb);
			__enable_irq();
			return tmp;
		}
		//---------------------------------------------------------------------------------------------------
		CBUFFER_TEMPLATE_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(getRawMean)		(CBUFFER_TEMPLATE_STR_NAME *cb) {
			return (CBUFFER_TEMPLATE_TYPE)(cb->fastSum / (CBUFFER_TEMPLATE_SUM_TYPE)cb->len);
		}

		synchronized CBUFFER_TEMPLATE_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(sgetRawMean)		(CBUFFER_TEMPLATE_STR_NAME *cb)  {
			CBUFFER_TEMPLATE_TYPE tmp;
			__disable_irq();
			tmp = CBUFFER_TEMPLATE_FUN_NAME(getRawMean)(cb);
			__enable_irq();
			return tmp;
		}
		//---------------------------------------------------------------------------------------------------
		CBUFFER_TEMPLATE_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(getRealMean)		(CBUFFER_TEMPLATE_STR_NAME *cb) {
			if (cb->count == 0)
				return CBUFFER_DEFAULT_RET;
			uint32_t n = (cb->count < cb->len)?(cb->count):(cb->len);
			return (CBUFFER_TEMPLATE_TYPE)(cb->fastSum / (CBUFFER_TEMPLATE_SUM_TYPE)n);
		}

		synchronized CBUFFER_TEMPLATE_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(sgetRealMean)		(CBUFFER_TEMPLATE_STR_NAME *cb) {
			CBUFFER_TEMPLATE_TYPE tmp;
			__disable_irq();
			tmp = CBUFFER_TEMPLATE_FUN_NAME(getRealMean)(cb);
			__enable_irq();
			return tmp;
		}
		//---------------------------------------------------------------------------------------------------
	#endif //CBUFFER_TEMPLATE_SUM_TYPE
	//---------------------------------------------------------------------------------------------------
	#ifdef CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE
		//---------------------------------------------------------------------------------------------------
		CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(weightedSumFromNewest)		(CBUFFER_TEMPLATE_STR_NAME *cb, const CBUFFER_TEMPLATE_TYPE *weights) {
			CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE sum = 0;
			uint16_t p;// = (cb->pin == 0)?(cb->len-1):(cb->pin-1);
			if (cb->pin == 0)
				p = cb->len-1;
			else
				p = cb->pin-1;

			CBUFFER_TEMPLATE_TYPE *elem = cb->buf;
			for (uint16_t u = 0; u < cb->len; u++) {
				sum += (elem[p])*(weights[u]);
					if (p == 0)
						p = cb->len-1;
					else
						p--;
			}
//			for (uint16_t u = p; u >= 0; u--) {
//				sum += (elem[u])*(weights[wp++]);
//			}
//			for (uint16_t u = cb->len-1; u > p; u--) {
//				sum += (elem[u])*(weights[wp++]);
//			}
			return sum;
		}

		synchronized CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE
		CBUFFER_TEMPLATE_FUN_NAME(sweightedSumFromNewest)		(CBUFFER_TEMPLATE_STR_NAME *cb, const CBUFFER_TEMPLATE_TYPE *weights) {
			CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE tmp;
			__disable_irq();
			tmp = CBUFFER_TEMPLATE_FUN_NAME(weightedSumFromNewest)(cb, weights);
			__enable_irq();
			return tmp;
		}
		//---------------------------------------------------------------------------------------------------
	#endif //CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE
	//---------------------------------------------------------------------------------------------------
#endif //defined(CBUFFER_TEMPLATE_DEF) && defined(CBUFFER_TEMPLATE_FUN_PREFIX) && defined(CBUFFER_TEMPLATE_TYPE) && defined(CBUFFER_TEMPLATE_STR_NAME)
//---------------------------------------------------------------------------------------------------
//#############################################################################################################################################################
//---------------------------------------------------------------------------------------------------
//Clear all definitions
//---------------------------------------------------------------------------------------------------
#undef 	CBUFFER_TEMPLATE_DEC
#undef 	CBUFFER_TEMPLATE_FUN_PREFIX
#undef 	CBUFFER_TEMPLATE_TYPE
#undef 	CBUFFER_TEMPLATE_STR_NAME
#undef	CBUFFER_DEFAULT_RET
#undef 	CBUFFER_TEMPLATE_SUM_TYPE
#undef 	CBUFFER_TEMPLATE_WEIGHTED_SUM_TYPE
//---------------------------------------------------------------------------------------------------
