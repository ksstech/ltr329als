/*
 * Copyright 2022 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#include <stdint.h>

//#include "endpoints.h"
#include "hal_i2c.h"

#ifdef __cplusplus
	extern "C" {
#endif

// ########################################### Macros ##############################################

#define	ltr329alsI2C_LOGIC			1					// 0 = delay, 1= stretch, 2= stages

// ######################################## Enumerations ###########################################

enum {
	ltr329alsCONTR		= 0x80,
	ltr329alsMEAS_RATE	= 0x85,
	ltr329alsPART_ID	= 0x86,
	ltr329alsMANUFAC_ID	= 0x87,
	ltr329alsDATA_CH1_0	= 0x88,
	ltr329alsDATA_CH1_1	= 0x89,
	ltr329alsDATA_CH0_0	= 0x8A,
	ltr329alsDATA_CH0_1	= 0x8B,
	ltr329alsSTATUS		= 0x8C,
};

enum {								// Status register bits
	ltr329alsR_STATUS_TDR = 0x02,
	ltr329alsR_STATUS_PDR = 0x04,
	ltr329alsR_STATUS_PTDR = 0x08,
};

// ######################################### Structures ############################################

typedef struct __attribute__((packed)) {
	uint8_t mode : 1;				// 0=Standby  1=Active
	uint8_t	reset : 1;
	uint8_t	gain : 3;
	uint8_t	res : 3;
} ltr329als_control_t;

typedef struct __attribute__((packed)) {
	uint8_t rate : 3;				// LSB
	uint8_t	time : 3;
	uint8_t	res : 2;
} ltr329als_meas_rate_t;

typedef struct __attribute__((packed)) {
	uint8_t rev : 4;				// LSB
	uint8_t	part : 4;
} ltr329als_part_id_t;

typedef struct __attribute__((packed)) {
	uint8_t res : 2;				// LSB
	uint8_t	data : 1;
	uint8_t intr : 1;				// don't care value ???
	uint8_t gain : 3;
	uint8_t valid :1;
} ltr329als_status_t;

typedef struct __attribute__((packed)) {
	union {
		ltr329als_control_t control;
		uint8_t CONTROL;
	};
	union {
		ltr329als_meas_rate_t meas_rate;
		uint8_t MEAS_RATE;
	};
	union {
		ltr329als_part_id_t part_id;
		uint8_t PART_ID;
	};
	uint8_t MANUFAC_ID;
	union {
		union {							// Channel 1 -
			uint16_t CH1;
			uint8_t ch1[2];
		};
		union {							// Channel 0 -
			uint16_t CH0;
			uint8_t ch0[2];
		};
		uint8_t ch[4];
	};
	union {
		ltr329als_status_t status;
		uint8_t STATUS;
	};
} ltr329als_reg_t;
DUMB_STATIC_ASSERT(sizeof(ltr329als_reg_t) == 9);

typedef struct __attribute__((packed)) {				// SI70006/13/14/20/xx TMP & RH sensors
	i2c_di_t *		psI2C;			// 4 bytes
	SemaphoreHandle_t mux;
	union {
		ltr329als_reg_t Reg;
		uint8_t u8Buf[sizeof(ltr329als_reg_t)];
	};
	uint8_t	spare[3];
	#if (ltr329alsI2C_LOGIC == 3)		// 3 step read -> wait -> convert
	TimerHandle_t th;
	StaticTimer_t ts;
	#endif
} ltr329als_t;
DUMB_STATIC_ASSERT(sizeof(ltr329als_t) == 20);

// ###################################### Public variables #########################################


// ###################################### Public functions #########################################

int	ltr329alsIdentify(i2c_di_t * psI2C_DI);
int	ltr329alsConfig(i2c_di_t * psI2C_DI);
int	ltr329alsReConfig(i2c_di_t * psI2C_DI);
int	ltr329alsDiags(i2c_di_t * psI2C_DI);
void ltr329alsReportAll(void) ;

struct rule_t;
int	ltr329alsConfigMode (struct rule_t *, int Xcur, int Xmax);

struct epw_t;
int	ltr329alsReadHdlr(epw_t * psEWP);

#ifdef __cplusplus
	}
#endif
