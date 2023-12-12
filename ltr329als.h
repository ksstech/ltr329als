/*
 * ltr329als.h - Copyright (c) 2022-23 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

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
	u8_t mode:1;				// 0=Standby  1=Active
	u8_t reset:1;
	u8_t gain:3;
	u8_t res:3;
} ltr329als_control_t;

typedef struct __attribute__((packed)) {
	u8_t rate:3;				// LSB
	u8_t time:3;
	u8_t res:2;
} ltr329als_meas_rate_t;

typedef struct __attribute__((packed)) {
	u8_t rev:4;				// LSB
	u8_t part:4;
} ltr329als_part_id_t;

typedef struct __attribute__((packed)) {
	u8_t res:2;				// LSB
	u8_t data:1;
	u8_t intr:1;				// don't care value ???
	u8_t gain:3;
	u8_t valid:1;
} ltr329als_status_t;

typedef struct __attribute__((packed)) {
	union {
		ltr329als_control_t control;
		u8_t CONTROL;
	};
	union {
		ltr329als_meas_rate_t meas_rate;
		u8_t MEAS_RATE;
	};
	union {
		ltr329als_part_id_t part_id;
		u8_t PART_ID;
	};
	u8_t MANUFAC_ID;
	union {
		union {							// Channel 1 -
			u16_t CH1;
			u8_t ch1[2];
		};
		union {							// Channel 0 -
			u16_t CH0;
			u8_t ch0[2];
		};
		u8_t ch[4];
	};
	union {
		ltr329als_status_t status;
		u8_t STATUS;
	};
} ltr329als_reg_t;
DUMB_STATIC_ASSERT(sizeof(ltr329als_reg_t) == 9);

struct i2c_di_t;
typedef struct __attribute__((packed)) ltr329als_t {	// SI70006/13/14/20/xx TMP & RH sensors
	struct i2c_di_t * psI2C;			// 4 bytes
	SemaphoreHandle_t mux;
	union {
		ltr329als_reg_t Reg;
		u8_t u8Buf[sizeof(ltr329als_reg_t)];
	};
	u8_t	spare[3];
	#if (ltr329alsI2C_LOGIC == 3)		// 3 step read -> wait -> convert
	TimerHandle_t th;
	StaticTimer_t ts;
	#endif
} ltr329als_t;
DUMB_STATIC_ASSERT(sizeof(ltr329als_t) == 20);

// ###################################### Public variables #########################################


// ###################################### Public functions #########################################

int ltr329alsReadReg(u8_t Reg, u8_t * pRxBuf);
int ltr329alsWriteReg(u8_t reg, u8_t val);

int	ltr329alsIdentify(struct i2c_di_t * psI2C);
int	ltr329alsConfig(struct i2c_di_t * psI2C);
int	ltr329alsDiags(struct i2c_di_t * psI2C);
struct report_t;
int ltr329alsReportAll(struct report_t * psR);

#ifdef __cplusplus
}
#endif
