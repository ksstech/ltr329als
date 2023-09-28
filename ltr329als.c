/*
 * ltr329als.c
 * Copyright (c) 2022 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_config.h"

#if (halHAS_LTR329ALS > 0)
#include "hal_i2c_common.h"
#include "endpoints.h"
#include "ltr329als.h"
#include "printfx.h"
#include "rules.h"
#include "syslog.h"
#include "systiming.h"
#include "x_errors_events.h"

#define	debugFLAG					0xF000

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ############################################# Macros ############################################


// #################################### SI7006/13/20/21 Addresses ##################################

#define	ltr329alsADDR0				0x60

// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################

const u8_t ltr329Gain[8] = { 1, 2, 4, 8, 0, 0, 48, 96 };
const f32_t ltr329IntgTime[8] = { 1, 0.5, 2, 4, 1.5, 2.5, 3, 3.5 };
const u16_t ltr329MeasRate[8] = { 50, 100, 200, 500, 1000, 2000, 2000, 2000 };

// ###################################### Local variables ##########################################

ltr329als_t sLTR329ALS = { 0 };

// #################################### Local ONLY functions #######################################

int ltr329alsReadReg(u8_t Reg, u8_t * pRxBuf) {
	IF_SYSTIMER_START(debugTIMING, stLTR329ALS);
	int iRV = halI2C_Queue(sLTR329ALS.psI2C, i2cWR_B, &Reg, sizeof(Reg), pRxBuf, sizeof(uint8_t), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stLTR329ALS);
	return iRV;
}

int ltr329alsWriteReg(u8_t reg, u8_t val) {
	u8_t u8Buf[2] = { [0] = reg, [1] = val };
	IF_SYSTIMER_START(debugTIMING, stLTR329ALS);
	int iRV = halI2C_Queue(sLTR329ALS.psI2C, i2cW_B, u8Buf, sizeof(u8Buf), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stLTR329ALS);
	return iRV;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	ltr329alsIdentify(i2c_di_t * psI2C) {
	sLTR329ALS.psI2C = psI2C;
	ESP_ERROR_CHECK(esp_cpu_set_watchpoint(0, &sLTR329ALS.psI2C, sizeof(i2c_di_t *), ESP_CPU_WATCHPOINT_STORE));
	psI2C->Type = i2cDEV_LTR329ALS;
	psI2C->Speed = i2cSPEED_400;
	psI2C->TObus = 25;
	psI2C->Test = 1;
	int iRV = ltr329alsReadReg(ltr329alsMANUFAC_ID, &sLTR329ALS.Reg.MANUFAC_ID);
	if (iRV < erSUCCESS) goto exit;
	if (sLTR329ALS.Reg.MANUFAC_ID != 0x05) goto err_whoami;

	iRV = ltr329alsReadReg(ltr329alsPART_ID, &sLTR329ALS.Reg.PART_ID);
	if (iRV < erSUCCESS) goto exit;
	if (sLTR329ALS.Reg.part_id.part != 0xA) goto err_whoami;
	// all OK, configure HW done
	psI2C->IDok = 1;
	psI2C->Test = 0;
	goto exit;
err_whoami:
	iRV = erINV_WHOAMI;
exit:
	ESP_ERROR_CHECK(esp_cpu_set_watchpoint(1, sLTR329ALS.psI2C, sizeof(i2c_di_t), ESP_CPU_WATCHPOINT_STORE));
	return iRV;
}

int	ltr329alsConfig(i2c_di_t * psI2C) {
	if (!psI2C->IDok) return erINV_STATE;

	ESP_ERROR_CHECK(esp_cpu_clear_watchpoint(1));
	psI2C->CFGok = 0;
	int iRV = ltr329alsWriteReg(ltr329alsCONTR, sLTR329ALS.Reg.CONTROL = 0x01);
	if (iRV < erSUCCESS) goto exit;

	if (iRV > erFAILURE) ltr329alsReadReg(ltr329alsMEAS_RATE, &sLTR329ALS.Reg.MEAS_RATE);
	if (iRV < erSUCCESS) goto exit;

	if (iRV > erFAILURE) ltr329alsReadReg(ltr329alsSTATUS, &sLTR329ALS.Reg.STATUS);
	if (iRV < erSUCCESS) goto exit;

	// once off init....
	if (!psI2C->CFGerr) {
		IF_SYSTIMER_INIT(debugTIMING, stLTR329ALS, stMICROS, "LTR329", 500, 4000);
		#if (ltr329alsI2C_LOGIC == 3)
		sLTR329ALS.th = xTimerCreateStatic("ltr329als", pdMS_TO_TICKS(5), pdFALSE, NULL, ltr329alsTimerHdlr, &sLTR329ALS.ts);
		#endif
	}
	psI2C->CFGok = 1;
exit:
	ESP_ERROR_CHECK(esp_cpu_set_watchpoint(1, sLTR329ALS.psI2C, sizeof(i2c_di_t), ESP_CPU_WATCHPOINT_STORE));
	return iRV;
}

int	ltr329alsDiags(i2c_di_t * psI2C) { return erSUCCESS; }

// ######################################### Reporting #############################################

int ltr329alsReportAll(report_t * psR) {
	int iRV = halI2C_DeviceReport(psR, sLTR329ALS.psI2C);
	iRV += wprintfx(psR, "\tCONTROL: 0x%0X  gain=%d (%dx)  mode=%s\r\n", sLTR329ALS.Reg.CONTROL,
			sLTR329ALS.Reg.control.gain, ltr329Gain[sLTR329ALS.Reg.control.gain],
			sLTR329ALS.Reg.control.mode ? "Active" : "Standby");
	iRV += wprintfx(psR, "\tMEAS_RATE: 0x%0X  time=%d (%fmS)  rate=%d (%dmS)\r\n", sLTR329ALS.Reg.MEAS_RATE,
			sLTR329ALS.Reg.meas_rate.time, ltr329IntgTime[sLTR329ALS.Reg.meas_rate.time]*100.0,
			sLTR329ALS.Reg.meas_rate.rate, ltr329MeasRate[sLTR329ALS.Reg.meas_rate.rate]);
	iRV += wprintfx(psR, "\tMANUFAC_ID: 0x%0X  PART=%d  REV=%d\r\n", sLTR329ALS.Reg.MANUFAC_ID,
			sLTR329ALS.Reg.part_id.part, sLTR329ALS.Reg.part_id.rev);
	iRV += wprintfx(psR, "\tSTATUS: 0x%0X  valid=%d  gain=%d  intr=%d  data=%d\r\n", sLTR329ALS.Reg.STATUS,
			sLTR329ALS.Reg.status.valid, sLTR329ALS.Reg.status.gain,
			sLTR329ALS.Reg.status.intr, sLTR329ALS.Reg.status.data);
	#if (ltr329alsI2C_LOGIC == 3)
	iRV += xRtosReportTimer(psR, sLTR329ALS.th);
	#endif
	return iRV;
}

#endif
