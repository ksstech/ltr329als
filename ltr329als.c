/*
 * ltr329als.c
 * Copyright (c) 2022 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "ltr329als.h"
#include "hal_variables.h"
#include "endpoints.h"
#include "options.h"
#include "printfx.h"
#include "syslog.h"
#include "systiming.h"
#include "x_errors_events.h"

#define	debugFLAG					0xF000

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ############################################# Macros ############################################

#define	ltr329alsI2C_LOGIC			1					// 0 = delay, 1= stretch, 2= stages

// #################################### SI7006/13/20/21 Addresses ##################################

#define	ltr329alsADDR0				0x60
#define	LTR329ALS_T_SNS				1000

// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################

const uint8_t ltr329Gain[8] = { 1, 2, 4, 8, 0, 0, 48, 96 };
const float ltr329IntgTime[8] = { 1, 0.5, 2, 4, 1.5, 2.5, 3, 3.5 };
const uint16_t ltr329MeasRate[8] = { 50, 100, 200, 500, 1000, 2000, 2000, 2000 };

// ###################################### Local variables ##########################################

ltr329als_t sLTR329ALS = { 0 };

// #################################### Local ONLY functions #######################################

int ltr329alsReadReg(uint8_t Reg, uint8_t * pRxBuf) {
	return halI2C_Queue(sLTR329ALS.psI2C, i2cWR_B, &Reg, sizeof(Reg),
			pRxBuf, sizeof(uint8_t), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

int ltr329alsWriteReg(uint8_t reg, uint8_t val) {
	uint8_t u8Buf[2] = { reg, val };
	return halI2C_Queue(sLTR329ALS.psI2C, i2cW_B, u8Buf, sizeof(u8Buf), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

#if (ltr329alsI2C_LOGIC == 1)		// read and convert in 1 go...
int	ltr329alsReadHdlr(epw_t * psEWP) {
	IF_SYSTIMER_START(debugTIMING, stLTR329ALS);
	for (uint8_t Reg = 0; Reg < 4; ++Reg) {
		ltr329alsReadReg(Reg+ltr329alsDATA_CH1_0, (uint8_t *) &sLTR329ALS.Reg.ch[Reg]);
	}
	IF_SYSTIMER_STOP(debugTIMING, stLTR329ALS);
//	P("ltr329als  [ %-`B ]\r\n", 4, sLTR329ALS.Reg.ch);
	x64_t X64;
	// Convert & update pressure/altitude sensor
	uint16_t data_ch0, data_ch1;
	data_ch0 = (sLTR329ALS.Reg.ch0[1] << 8) | sLTR329ALS.Reg.ch0[0];
	data_ch1 = (sLTR329ALS.Reg.ch1[1] << 8) | sLTR329ALS.Reg.ch1[0];
	if (data_ch0 + data_ch1 > 0) {
		float ratio = data_ch1 / (data_ch0 + data_ch1);
		X64.x32[0].f32 = (ratio < 0.45) ? (1.7743 * data_ch0 + 1.1059 * data_ch1)
		    : (ratio >= 0.45 && ratio < 0.64) ? (4.2785 * data_ch0 - 1.9548 * data_ch1)
		    : (ratio >= 0.64 && ratio < 0.85) ? (0.5926 * data_ch0 + 0.1185 * data_ch1) : 0;
		X64.x32[0].f32 /= (ltr329Gain[sLTR329ALS.Reg.control.gain] * ltr329IntgTime[sLTR329ALS.Reg.meas_rate.time]);
	} else {
		X64.x32[0].f32 = 0;
	}
	vCV_SetValue(&table_work[URI_LTR329ALS].var, X64);
	return erSUCCESS;
}
#elif (ltr329alsI2C_LOGIC == 2)		// clock stretching

	#error "Not supported"

#elif (ltr329alsI2C_LOGIC == 3)		// 3 step read -> wait -> convert

	#error "Not supported"

#endif

// ################################ Rules configuration support ####################################

int	ltr329alsConfigMode (struct rule_t * psR, int Xcur, int Xmax) {
	// mode /ltr329als idx gain time rate
	uint8_t	AI = psR->ActIdx;
	int gain = psR->para.x32[AI][0].i32;
	int time = psR->para.x32[AI][1].i32;
	int rate = psR->para.x32[AI][2].i32;
	IF_P(debugTRACK && ioB1GET(ioMode), "mode 'LTR329ALS' Xcur=%d Xmax=%d gain=%d time=%d rate=%d\r\n", Xcur, Xmax, gain, time, rate);

	if (OUTSIDE(0, gain, 7, int) ||
		OUTSIDE(0, time, 7, int) ||
		OUTSIDE(0, rate, 7, int) ||
		gain==4 || gain==5) {
		RETURN_MX("Invalid gain / time / rate specified", erINVALID_PARA);
	}
	int iRV = ltr329alsWriteReg(ltr329alsCONTR, sLTR329ALS.Reg.control.gain = gain);
	IF_RETURN_X(iRV != erSUCCESS, iRV);
	sLTR329ALS.Reg.meas_rate.time = time;
	sLTR329ALS.Reg.meas_rate.rate = rate;
	return ltr329alsWriteReg(ltr329alsMEAS_RATE, sLTR329ALS.Reg.MEAS_RATE);
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	ltr329alsIdentify(i2c_di_t * psI2C_DI) {
	psI2C_DI->TRXmS	= 50;
	psI2C_DI->CLKuS = 400;
	psI2C_DI->Test = 1;
	sLTR329ALS.psI2C = psI2C_DI;
	int iRV = ltr329alsReadReg(ltr329alsMANUFAC_ID, &sLTR329ALS.Reg.MANUFAC_ID);
	IF_EXIT(iRV != erSUCCESS);
	IF_GOTO(sLTR329ALS.Reg.MANUFAC_ID != 0x05, exit_err);

	iRV = ltr329alsReadReg(ltr329alsPART_ID, &sLTR329ALS.Reg.PART_ID);
	IF_EXIT(iRV != erSUCCESS);
	IF_GOTO(sLTR329ALS.Reg.part_id.part != 0xA, exit_err);

	psI2C_DI->Type		= i2cDEV_LTR329ALS;
	psI2C_DI->Speed		= i2cSPEED_400;
	psI2C_DI->DevIdx 	= 0;
	goto exit;
exit_err:
	iRV = erFAILURE;
exit:
	psI2C_DI->Test = 0;
	return iRV ;
}

int	ltr329alsConfig(i2c_di_t * psI2C_DI) {
	ltr329alsWriteReg(ltr329alsCONTR, sLTR329ALS.Reg.CONTROL = 0x01);
	ltr329alsReadReg(ltr329alsMEAS_RATE, &sLTR329ALS.Reg.MEAS_RATE);
	ltr329alsReadReg(ltr329alsSTATUS, &sLTR329ALS.Reg.STATUS);

	epw_t * psEWP = &table_work[URI_LTR329ALS];
	psEWP->var.def.cv.vc = 1;
	psEWP->var.def.cv.vs = vs32B;
	psEWP->var.def.cv.vf = vfFXX;
	psEWP->var.def.cv.vt = vtVALUE;
	psEWP->Tsns = psEWP->Rsns = LTR329ALS_T_SNS;
	psEWP->uri = URI_LTR329ALS;

#if (ltr329alsI2C_LOGIC == 3)
	sLTR329ALS.timer = xTimerCreate("ltr329als", pdMS_TO_TICKS(5), pdFALSE, NULL, ltr329alsTimerHdlr);
#endif
	IF_SYSTIMER_INIT(debugTIMING, stLTR329ALS, stMICROS, "LTR329", 500, 4000);
	return erSUCCESS ;
}

int ltr329alsReConfig(i2c_di_t * psI2C_DI) { return erSUCCESS; }

int	ltr329alsDiags(i2c_di_t * psI2C_DI) { return erSUCCESS; }

// ######################################### Reporting #############################################

void ltr329alsReportAll(void) {
	halI2C_DeviceReport(sLTR329ALS.psI2C);
	P("\tCONTROL: 0x%0X  gain=%d (%dx)  mode=%s\r\n", sLTR329ALS.Reg.CONTROL,
			sLTR329ALS.Reg.control.gain, ltr329Gain[sLTR329ALS.Reg.control.gain],
			sLTR329ALS.Reg.control.mode ? "Active" : "Standby");
	P("\tMEAS_RATE: 0x%0X  time=%d (%fmS)  rate=%d (%dmS)\r\n", sLTR329ALS.Reg.MEAS_RATE,
			sLTR329ALS.Reg.meas_rate.time, ltr329IntgTime[sLTR329ALS.Reg.meas_rate.time]*100.0,
			sLTR329ALS.Reg.meas_rate.rate, ltr329MeasRate[sLTR329ALS.Reg.meas_rate.rate]);
	P("\tMANUFAC_ID: 0x%0X  PART=%d  REV=%d\r\n", sLTR329ALS.Reg.MANUFAC_ID,
			sLTR329ALS.Reg.part_id.part, sLTR329ALS.Reg.part_id.rev);
	P("\tSTATUS: 0x%0X  valid=%d  gain=%d  intr=%d  data=%d\r\n", sLTR329ALS.Reg.STATUS,
			sLTR329ALS.Reg.status.valid, sLTR329ALS.Reg.status.gain,
			sLTR329ALS.Reg.status.intr, sLTR329ALS.Reg.status.data);
}
