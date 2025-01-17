/*
 * ffz.c
 *
 *  Created on: Nov 6, 2024
 *      Author: Testrechner
 */

#include "ffz.h"

/**
 *
 */
void FFZ_Init() {
//	FFZ_Objects FFZ;

	//TX
	FFZ.Driving_Active = NOT_ACTIVE;
	FFZ.Lift_Hydraulic_Active = NOT_ACTIVE;
	FFZ.DriverPresent = NOT_ACTIVE;
	FFZ.TruckSpeed = 0;
	FFZ.OperationTime = EBZ_Init();
	FFZ.F1A_CreepSpeed = AVAILABLE;
	FFZ.F2A_LiftingLimitiation = AVAILABLE;
	FFZ.F3A_TruckSpeedSignal = AVAILABLE;
	FFZ.MainVersion = 0x01;
	FFZ.Subversion = 0x01;

	// RX
	FFZ.ACRQ_AcceesReq = INHIBT;
	FFZ.SRRQ_SpeedReductionReq = INHIBT;
	FFZ.LRRQ_LiftReductionReq = INHIBT;

}
