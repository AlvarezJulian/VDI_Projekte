/*
 * ffz.h
 *
 *  Created on: Nov 6, 2024
 *      Author: Testrechner
 */


#include "main.h"
#include <stdio.h>
#include <stdint.h>

#ifndef INC_FFZ_H_
#define INC_FFZ_H_

typedef enum  {
	NOT_ACTIVE = 0x0,
	ACTIVE = 0x1,
	NOT_USED = 0x2,
	SERVICE_NOT_AVAILABLE =0x3
} Txobjects;

typedef enum  {
	INHIBT = 0x0,
	RELEASE,
} Rxobjects;

typedef enum {
	NOT_AVAILABLE = 0x0,
	AVAILABLE = 0x1,
}FeatureObtions;


typedef struct {
	//TX Objects
	Txobjects Driving_Active;
	Txobjects Lift_Hydraulic_Active;
	Txobjects DriverPresent;
	int16_t TruckSpeed;
	int32_t OperationTime;
//	uint64_t OperationTime;
	FeatureObtions F1A_CreepSpeed;
	FeatureObtions F2A_LiftingLimitiation;
	FeatureObtions F3A_TruckSpeedSignal;
	uint8_t MainVersion;
	uint8_t Subversion;

	//RX Objects
	Rxobjects ACRQ_AcceesReq;
	Rxobjects SRRQ_SpeedReductionReq;
	Rxobjects LRRQ_LiftReductionReq;

} FFZ_Objects;


extern void FFZ_Init();


#endif /* INC_FFZ_H_ */
