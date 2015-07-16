/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file Application_Interface.h
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __SMAC_APPLICATION_CONF_H__
#define __SMAC_APPLICATION_CONF_H__

#include "MemManager.h"
#include "ConnectivityMenus.h"      /*Defines the Application menus*/

#include "SMAC_Interface.h"         /*@CMA Conn Test*/
#include "FunctionLib.h"
#include "AspInterface.h"
#ifdef gPHY_802_15_4g_d
#include "PhyTime.h"
#endif

#include "mbed.h"
#include "rtos.h"



/* BEGIN Configuration Parameters Definition */ 
typedef enum ConnectivityStates_tag
{
  gConnInitState_c,
  gConnIdleState_c,
  gConnSelectTest_c,
  gConnContinuousTxRxState_c,
  gConnPerState_c,
  gConnRangeState_c,
  gConnRegEditState_c,
  gConnSetChannelState_c,
  gConnSetPowerState_c,
  gConnSetTxTestState_c,
  gConnSetRxTestState_c,
  gConnSetTrimCoarseState_c,
  gConnSetTrimFineState_c,
  gConnContinuousTxState_c,
  gConnPerTxState_c,
  gConnRangeTxState_c,
  gConnContinuousRxState_c,
  gConnPerRxState_c,
  gConnRangeRxState_c,
  gConnBitrateSelectState_c,
  gConnCSenseAndTCtrl_c,
  gConnEDMeasCalib_c,
  gConnMaxState_c
}ConnectivityStates_t;


typedef enum ContinuousTxRxTestStates_tag 
{
  gCTxRxStateInit_c,
  gCTxRxStateIdle_c,
  gCTxRxStateSelectTest_c,
  gCTxRxStateRunnigEdTest_c,
  gCTxRxStateRunningEdTestGotResult_c,
  gCTxRxStateRunningPRBS9Test_c,
  gCTxRxStateRunnigScanTest_c,
  gCTxRxStateRunnigRxTest_c,
  gCTxRxStateRunningTXModSelectOpt,  
  gCTxRxStateRunnigCcaTest_c,
  gCTxRxMaxState_c
}ContinuousTxRxTestStates_t;

typedef enum PerTxStates_tag 
{
  gPerTxStateInit_c = 0,
  gPerTxStateIdle_c,
  gPerTxStateSelectPacketNum_c,
  gPerTxStateInputPacketDelay_c,
  gPerTxStateStartTest_c,
  gPerTxStateRunningTest_c,
  gPerTxStateSendingLastFrames_c,
  gPerTxStateMaxState_c
}PerTxStates_t;

typedef enum PerRxStates_tag 
{
  gPerRxStateInit_c = 0,
  gPerRxStateIdle_c,
  gPerRxWaitStartTest_c,
  gPerRxStateStartTest_c,
  gPerRxStateRunningTest_c,
  gPerRxStateReceivingLastFrames_c,
  gPerrxStateMaxState_c
}PerRxStates_t;

typedef enum RangeTxStates_tag 
{
  gRangeTxStateInit_c = 0,
  gRangeTxStateIdle_c,
  gRangeTxWaitStartTest_c,
  gRangeTxStateStartTest_c,
  gRangeTxStateRunningTest_c,
  gRangeTxStatePrintTestResults_c,
  gRangeTxStateMaxState_c
}RangeTxStates_t;

typedef enum RangeRxStates_tag 
{
  gRangeRxStateInit_c = 0,
  gRangeRxStateIdle_c,
  gRangeRxWaitStartTest_c,
  gRangeRxStateStartTest_c,
  gRangeRxStateRunningTest_c,
  gRangeRxStatePrintTestResults_c,
  gRangeRxStateMaxState_c
}RangeRxStates_t;

typedef enum CSenseTCtrlStates_tag
{
  gCsTcStateInit_c = 0,
  gCsTcStateSelectTest_c,
  gCsTcStateCarrierSenseStart_c,
  gCsTcStateCarrierSenseSelectType_c,
  gCsTcStateCarrierSensePerformingTest_c,
  gCsTcStateCarrierSenseEndTest_c,
  gCsTcStateTransmissionControlStart_c,
  gCsTcStateTransmissionControlSelectNumOfPackets_c,
  gCsTcStateTransmissionControlSelectInterpacketDelay_c,
  gCsTcStateTransmissionControlPerformingTest_c,
  gCsTcStateTransmissionControlEndTest_c,
  gCsTcStateMaxState_c
}CSenseTCtrlStates_t;

typedef enum overrideRegistersStates_tag 
{
  gORStateInit_c = 0,
  gORStateStart_c,
  gORWaitForTheAddress_c,
  gORWaitForTheValue_c,  
  gORStateIdle_c,
  gORStateMaxState_c
}oRStates_t;

typedef enum readRegistersStates_tag
{
  gRRStateInit_c = 0,
  gRRStateStart_c,
  gRRWaitForTheAddress_c,
  gRRStateMaxState_c
}rRStates_t;

typedef enum EditRegsStates_tag 
{
  gERStateInit_c = 0,
  gERWaitSelection_c,
  gERStateOverrideReg_c,
  gERStateReadReg_c,
  gERStateDumpAllRegs_c,
  gERStateOverrideRadioSpiReg_c,
  gERStateReadRadioSpiReg_c,
  gERStateMaxState_c
}EditRegsStates_t;

typedef enum dumpRegistersStates_t
{
  gDRStateInit_c = 0,
  gDRStateDumpRegs_c
}dRStates_t;
/* END Configuration Parameters Definition */

#define gDefaultOperationMode_c         mTxOperation_c
#define gDefaultPayload_c               20
#define mTotalFinalFrames_c             25

#ifdef gPHY_802_15_4g_d
 #define gMaxChannel_c                   ((channels_t)(gTotalChannels - 1))
 #define gMinChannel_c                   (gChannel0_c)
#else
 #define gMaxChannel_c                   ((channels_t)gTotalChannels)
 #define gMinChannel_c                   (gChannel11_c)
#endif

extern smacErrors_t smacToAppMlmeSap(smacToAppMlmeMessage_t* pMsg, instanceId_t instance);
extern smacErrors_t smacToAppMcpsSap(smacToAppDataMessage_t* pMsg, instanceId_t instance);
extern void InitApp();

#endif /* __SMAC_APP_CONFIG_H__ */
