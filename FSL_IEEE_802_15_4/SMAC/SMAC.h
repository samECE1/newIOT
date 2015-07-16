/**************************************************************************************************
* SMAC implementation.
* 
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2010 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
***************************************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
***********************************************************************************************//*!
**************************************************************************************************/

#ifndef SMAC_H_
#define SMAC_H_

/************************************************************************************
*************************************************************************************
* Includes
*************************************************************************************
************************************************************************************/

#include "SMAC_Interface.h"
#include "Phy.h"
#include "PhyInterface.h"
//#include "TimersManager.h"
#include "RNG_Interface.h"

#if defined (gPHY_802_15_4g_d)
#include "PhyPib.h"
#include "PhyExtended.h"
#include "PhyTime.h"
#endif
/************************************************************************************
*************************************************************************************
* Private Prototypes
*************************************************************************************
************************************************************************************/
void SmacSetRxTimeout(smacTime_t timeoutSymbols);

#if defined (gPHY_802_15_4g_d)

#define smacPreambleSizeOf16_c          (16)
#define smacPreambleSizeOf3_c           (3)

#endif
/*smacParametersValidation_d:
TRUE :  SMAC primitives validate their incoming parameters.
FALSE:  SMAC primitives do their stuff without validating their incoming 
parameters
*Note: Setting this as FALSE will compile a smaller foot print SMAC.
*/
#define smacParametersValidation_d          TRUE

/*smacInitializationValidation_d:
TRUE :  SMAC primitives validate whether SMAC is initialized or not.
FALSE:  SMAC primitives don't care about SMAC's initialization.
*Note: Setting this as FALSE will compile a smaller foot print SMAC.
*/
#define smacInitializationValidation_d      TRUE

#define FRAME_CTRL_ACK_FIELD_SET                (1 << 5)
/************************************************************************************
*************************************************************************************
* Module Type definitions
*************************************************************************************
************************************************************************************/
typedef enum smacStates_tag {
  mSmacStateIdle_c,
  mSmacStateTransmitting_c,
  mSmacStateReceiving_c,
  mSmacStateScanningChannels_c,
  mSmacStatePerformingCca_c,
  mSmacStatePerformingEd_c,
  mSmacStatePerformingTest_c,
  mSmacStateHibernate_c, 
  mSmacStateDoze_c    
} smacStates_t;

typedef union prssPacketPtr_tag
{
  uint8_t*    smacScanResultsPointer;     
  rxPacket_t  *smacRxPacketPointer;
  pdDataReq_t *smacTxPacketPointer;
}prssPacketPtr_t;

/***********************************************************************************
* Phy to SMAC SAP prototype
************************************************************************************/
typedef phyStatus_t ( * PD_SMAC_SapHandler_t)(pdDataToMacMessage_t * pMsg, instanceId_t instanceId);

typedef phyStatus_t ( * PLME_SMAC_SapHandler_t)(plmeToMacMessage_t * pMsg, instanceId_t instanceId);

/***********************************************************************************
* SMAC to App SAP handlers
************************************************************************************/
static SMAC_APP_MCPS_SapHandler_t gSMAC_APP_MCPS_SapHandler;
static SMAC_APP_MLME_SapHandler_t gSMAC_APP_MLME_SapHandler;
/************************************************************************************
*************************************************************************************
* Private definitions
*************************************************************************************
************************************************************************************/
 
#define smacInstance 0

#endif /* SMAC_H_ */
