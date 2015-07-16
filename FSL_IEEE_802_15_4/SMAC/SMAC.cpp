/**************************************************************************************************
* SMAC implementation.
* 
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
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
#include "SMAC.h"
#include "PhyInterface.h"
#include "EmbeddedTypes.h"

#include "SMAC_Config.h"

#include "MemManager.h"
#include "FunctionLib.h"
#if 0
#include "panic.h"
#endif 

#include "cmsis_os.h"
#include "rtos.h"

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/          
uint8_t gTotalChannels;

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static smacStates_t    smacState;
/*volatile*/ static prssPacketPtr_t smacProccesPacketPtr;
static phyRxParams_t   smacLastDataRxParams;
#if defined (gPHY_802_15_4g_d)
static smacPacket_t smacPacketConfig;
#endif
static macToPdDataMessage_t * gSmacDataMessage;
static macToPlmeMessage_t *   gSmacMlmeMessage;

static uint16_t u16PanID;
static uint16_t u16ShortSrcAddress;

static uint8_t u8AckRetryCounter = 0;
static uint8_t u8CCARetryCounter = 0;

static uint8_t mSmacInitialized;
static uint8_t mSmacTimeoutAsked;
static uint8_t u8BackoffTimerId;
static txContextConfig_t txConfigurator;
static uint8_t u8SmacSeqNo;
//Sap Handlers Called by PHY
phyStatus_t PD_SMAC_SapHandler(void* pMsg, instanceId_t smacInstanceId);
phyStatus_t PLME_SMAC_SapHandler(void* pMsg, instanceId_t smacInstanceId);

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/
static bool_t SMACPacketCheck(pdDataToMacMessage_t* pMsgFromPhy);
static void BackoffTimeElapsed(void const *arg);

osTimerDef (SmacTimer, BackoffTimeElapsed);                          
/************************************************************************************
*************************************************************************************
* Interface functions
*************************************************************************************
************************************************************************************/
uint8_t TMR_AllocateTimer
(
    void
)
{
    //RtosTimer smac_timer(u8BackoffTimerId, osTimerOnce , NULL);
    return 0;
}

uint8_t TMR_StartSingleShotTimer
(
    uint8_t timerID,
    uint32_t timeInMilliseconds,
    void (*pfTimerCallBack)(const void *),
    void *param
)
{
    //RtosTimer smac_timer(pfTimerCallBack, osTimerOnce , NULL);

    osTimerId id = osTimerCreate (osTimer(SmacTimer), osTimerOnce, NULL);
    osTimerStart (id, timeInMilliseconds);
    return 0;
}


/***********************************************************************************/
/******************************** SMAC Data primitives *****************************/
/***********************************************************************************/

/************************************************************************************
* MCPSDataRequest
* 
* This data primitive is used to send an over the air packet. This is an asynchronous 
* function,  it means it asks SMAC to transmit one OTA packet,  but when the function 
* returns it is not sent already.
*
************************************************************************************/
smacErrors_t MCPSDataRequest
(
txPacket_t *psTxPacket        //IN:Pointer to the packet to be transmitted
)
{  
  macToPdDataMessage_t *pMsg;
  phyStatus_t u8PhyRes = gPhySuccess_c; 
  
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif      /* TRUE == smacInitializationValidation_d */
  
#if(TRUE == smacParametersValidation_d)
  uint8_t u8MaxLen=0;
  
  u8MaxLen = gMaxSmacSDULength_c;
  
  if((NULL == psTxPacket) || (u8MaxLen < psTxPacket->u8DataLength))
  {
    return gErrorOutOfRange_c;
  }  
#endif         /* TRUE == smacParametersValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }

  u8SmacSeqNo++;
  u8AckRetryCounter = 0;
  u8CCARetryCounter = 0;
  
  psTxPacket->u8DataLength = psTxPacket->u8DataLength + gSmacHeaderBytes_c; 
  
  pMsg = (macToPdDataMessage_t*)MEM_BufferAlloc( sizeof(macToPdDataMessage_t) +
                         psTxPacket->u8DataLength);
  if(pMsg == NULL )
  {
    return gErrorNoResourcesAvailable_c;
  }
  /* Fill with Phy related data */
  pMsg->macInstance = smacInstance;
  pMsg->msgType = gPdDataReq_c;
  //SMAC doesn't use slotted mode
  pMsg->msgData.dataReq.slottedTx = gPhyUnslottedMode_c;
  //start transmission immediately
  pMsg->msgData.dataReq.startTime = gPhySeqStartAsap_c;
#ifdef gPHY_802_15_4g_d
  //for sub-Gig phy handles duration in case of ACK
  pMsg->msgData.dataReq.txDuration = 0xFFFFFFFF;
#else
  if(txConfigurator.autoAck && 
     psTxPacket->smacHeader.destAddr != 0xFFFF &&
       psTxPacket->smacHeader.panId != 0xFFFF)
  {
                                    //Turn@       +       phy payload(symbols)+ Turn@ + ACK
    pMsg->msgData.dataReq.txDuration = 12 + (6 + psTxPacket->u8DataLength + 2)*2 + 12 + 42; 
    if(txConfigurator.ccaBeforeTx)
    {
      pMsg->msgData.dataReq.txDuration += 0x08; //CCA Duration: 8 symbols
    }
  }
  else
  {
    pMsg->msgData.dataReq.txDuration = 0xFFFFFFFF;
  }
#endif
  pMsg->msgData.dataReq.psduLength = psTxPacket->u8DataLength;
  pMsg->msgData.dataReq.pPsdu = (uint8_t*)&pMsg->msgData.dataReq.pPsdu +
    sizeof(pMsg->msgData.dataReq.pPsdu);
  
  FLib_MemCpy(pMsg->msgData.dataReq.pPsdu, &(psTxPacket->smacHeader), gSmacHeaderBytes_c);
  FLib_MemCpy(pMsg->msgData.dataReq.pPsdu + gSmacHeaderBytes_c, 
              &(psTxPacket->smacPdu), 
              psTxPacket->u8DataLength - gSmacHeaderBytes_c);
  
  if(txConfigurator.ccaBeforeTx)
  {
    //tell phy to perform CCA before transmission
    pMsg->msgData.dataReq.CCABeforeTx = gPhyCCAMode1_c;
  }
  else
  {
    pMsg->msgData.dataReq.CCABeforeTx = gPhyNoCCABeforeTx_c;
  }
  
  if(txConfigurator.autoAck && 
     psTxPacket->smacHeader.destAddr != 0xFFFF &&
       psTxPacket->smacHeader.panId != 0xFFFF)
  {
    //set frame control option: ACK.
    pMsg->msgData.dataReq.pPsdu[0] |= FRAME_CTRL_ACK_FIELD_SET;
    pMsg->msgData.dataReq.ackRequired = gPhyRxAckRqd_c;
  }
  else
  {
    pMsg->msgData.dataReq.ackRequired = gPhyNoAckRqd_c;
  }
  //set sequence number;
  pMsg->msgData.dataReq.pPsdu[2] = u8SmacSeqNo;
  gSmacDataMessage = pMsg;      //Store pointer for freeing later 
  u8PhyRes = MAC_PD_SapHandler(pMsg, 0);
  
  psTxPacket->u8DataLength -= gSmacHeaderBytes_c;
  
  if(u8PhyRes == gPhySuccess_c)
  {
    smacState= mSmacStateTransmitting_c; 
    return gErrorNoError_c;
  }
  else
  {
    MEM_BufferFree(gSmacDataMessage);
    gSmacDataMessage = NULL;
    return gErrorNoResourcesAvailable_c;
  }
}

/************************************************************************************
* MLMEConfigureTxContext
* 
* This management primitive is used to configure the conditions under which SMAC will
* perform a transmission OTA.
*
************************************************************************************/
smacErrors_t MLMEConfigureTxContext(txContextConfig_t* pTxConfig)
{
  if( (pTxConfig->autoAck == FALSE && pTxConfig->retryCountAckFail !=0) || 
      (pTxConfig->ccaBeforeTx == FALSE && pTxConfig->retryCountCCAFail !=0) )
  {
    return gErrorNoValidCondition_c;
  }
  if( pTxConfig->retryCountAckFail > gMaxRetriesAllowed_c || 
     pTxConfig->retryCountCCAFail > gMaxRetriesAllowed_c)
  {
    return gErrorOutOfRange_c;
  }
  txConfigurator.autoAck           = pTxConfig->autoAck;
  txConfigurator.ccaBeforeTx       = pTxConfig->ccaBeforeTx;
  txConfigurator.retryCountAckFail = pTxConfig->retryCountAckFail;
  txConfigurator.retryCountCCAFail = pTxConfig->retryCountCCAFail;
  
  return gErrorNoError_c;
}

/************************************************************************************
* MLMERXEnableRequest
* 
* Function used to place the radio into receive mode 
*
************************************************************************************/
smacErrors_t MLMERXEnableRequest
(
rxPacket_t *gsRxPacket, //OUT: Pointer to the structure where the reception results 
//     will be stored.
smacTime_t stTimeout     //IN:  64-bit timeout value, absolute value in symbols
)
{
  
  uint8_t u8PhyRes = 0; 
  macToPlmeMessage_t lMsg;
  
#if(TRUE == smacParametersValidation_d)
  uint8_t u8MaxLen=0;
  
  u8MaxLen = gMaxSmacSDULength_c;
#endif         /* TRUE == smacParametersValidation_d */
  
#if(TRUE == smacParametersValidation_d)
  if((NULL == gsRxPacket) || (u8MaxLen < gsRxPacket->u8MaxDataLength))
  {
    return gErrorOutOfRange_c;
  }
#endif     /* TRUE == smacParametersValidation_d */
  
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif     /* TRUE == smacInitializationValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }
  if(stTimeout)
  {
    lMsg.msgType = gPlmeSetTRxStateReq_c;
    lMsg.msgData.setTRxStateReq.startTime = gPhySeqStartAsap_c;
    lMsg.macInstance = smacInstance;
    lMsg.msgData.setTRxStateReq.state = gPhySetRxOn_c;
    mSmacTimeoutAsked = TRUE;
    lMsg.msgData.setTRxStateReq.rxDuration = stTimeout;
  }
  else
  {
    lMsg.msgType = gPlmeSetReq_c;
    lMsg.msgData.setReq.PibAttribute = gPhyPibRxOnWhenIdle;
    lMsg.msgData.setReq.PibAttributeValue = (uint64_t)1;
  }
  u8PhyRes = MAC_PLME_SapHandler(&lMsg, 0);
  if(u8PhyRes == gPhySuccess_c)
  {
    gsRxPacket->rxStatus = rxProcessingReceptionStatus_c;
    smacProccesPacketPtr.smacRxPacketPointer  = gsRxPacket;
    
    smacState= mSmacStateReceiving_c; 
    return gErrorNoError_c;
  }
  else
  {
    return gErrorNoResourcesAvailable_c;
  }
}
#if defined (gPHY_802_15_4g_d)
/************************************************************************************
* MLMESetPreambleLength
* 
* Function used to change the preamble size in the OTA Packet 
*
************************************************************************************/
smacErrors_t MLMESetPreambleLength
(
uint16_t u16preambleLength
)
{
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif     /* TRUE == smacInitializationValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }
  smacPacketConfig.u16PreambleLength = u16preambleLength;
  gPhyPib.mPIBphyFSKPreambleRepetitions = u16preambleLength;
  PhyPib_RFUpdatePreambleLength();
  
  return gErrorNoError_c;
  
}

/************************************************************************************
* MLMESetSyncWordSize
* 
* Function used to change the synchronization word size. Values from 0 to 8 required. 
* IMPORTANT-> Use below arguments only (indicating a direct value from 1-8 will not work) 
* Inputs      : 
*   SyncConfig_SyncSize_1             
*   SyncConfig_SyncSize_2              
*   SyncConfig_SyncSize_3               
*   SyncConfig_SyncSize_4               
*   SyncConfig_SyncSize_5                
*   SyncConfig_SyncSize_6                
*   SyncConfig_SyncSize_7              
*   SyncConfig_SyncSize_8    
*
************************************************************************************/
smacErrors_t MLMESetSyncWordSize
(
uint8_t u8syncWordSize
)
{
  
  phyStatus_t status;
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif     /* TRUE == smacInitializationValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }
  
  status = (phyStatus_t)Phy_SetSyncWordSize(u8syncWordSize);
  if(status == gPhyInvalidParameter_c)
    return gErrorOutOfRange_c;
  
  smacPacketConfig.u8SyncWordSize = u8syncWordSize;
  
  return gErrorNoError_c;
  
}

/************************************************************************************
* MLMESetSyncWordValue
* 
* Function used to change the synchronization word value. 
*     
*
************************************************************************************/
smacErrors_t MLMESetSyncWordValue
(
uint8_t *u8syncWordValue
)
{
  uint8_t syncWordSizeTemp = smacPacketConfig.u8SyncWordSize;
  uint8_t syncValueRegIndex = 0;
  
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif     /* TRUE == smacInitializationValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }
  
  smacPacketConfig.u8SyncWordValue = u8syncWordValue;
  
  while (syncWordSizeTemp--)
  {
    Phy_SetSyncWordValue(syncValueRegIndex, (uint8_t)*u8syncWordValue);
    syncValueRegIndex++;
    u8syncWordValue++;
  }
  while(syncValueRegIndex < 8)
  {
    Phy_SetSyncWordValue(syncValueRegIndex, 0x00);
    syncValueRegIndex++;
  }
  
  return gErrorNoError_c;
  
}

/************************************************************************************
* MLMEPacketConfig
* 
*  
*
************************************************************************************/
smacErrors_t MLMEPacketConfig
(
packetConfig_t *pPacketCfg
)
{
  smacErrors_t err = gErrorNoError_c;
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif     /* TRUE == smacInitializationValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  } 
  err  = MLMESetSyncWordSize(pPacketCfg->u8SyncWordSize);
  err |=  MLMESetSyncWordValue(pPacketCfg->pu8SyncWord);
  err |= MLMESetPreambleLength(pPacketCfg->u16PreambleSize);
  if(err != gErrorNoError_c)
    return gErrorOutOfRange_c;
  
  return gErrorNoError_c;   
}

#endif

/************************************************************************************
* MLMESetChannelRequest
* 
*  
*
************************************************************************************/
smacErrors_t MLMESetChannelRequest
(
channels_t newChannel
)
{
  uint8_t errorVal;
  smacErrors_t err = gErrorNoError_c;
  
  macToPlmeMessage_t lMsg;
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif     /* TRUE == smacInitializationValidation_d */
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  } 
  lMsg.msgType = gPlmeSetReq_c;
  lMsg.macInstance = smacInstance;
  lMsg.msgData.setReq.PibAttribute = gPhyPibCurrentChannel_c;
  lMsg.msgData.setReq.PibAttributeValue = (uint64_t) newChannel;
  
  errorVal = MAC_PLME_SapHandler(&lMsg, 0);
  switch (errorVal)
  {
  case  gPhyBusy_c:
    err = gErrorBusy_c;
    break;
    
  case gPhyInvalidParameter_c:   
    err = gErrorOutOfRange_c;
    break;
    
  case gPhySuccess_c: 
    err = gErrorNoError_c;
    break;
    
  default:
    err = gErrorOutOfRange_c;
    break;
  }
  
  return err;
}


/************************************************************************************
* MLMESetAdditionalRFOffset
*
*************************************************************************************/
smacErrors_t MLMESetAdditionalRFOffset (uint32_t additionalRFOffset)
{
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif     /* TRUE == smacInitializationValidation_d */

#ifdef gIncludeCalibrationOption
  gPhyPib.mPIBAdditionalRFFrequencyOffset = additionalRFOffset;
  return gErrorNoError_c;
#else
  return gErrorNoResourcesAvailable_c;
#endif
}


/************************************************************************************
* MLMEGetAdditionalRFOffset
*
*************************************************************************************/
uint32_t MLMEGetAdditionalRFOffset( void )
{
#ifdef gIncludeCalibrationOption
  return gPhyPib.mPIBAdditionalRFFrequencyOffset;
#else
  return 0;
#endif
}

#if defined (gPHY_802_15_4g_d)
/************************************************************************************
* MLMESetFreqBand
*
************************************************************************************/
smacErrors_t MLMESetFreqBand
(
smacFrequencyBands_t freqBand,
smacRFModes_t phyMode
)
{               
  return gErrorNoResourcesAvailable_c;
}

smacErrors_t MLMESetPhyMode(smacRFModes_t phyMode)
{
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif     /* TRUE == smacInitializationValidation_d */
  phyStatus_t err;
  macToPlmeMessage_t lMsg;
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }
  lMsg.macInstance = smacInstance;
  lMsg.msgType = gPlmeSetReq_c;
  lMsg.msgData.setReq.PibAttribute = gPhyPibCurrentMode_c;
  lMsg.msgData.setReq.PibAttributeValue = (uint64_t)phyMode;
  err = MAC_PLME_SapHandler(&lMsg, 0);
  if(err == gPhyInvalidParameter_c)
    return gErrorOutOfRange_c;
  if(err == gPhyBusy_c)
    return gErrorBusy_c;
  
  gTotalChannels = gPhyPib.pPIBphyRfConstants->totalNumChannels;
  
  return gErrorNoError_c;
}

#endif
/************************************************************************************
* MLMEGetChannelRequest
* 
*  
*
************************************************************************************/
channels_t MLMEGetChannelRequest
(
void 
)
{
  channels_t currentChannel;
  macToPlmeMessage_t lMsg;  
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    //panic(0,0,0,0);
  }
#endif     /* TRUE == smacInitializationValidation_d */ 
  lMsg.msgType = gPlmeGetReq_c;
  lMsg.macInstance = smacInstance;
  lMsg.msgData.getReq.PibAttribute = gPhyPibCurrentChannel_c;
  lMsg.msgData.getReq.pPibAttributeValue = (uint64_t*)&currentChannel;
  MAC_PLME_SapHandler(&lMsg, 0);
  return currentChannel;    
}

#if defined (gPHY_802_15_4g_d)
/************************************************************************************
* MLMERssi
* 
*  
* 
************************************************************************************/
uint8_t MLMERssi(void )
{
  uint8_t rssiVal;
  
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif     /* TRUE == smacInitializationValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }
  
  rssiVal = Phy_GetRssi();
  return rssiVal;
}

/************************************************************************************
* MLMESetCCADuration
* 
*  
* 
************************************************************************************/
smacErrors_t MLMESetCCADuration(uint64_t usCCADuration )
{
  macToPlmeMessage_t lMsg;
  phyStatus_t status;
  
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif     /* TRUE == smacInitializationValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }
  
  usCCADuration = TIME_US_TO_TICKS(usCCADuration);
  Phy_TimeDivider((phyTime_t*)&usCCADuration);
  
  lMsg.msgType = gPlmeSetReq_c;
  lMsg.msgData.setReq.PibAttribute = gPhyPibCCADuration_c;
  lMsg.msgData.setReq.PibAttributeValue = usCCADuration;
  status = MAC_PLME_SapHandler(&lMsg, 0);
  
  if(status == gPhySuccess_c)
    return gErrorNoError_c;
  else
    return gErrorNoResourcesAvailable_c;
}

/************************************************************************************
* MLMESetInterPacketRxDelay
* 
* IMPORTANT-> Use below arguments only (indicating a direct value from 1-12 will not work)
* Inputs      :
*
* InterPacketRxDelay_0
* InterPacketRxDelay_1
* InterPacketRxDelay_2
* InterPacketRxDelay_3
* InterPacketRxDelay_4
* InterPacketRxDelay_5
* InterPacketRxDelay_6
* InterPacketRxDelay_7
* InterPacketRxDelay_8
* InterPacketRxDelay_9
* InterPacketRxDelay_A
* InterPacketRxDelay_B
************************************************************************************/
smacErrors_t MLMESetInterPacketRxDelay
(
uint8_t u8InterPacketRxDelay
)
{   
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif     /* TRUE == smacInitializationValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }
  
  if (gPhySuccess_c != Phy_SetInterPacketRxDelay(u8InterPacketRxDelay))
  {
    return gErrorOutOfRange_c;
  }
  return gErrorNoError_c;
}

#endif
/************************************************************************************
* MLMERXDisableRequest
* 
* Returns the radio to idle mode from receive mode.
*
************************************************************************************/
smacErrors_t MLMERXDisableRequest(void)
{
  macToPlmeMessage_t lMsg;
  phyStatus_t err;
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif
  if((mSmacStateReceiving_c != smacState) && (mSmacStateIdle_c != smacState))
  {
    return gErrorNoValidCondition_c;
  }
  lMsg.macInstance = smacInstance;
  if(!mSmacTimeoutAsked)
  {
    lMsg.msgType                          = gPlmeSetReq_c;
    lMsg.msgData.setReq.PibAttribute      = gPhyPibRxOnWhenIdle;
    lMsg.msgData.setReq.PibAttributeValue = (uint64_t)0;
    err = MAC_PLME_SapHandler(&lMsg, 0);
    if(err != gPhySuccess_c)
      return gErrorBusy_c;
  }
  else
  {
    lMsg.msgType = gPlmeSetTRxStateReq_c;
    lMsg.msgData.setTRxStateReq.state = gPhyForceTRxOff_c;
    (void)MAC_PLME_SapHandler(&lMsg, 0);
    mSmacTimeoutAsked = FALSE;
  }
  smacState= mSmacStateIdle_c;
  
  return gErrorNoError_c;
  
}

/*@CMA, Conn Test Added*/
/************************************************************************************
* MLMETXDisableRequest
* 
* Returns the radio to idle mode from Tx mode.
*
************************************************************************************/
void MLMETXDisableRequest(void)
{
  macToPlmeMessage_t lMsg;
  lMsg.macInstance = smacInstance;
  lMsg.msgType     = gPlmeSetTRxStateReq_c;
  lMsg.msgData.setTRxStateReq.state = gPhyForceTRxOff_c;
  (void)MAC_PLME_SapHandler(&lMsg, 0);
  if(gSmacDataMessage != NULL)
  {
    (void)MEM_BufferFree(gSmacDataMessage);
    gSmacDataMessage = NULL;
  }
  smacState= mSmacStateIdle_c;
}

/************************************************************************************
* MLMELinkQuality
* 
* This  function  returns  an  integer  value  that is the link quality from the last 
* received packet of the form:  dBm = (-Link Quality/2).
*
************************************************************************************/
uint8_t MLMELinkQuality(void)
{
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return 0;
  }
#endif
  return smacLastDataRxParams.linkQuality;
}

/************************************************************************************
* MLMEPAOutputAdjust
* 
*
************************************************************************************/
smacErrors_t MLMEPAOutputAdjust
( 
uint8_t u8PaValue
)
{
  macToPlmeMessage_t lMsg;
  smacErrors_t err = gErrorNoError_c;
  uint8_t errorVal;     
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif /* TRUE == smacInitializationValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }
  lMsg.macInstance = smacInstance;
  lMsg.msgType     = gPlmeSetReq_c;
  lMsg.msgData.setReq.PibAttribute      =  gPhyPibTransmitPower_c;
  lMsg.msgData.setReq.PibAttributeValue =  (uint64_t) u8PaValue;
  errorVal = MAC_PLME_SapHandler(&lMsg, 0);
  switch (errorVal)
  {
  case  gPhyBusy_c:
    err = gErrorBusy_c;
    break;
    
  case gPhyInvalidParameter_c:   
    err = gErrorOutOfRange_c;
    break;
    
  case gPhySuccess_c: 
    err = gErrorNoError_c;
    break;
    
  default:
    err = gErrorOutOfRange_c; 
    break;
  }
  
  return err;
}

/************************************************************************************
* MLMEScanRequest
* 
* This  function  returns  the RSSI value of the channel passes as a parameter  
* 
*
************************************************************************************/
smacErrors_t MLMEScanRequest(channels_t u8ChannelToScan)
{
  smacErrors_t err = gErrorNoError_c;
  phyStatus_t u8PhyRes;
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif /* TRUE == smacInitializationValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }
  if(u8ChannelToScan != MLMEGetChannelRequest())
    err = MLMESetChannelRequest(u8ChannelToScan);
  if(err != gErrorNoError_c)
    return err;
  
  macToPlmeMessage_t* pMsg = (macToPlmeMessage_t*)MEM_BufferAlloc(sizeof(macToPlmeMessage_t));
  pMsg->msgType = gPlmeEdReq_c;
  pMsg->msgData.edReq.startTime = gPhySeqStartAsap_c;
  pMsg->macInstance = smacInstance;
  gSmacMlmeMessage = pMsg;
  u8PhyRes = MAC_PLME_SapHandler(pMsg,0);
  if(u8PhyRes != gPhySuccess_c)
  {
    MEM_BufferFree(gSmacMlmeMessage);
    gSmacMlmeMessage = NULL;
    return gErrorBusy_c;
  }
  smacState = mSmacStateScanningChannels_c;
  return gErrorNoError_c;
}


/************************************************************************************
* MLMECcaRequest
* 
* This  function  performs Clear Channel Assessment on the active channel  
* 
*
************************************************************************************/
smacErrors_t MLMECcaRequest()
{
  phyStatus_t u8PhyRes;
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif /* TRUE == smacInitializationValidation_d */
  
  if(mSmacStateIdle_c != smacState)
  {
    return gErrorBusy_c;
  }
  macToPlmeMessage_t* pMsg = (macToPlmeMessage_t*)MEM_BufferAlloc(sizeof(macToPlmeMessage_t));
  pMsg->msgType = gPlmeCcaReq_c;
  pMsg->msgData.ccaReq.ccaType = gPhyCCAMode1_c;
  pMsg->msgData.ccaReq.contCcaMode = gPhyContCcaDisabled;
  pMsg->macInstance = smacInstance;
  gSmacMlmeMessage = pMsg;
  u8PhyRes = MAC_PLME_SapHandler(pMsg,0);
  if(u8PhyRes != gPhySuccess_c)
  {
    MEM_BufferFree(gSmacMlmeMessage);
    gSmacMlmeMessage = NULL;
    return gErrorBusy_c;
  }
  smacState = mSmacStatePerformingCca_c;
  return gErrorNoError_c;
}
/************************************************************************************
* MLMEPhySoftReset
* 
* This function performs a software reset on the radio, PHY and SMAC state machines.
* 
*
************************************************************************************/
smacErrors_t MLMEPhySoftReset
(
void    
)
{   
  macToPlmeMessage_t lMsg;
#if(TRUE == smacInitializationValidation_d)
  if(FALSE == mSmacInitialized)
  {
    return gErrorNoValidCondition_c;
  }
#endif /* TRUE == smacInitializationValidation_d */
//  
//  if(mSmacStateIdle_c != smacState)
//  {
//    return gErrorBusy_c;
//  }
  lMsg.macInstance = smacInstance;
  lMsg.msgType     = gPlmeSetTRxStateReq_c;
  lMsg.msgData.setTRxStateReq.state = gPhyForceTRxOff_c;
  (void)MAC_PLME_SapHandler(&lMsg, 0);
  smacState= mSmacStateIdle_c; 
  
  return gErrorNoError_c;
}
/************************************************************************************
* PD_SMAC_SapHandler
* 
* This SAP handles data confirm and data indication from PHY.
* 
************************************************************************************/
phyStatus_t PD_SMAC_SapHandler(void* pMsg, instanceId_t smacInstanceId)
{
  phyStatus_t status = gPhyInvalidPrimitive_c;
  smacToAppDataMessage_t* pSmacMsg;
  pdDataToMacMessage_t* pDataMsg = (pdDataToMacMessage_t*)pMsg;
  (void)smacInstanceId;
  
  switch(pDataMsg->msgType)
  {
  case gPdDataCnf_c:
    //no data request was fired
    if(NULL == gSmacDataMessage)
    {
      status = gPhySuccess_c;
    }
    else
    {
      //phy finished work with the data request packet so it can be freed
      MEM_BufferFree(gSmacDataMessage);
      gSmacDataMessage = NULL;
      
      pSmacMsg = (smacToAppDataMessage_t*)MEM_BufferAlloc(sizeof(smacToAppDataMessage_t));
      if(pSmacMsg == NULL)
      {
        status = gPhySuccess_c;
      }
      else
      {
        pSmacMsg->msgType = gMcpsDataCnf_c;
        pSmacMsg->msgData.dataCnf.status = gErrorNoError_c;
        // call App Sap
        gSMAC_APP_MCPS_SapHandler(pSmacMsg,smacInstance); 
      }
      smacState = mSmacStateIdle_c;
    }
    break;
  case gPdDataInd_c:
    if(FALSE == SMACPacketCheck(pDataMsg))
    {
      MEM_BufferFree(pDataMsg);
      status = gPhySuccess_c;
    }
    else
    {
      smacLastDataRxParams.linkQuality = ((pdDataToMacMessage_t*)pMsg)->msgData.dataInd.ppduLinkQuality;
      smacLastDataRxParams.timeStamp = (phyTime_t)((pdDataToMacMessage_t*)pMsg)->msgData.dataInd.timeStamp;
      smacProccesPacketPtr.smacRxPacketPointer->rxStatus = rxSuccessStatus_c;
      
      // in case no timeout was asked we need to unset RXOnWhenIdle Pib.
      if(!mSmacTimeoutAsked) 
      {
        (void)MLMERXDisableRequest();
      }
      smacProccesPacketPtr.smacRxPacketPointer->u8DataLength = 
        pDataMsg->msgData.dataInd.psduLength - gSmacHeaderBytes_c;
      FLib_MemCpy(&smacProccesPacketPtr.smacRxPacketPointer->smacHeader, 
                  ((smacHeader_t*)pDataMsg->msgData.dataInd.pPsdu), 
                  gSmacHeaderBytes_c);
      FLib_MemCpy(&smacProccesPacketPtr.smacRxPacketPointer->smacPdu, 
                  ((smacPdu_t*)(pDataMsg->msgData.dataInd.pPsdu + gSmacHeaderBytes_c)), 
                  smacProccesPacketPtr.smacRxPacketPointer->u8DataLength);
      
      pSmacMsg = (smacToAppDataMessage_t*)MEM_BufferAlloc(sizeof(smacToAppDataMessage_t));
      if(pSmacMsg == NULL)
      {
        status = gPhySuccess_c;
      }
      else
      {
        pSmacMsg->msgType = gMcpsDataInd_c;
        pSmacMsg->msgData.dataInd.pRxPacket = smacProccesPacketPtr.smacRxPacketPointer;
        pSmacMsg->msgData.dataInd.u8LastRxRssi = PhyGetLastRxRssiValue();
        gSMAC_APP_MCPS_SapHandler(pSmacMsg,smacInstance); 
      }
      smacState = mSmacStateIdle_c;
    }
    break;
  default:
    break;
  }
  MEM_BufferFree(pMsg);
  return status;
}

/************************************************************************************
* PLME_SMAC_SapHandler
* 
* This SAP handles management for confirms and indications from PHY.
* 
************************************************************************************/

phyStatus_t PLME_SMAC_SapHandler(void* pMsg, instanceId_t smacInstanceId)
{
  MEM_BufferFree(gSmacMlmeMessage);
  gSmacMlmeMessage = NULL;
  uint32_t backOffTime;

  plmeToMacMessage_t* pPlmeMsg = (plmeToMacMessage_t*)pMsg;
  
  smacToAppMlmeMessage_t* pSmacToApp;
  smacToAppDataMessage_t* pSmacMsg;
  switch(pPlmeMsg->msgType)
  {
  case gPlmeCcaCnf_c:
    if(pPlmeMsg->msgData.ccaCnf.status == gPhyChannelBusy_c && 
       smacState == mSmacStateTransmitting_c)
    {
      if(txConfigurator.ccaBeforeTx)
      { 
          if(txConfigurator.retryCountCCAFail > u8CCARetryCounter)
          {
            //increment cca fail counter
            u8CCARetryCounter++;
            //get random number for backoff time.
            RNG_GetRandomNo(&backOffTime);
            //start event timer. After time elapses, Data request will be fired.
            TMR_StartSingleShotTimer(u8BackoffTimerId, ((backOffTime & gMaxBackoffTime_c) + gMinBackoffTime_c), BackoffTimeElapsed, NULL);
          }
          else
          {
            MEM_BufferFree(gSmacDataMessage);
            gSmacDataMessage = NULL;
            
            //retries failed so create message for the application
            pSmacMsg = (smacToAppDataMessage_t*)MEM_BufferAlloc(sizeof(smacToAppDataMessage_t));
            if(pSmacMsg != NULL)
            {
              //error type : Channel Busy
              pSmacMsg->msgData.dataCnf.status = gErrorChannelBusy_c;
              //type is Data Confirm
              pSmacMsg->msgType = gMcpsDataCnf_c;
              gSMAC_APP_MCPS_SapHandler(pSmacMsg, smacInstance);
            }
            //place SMAC into idle state
            smacState = mSmacStateIdle_c;
          }
      }
      MEM_BufferFree(pMsg);
      return gPhySuccess_c;
    }
    //if SMAC isn't in TX then definitely it is a CCA confirm
    //allocate a message for the application
    pSmacToApp = (smacToAppMlmeMessage_t*)MEM_BufferAlloc(sizeof(smacToAppMlmeMessage_t));
    if(pSmacToApp != NULL)
    {
      //type is CCA Confirm
      pSmacToApp->msgType = gMlmeCcaCnf_c;
      //Channel status translated into SMAC messages: idle channel means no error.
      if(pPlmeMsg->msgData.ccaCnf.status == gPhyChannelIdle_c)
      {
        pSmacToApp->msgData.ccaCnf.status = gErrorNoError_c;
      }
      else
      {
        pSmacToApp->msgData.ccaCnf.status = gErrorChannelBusy_c;
      }
    }
    break;
  case gPlmeEdCnf_c:
    //allocate a message for the application
    pSmacToApp = (smacToAppMlmeMessage_t*)MEM_BufferAlloc(sizeof(smacToAppMlmeMessage_t));
    if(pSmacToApp != NULL)
    {
      //message type is ED Confirm
      pSmacToApp->msgType = gMlmeEdCnf_c;
      if(pPlmeMsg->msgData.edCnf.status == gPhySuccess_c)
      {
        pSmacToApp->msgData.edCnf.status = gErrorNoError_c;
        pSmacToApp->msgData.edCnf.energyLevel = pPlmeMsg->msgData.edCnf.energyLevel;
        pSmacToApp->msgData.edCnf.energyLeveldB = pPlmeMsg->msgData.edCnf.energyLeveldB;
        pSmacToApp->msgData.edCnf.scannedChannel = MLMEGetChannelRequest();
      }
      else
      {
        pSmacToApp->msgData.edCnf.status = gErrorBusy_c;
      }
    }
    break;
  case gPlmeTimeoutInd_c:
    if(smacState == mSmacStateTransmitting_c)
    {
      if(txConfigurator.autoAck)
      {
        //re-arm retries for channel busy at retransmission.
        u8CCARetryCounter = 0;
        
        if(txConfigurator.retryCountAckFail > u8AckRetryCounter)
        {
          u8AckRetryCounter++;
          
          RNG_GetRandomNo(&backOffTime);
          //start event timer. After time elapses, Data request will be fired.
          TMR_StartSingleShotTimer(u8BackoffTimerId, ((backOffTime & gMaxBackoffTime_c) + gMinBackoffTime_c), BackoffTimeElapsed, NULL);
        }
        else
        {
          (void)MEM_BufferFree(gSmacDataMessage);
          gSmacDataMessage = NULL;
          
          //retries failed so create message for the application
          pSmacMsg = (smacToAppDataMessage_t*)MEM_BufferAlloc(sizeof(smacToAppDataMessage_t));
          if(pSmacMsg != NULL)
          {
            //set error code: No Ack
            pSmacMsg->msgData.dataCnf.status = gErrorNoAck_c;
            //type is Data Confirm
            pSmacMsg->msgType = gMcpsDataCnf_c;
            
            gSMAC_APP_MCPS_SapHandler(pSmacMsg, smacInstance);
          }
          //place SMAC into idle state
          smacState = mSmacStateIdle_c;
        }
      }
      MEM_BufferFree(pMsg);
      return gPhySuccess_c;
    }
    //if no ack timeout was received then it is definitely a RX timeout
    pSmacToApp = (smacToAppMlmeMessage_t*)MEM_BufferAlloc(sizeof(smacToAppMlmeMessage_t));
    if(pSmacToApp != NULL)
    {
      if(smacState == mSmacStateReceiving_c)
      {
        smacProccesPacketPtr.smacRxPacketPointer->rxStatus = rxTimeOutStatus_c;
      }
      pSmacToApp->msgType = gMlmeTimeoutInd_c;
    }
    break;
  case gPlme_UnexpectedRadioResetInd_c:
    pSmacToApp = (smacToAppMlmeMessage_t*)MEM_BufferAlloc(sizeof(smacToAppMlmeMessage_t));
    if(pSmacToApp != NULL)
      pSmacToApp->msgType = gMlme_UnexpectedRadioResetInd_c;
    break;
  default:
    //MEM_BufferFree(pSmacToApp);
    MEM_BufferFree(pMsg);
    return gPhySuccess_c;
  }
  smacState = mSmacStateIdle_c;
  //send message to upper layer if it is not NULL
  if(pSmacToApp != NULL)
    (gSMAC_APP_MLME_SapHandler)(pSmacToApp,0);
  MEM_BufferFree(pMsg);
  return gPhySuccess_c;
}

/************************************************************************************
* Smac_RegisterSapHandlers
* 
* This function helps the user register the handlers for the messages that come from 
* SMAC.
* 
************************************************************************************/

void Smac_RegisterSapHandlers(
                              SMAC_APP_MCPS_SapHandler_t pSMAC_APP_MCPS_SapHandler,
                              SMAC_APP_MLME_SapHandler_t pSMAC_APP_MLME_SapHandler,
                              instanceId_t smacInstanceId
                                )
{
  gSMAC_APP_MCPS_SapHandler = pSMAC_APP_MCPS_SapHandler;
  gSMAC_APP_MLME_SapHandler = pSMAC_APP_MLME_SapHandler;
  (void)smacInstanceId;
}

/************************************************************************************
* SMACFillHeader
* 
* This function helps the user fill the SMAC header(short hardcoded MAC header) with
* the desired short destination address.
* 
************************************************************************************/

void SMACFillHeader(smacHeader_t* pSmacHeader, uint16_t destAddr)
{
  pSmacHeader->frameControl = gSmacDefaultFrameCtrl;
  pSmacHeader->panId        = u16PanID;
  pSmacHeader->seqNo        = gSmacDefaultSeqNo;
  pSmacHeader->srcAddr      = u16ShortSrcAddress;
  pSmacHeader->destAddr     = destAddr;
}

/************************************************************************************
* InitSmac
* 
* Basic SMAC initialisation.
* 
************************************************************************************/
void InitSmac(void)
{
  /* SMAC Initialization */
  smacState = mSmacStateIdle_c;
  smacLastDataRxParams.linkQuality = 0;
  smacLastDataRxParams.timeStamp = 0;
  uint32_t u32RandomNo;
#if defined(gPHY_802_15_4g_d)
  gTotalChannels = gPhyPib.pPIBphyRfConstants->totalNumChannels;
#else
  gTotalChannels = 26;
#endif
  
  
#if(TRUE == smacInitializationValidation_d)
  mSmacInitialized = TRUE;  
#endif
  txConfigurator.autoAck = FALSE;
  txConfigurator.ccaBeforeTx = FALSE;
  txConfigurator.retryCountAckFail = 0;
  txConfigurator.retryCountCCAFail = 0;
  u8BackoffTimerId = (uint8_t)TMR_AllocateTimer();
  RNG_Init();
  RNG_GetRandomNo(&u32RandomNo);
  u8SmacSeqNo = (uint8_t)u32RandomNo;
  //Notify the PHY what function to call for communicating with SMAC  
  Phy_RegisterSapHandlers((PD_MAC_SapHandler_t)PD_SMAC_SapHandler, (PLME_MAC_SapHandler_t)PLME_SMAC_SapHandler, 0);
  u16PanID = gDefaultPanID_c;
  u16ShortSrcAddress = gNodeAddress_c;
  (void)SMACSetShortSrcAddress(u16ShortSrcAddress);
  (void)SMACSetPanID(u16PanID);
}

/************************************************************************************
* SMACSetShortSrcAddress
* 
* This function sets the short source address so that PHY can perform filtering
* 
************************************************************************************/

smacErrors_t SMACSetShortSrcAddress(uint16_t nwShortAddress)
{
  macToPlmeMessage_t lMsg;
  lMsg.msgType = gPlmeSetReq_c;
  lMsg.msgData.setReq.PibAttribute = gPhyPibShortAddress_c;
  lMsg.msgData.setReq.PibAttributeValue = (uint64_t)nwShortAddress;
  
  phyStatus_t u8PhyRes = MAC_PLME_SapHandler(&lMsg,0);
  if(u8PhyRes == gPhyBusy_c || u8PhyRes == gPhyBusyTx_c || u8PhyRes == gPhyBusyRx_c)
    return gErrorBusy_c;
  if(u8PhyRes != gPhySuccess_c)
    return gErrorNoResourcesAvailable_c;
  u16ShortSrcAddress = nwShortAddress;
  return gErrorNoError_c;
}

/************************************************************************************
* SMACSetPanID
* 
* This function sets the pan ID so that PHY can perform filtering
* 
************************************************************************************/

smacErrors_t SMACSetPanID(uint16_t nwShortPanID)
{
  macToPlmeMessage_t lMsg;
  lMsg.msgType = gPlmeSetReq_c;
  lMsg.msgData.setReq.PibAttribute = gPhyPibPanId_c;
  lMsg.msgData.setReq.PibAttributeValue = (uint64_t)nwShortPanID;
  
  phyStatus_t u8PhyRes = MAC_PLME_SapHandler(&lMsg,0);
  if(u8PhyRes == gPhyBusy_c || u8PhyRes == gPhyBusyTx_c || u8PhyRes == gPhyBusyRx_c)
    return gErrorBusy_c;
  if(u8PhyRes != gPhySuccess_c)
    return gErrorNoResourcesAvailable_c;
  u16PanID = nwShortPanID;
  return gErrorNoError_c;
}

void BackoffTimeElapsed(void const *arg)
{
  uint8_t u8PhyRes = MAC_PD_SapHandler(gSmacDataMessage, 0);
  if(u8PhyRes != gPhySuccess_c)
  {
    smacState = mSmacStateIdle_c;
    MEM_BufferFree(gSmacDataMessage);
    gSmacDataMessage = NULL;
  }
}

/************************************************************************************
* SMACPacketCheck
* 
* This function returns TRUE if Phy payload can be of SMAC packet type
* 
************************************************************************************/

bool_t SMACPacketCheck(pdDataToMacMessage_t* pMsgFromPhy)
{
  //check if packet is of type Data
  if( (pMsgFromPhy->msgData.dataInd.pPsdu[0] & 0x07) != 0x01 )
    return FALSE;
  //check if PSDU length is at least of smac header size.
  if( (pMsgFromPhy->msgData.dataInd.psduLength < gSmacHeaderBytes_c) )
    return FALSE;
  
  return TRUE;
}
