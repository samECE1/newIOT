/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file Connectivity_TestApp.c
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

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

#include "Application_Interface.h"
#include "Connectivity_Test_Platform.h"
/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Macros
*************************************************************************************
************************************************************************************/
#define gPrbs9BufferLength_c     ( 65 )
#define gContTxModSelectPN9_c    ( 2 )
#define gContTxModSelectOnes_c   ( 1 )
#define gContTxModSelectZeros_c  ( 0 )
#define SelfNotificationEvent()  \
                                 do                                        \
                                 {                                         \
                                    gTaskEventFlags |= gCTSelf_EVENT_c;    \
                                    mainTask->signal_set(gEventsAny_c);    \
                                 }while(0);
#define ResetMCU()  NVIC_SystemReset()

#define gUART_RX_EVENT_c         (1<<0)
#define gMcps_Cnf_EVENT_c        (1<<1)
#define gMcps_Ind_EVENT_c        (1<<2)
#define gMlme_EdCnf_EVENT_c      (1<<3)
#define gMlme_CcaCnf_EVENT_c     (1<<4)
#define gMlme_TimeoutInd_EVENT_c (1<<5)
#define gRangeTest_EVENT_c       (1<<6)
#define gCTSelf_EVENT_c          (1<<7)
#define gTimePassed_EVENT_c      (1<<8)

#define gEventsAny_c             (1<<9)

#define Delay_ms(a)        
#define FlaggedDelay_ms(a)       ConnTestTimeout.attach_us(DelayTimeElapsed, (a) * 1000)

#ifdef gPHY_802_15_4g_d
#define GetTimestampUS() PhyTime_GetTimestampUs()
#define GetTransmissionTime(payload, bitrate) ((((gPhyFSKPreambleLength_c + \
                                                  gPhyMRFSKPHRLength_c + gPhyMRFSKSFDLength_c + \
                                                    sizeof(smacHeader_t) + payload +  gPhyFCSSize_c )*8000 )/ bitrate))
#else
#define GetTimestampUS() (16*PhyTime_GetTimestamp())
#define GetTransmissionTime(payload, bitrate) (((6 + sizeof(smacHeader_t) + payload + 2)*32))
//bitrate is fixed for 2.4 GHz
#define crtBitrate      (0)
#endif
/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

uint32_t       gTaskEventFlags;

/*smac related variables*/
bool_t bTxDone;
bool_t bRxDone;
bool_t bScanDone;
bool_t gCCaGotResult;
bool_t gIsChannelIdle;
bool_t bEdDone;
bool_t failedPRBS9;
uint8_t u8LastRxRssiValue;
bool_t evTestParameters;
uint8_t au8ScanResults[129];

/*serial manager related variables*/
uint8_t gu8UartData;
bool_t evDataFromUART;
uint8_t mAppSer;

/*connectivity test state machine variables*/
operationModes_t testOpMode;                                                    
operationModes_t prevOpMode; 

channels_t       testChannel;
uint8_t          testPower;
uint8_t          testPayloadLen;
uint8_t          contTxModBitValue;
uint8_t          ccaThresh;
bool_t shortCutsEnabled;
ConnectivityStates_t       connState;
ContinuousTxRxTestStates_t cTxRxState;
PerTxStates_t              perTxState;
PerRxStates_t              perRxState;
RangeTxStates_t            rangeTxState;
RangeRxStates_t            rangeRxState;
EditRegsStates_t    eRState; 
oRStates_t          oRState;
rRStates_t          rRState;
dRStates_t          dRState;
CSenseTCtrlStates_t   cstcState;
uint8_t ChannelToScan; 
smacTestMode_t contTestRunning;

/*asp related variables*/
AppToAspMessage_t aspTestRequestMsg;

extern uint8_t u8Prbs9Buffer[gPrbs9BufferLength_c];

Serial uart(USBTX,USBRX);
Thread *mainTask;

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static uint8_t gau8RxDataBuffer[gMaxSmacSDULength_c  + sizeof(txPacket_t)];                         
static uint8_t gau8TxDataBuffer[gMaxSmacSDULength_c  + sizeof(rxPacket_t)];                        

static txPacket_t * gAppTxPacket;
static rxPacket_t * gAppRxPacket;

static uint8_t timePassed;
Timeout RangeTestTmr;
Timeout ConnTestTimeout;
Timer AppDelayTmr;
/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
#if CT_Feature_Calibration
extern void StoreTrimValueToFlash (uint32_t trimValue, CalibrationOptionSelect_t option);
#endif

/*platform independent functions*/
static void SerialUIStateMachine(void);
static bool_t SerialContinuousTxRxTest(void);
static bool_t PacketErrorRateTx(void);
static bool_t PacketErrorRateRx(void);
static void SetRadioRxOnNoTimeOut(void);
static void HandleEvents(int32_t evSignals);
static void PrintTestParameters(bool_t bEraseLine);

static void PrintPerRxFinalLine(uint16_t u16Received, uint16_t u16Total);
extern uint32_t HexString2Dec(uint8_t * au8String);
static bool_t stringComp(uint8_t * au8leftString, uint8_t * au8RightString, uint8_t bytesToCompare);
/********************************/

static void RangeTest_Timer_CallBack ();
static bool_t RangeTx(void);
static bool_t RangeRx(void);

static bool_t EditRegisters(void);
#if CT_Feature_Direct_Registers || CT_Feature_Indirect_Registers
bool_t OverrideRegisters(void);
bool_t ReadRegisters(void);
bool_t DumpRegisters(void);
bool_t bIsRegisterDirect = TRUE;
#endif

static bool_t CSenseAndTCtrl(void);
static void TransmissionControlHandler(void);
static void CarrierSenseHandler(void);
static smacErrors_t TestMode ( smacTestMode_t  mode);
static void PacketHandler_Prbs9(void);
static void DelayTimeElapsed();
static void IncrementChannelOnEdEvent();
extern void ReadRFRegs(registerAddressSize_t, registerAddressSize_t);
extern void PrintTestParameters(bool_t bEraseLine);

/*************************************/
/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
void InitProject(void);
void InitSmac(void);
void main_task(void const *argument);
void UartRxCallBack(void);
void PrintMenu(char * const pu8Menu[], uint8_t port);

/************************************************************************************
*
* InitProject
*
************************************************************************************/
void InitProject(void)
{   
  /*Global Data init*/
  testPayloadLen = gMaxSmacSDULength_c;
  
  testOpMode       = gDefaultOperationMode_c;
  testChannel      = gDefaultChannelNumber_c;
  testPower        = gDefaultOutputPower_c;
  testPayloadLen   = gDefaultPayload_c;
  contTestRunning  = gTestModeForceIdle_c;
  shortCutsEnabled = FALSE; 
  connState        = gConnInitState_c;
  cTxRxState       = gCTxRxStateInit_c;
  perTxState       = gPerTxStateInit_c;
  perRxState       = gPerRxStateInit_c;
  rangeTxState     = gRangeTxStateInit_c;
  rangeRxState     = gRangeRxStateInit_c;
  prevOpMode       = gDefaultOperationMode_c;
  oRState          = gORStateInit_c;
  rRState          = gRRStateInit_c;
  dRState          = gDRStateInit_c;
  ccaThresh        = gDefaultCCAThreshold_c;
  bEdDone          = FALSE;
  evDataFromUART = FALSE; 
  
  InitProject_custom();
}

/************************************************************************************
*************************************************************************************
* SAP functions
*************************************************************************************
************************************************************************************/

//(Management) Sap handler for managing timeout indication and ED confirm
smacErrors_t smacToAppMlmeSap(smacToAppMlmeMessage_t* pMsg, instanceId_t instance)
{
  switch(pMsg->msgType)
  {
  case gMlmeEdCnf_c:
    au8ScanResults[pMsg->msgData.edCnf.scannedChannel] = pMsg->msgData.edCnf.energyLeveldB;
    gTaskEventFlags |= gMlme_EdCnf_EVENT_c;
    mainTask->signal_set(gEventsAny_c);
    break;
  case gMlmeCcaCnf_c:
    gTaskEventFlags |= gMlme_CcaCnf_EVENT_c;
    mainTask->signal_set(gEventsAny_c);
    if(pMsg->msgData.ccaCnf.status == gErrorNoError_c)
      gIsChannelIdle = TRUE;
    else
      gIsChannelIdle = FALSE;
    break;
  case gMlmeTimeoutInd_c:
    gTaskEventFlags |= gMlme_TimeoutInd_EVENT_c;
    mainTask->signal_set(gEventsAny_c);
    break;
  default:
    break;
  }
  MEM_BufferFree(pMsg);
  return gErrorNoError_c;
}
//(Data) Sap handler for managing data confirm and data indication
smacErrors_t smacToAppMcpsSap(smacToAppDataMessage_t* pMsg, instanceId_t instance)
{
  switch(pMsg->msgType)
  {
  case gMcpsDataInd_c:
    if(pMsg->msgData.dataInd.pRxPacket->rxStatus == rxSuccessStatus_c)
    {
      u8LastRxRssiValue = pMsg->msgData.dataInd.u8LastRxRssi;
      gTaskEventFlags |= gMcps_Ind_EVENT_c;
      mainTask->signal_set(gEventsAny_c);
    }
    break;
  case gMcpsDataCnf_c:
    if(pMsg->msgData.dataCnf.status == gErrorNoError_c)
    {
      gTaskEventFlags |= gMcps_Cnf_EVENT_c;
      mainTask->signal_set(gEventsAny_c);      
    }
    break;
  default:
    break;
  }
  
  MEM_BufferFree(pMsg);
  return gErrorNoError_c;
}

static void HandleEvents(int32_t evSignals)
{
  if(evSignals & gUART_RX_EVENT_c)
  {
    if(shortCutsEnabled)
    {
      ShortCutsParser(gu8UartData);  
    }
    else
    {
      evDataFromUART = TRUE;
    }
  }
  if(evSignals & gMcps_Cnf_EVENT_c)
  {
    bTxDone = TRUE;
  }
  if(evSignals & gMcps_Ind_EVENT_c)
  {
    bRxDone = TRUE;
  }
  if(evSignals & gMlme_TimeoutInd_EVENT_c)
  {
  }
  if(evSignals & gRangeTest_EVENT_c)
  {
    bRxDone=TRUE; 
  }
  if(evSignals & gMlme_EdCnf_EVENT_c)
  {
    if (cTxRxState == gCTxRxStateRunnigScanTest_c)
    {
      IncrementChannelOnEdEvent();
    }
    if (cTxRxState == gCTxRxStateRunnigEdTest_c)
    {
      cTxRxState = gCTxRxStateRunningEdTestGotResult_c;
    }
    if (connState == gConnCSenseAndTCtrl_c)
    {
      bScanDone = TRUE;
    }
    bEdDone = TRUE;
  }
  if(evSignals & gMlme_CcaCnf_EVENT_c)
  {
    gCCaGotResult = TRUE;
    uart.printf("Channel %d is", (uint32_t)testChannel);
    if(gIsChannelIdle)
      uart.printf("Idle\r\n");
    else
      uart.printf("Busy\r\n");
  }
  if(evSignals & gCTSelf_EVENT_c)
  {
  }
}


/*************************************************************************/
/*Main Task: Application entry point*/
/*************************************************************************/
void main_task(void const *argument)
{
  static bool_t bIsInitialized = FALSE;
  static bool_t bUserInteraction = FALSE;
  //Initialize Memory Manager, Timer Manager and LEDs.
  if( !bIsInitialized )
  {
    
    MEM_Init();
    //initialize PHY
    Phy_Init();

    InitApp();
    
    uart.attach(&UartRxCallBack);
    
    /*Prints the Welcome screens in the terminal*/  
    PrintMenu(cu8FreescaleLogo, mAppSer);
  
    connState = gConnIdleState_c; 
    bIsInitialized = TRUE;
  }
  if(!bUserInteraction)
  {
    while(1)
    { 
      Thread::signal_wait(gEventsAny_c);
      if(gu8UartData == '\r')
      {
        SelfNotificationEvent();
        bUserInteraction = TRUE;
        break;
      }
      else
      {
        PrintMenu(cu8FreescaleLogo, mAppSer);
      }
    }
  }
  if(bUserInteraction)
  {
    while(1)
    {
      Thread::signal_wait(gEventsAny_c);
      HandleEvents(gTaskEventFlags);
      SerialUIStateMachine();  
    }
  } 
}

/*************************************************************************/
/*InitApp: Initializes application mdoules and data*/
/*************************************************************************/
void InitApp()
{
  
  gAppTxPacket = (txPacket_t*)gau8TxDataBuffer;   //Map TX packet to buffer
  gAppRxPacket = (rxPacket_t*)gau8RxDataBuffer;   //Map Rx packet to buffer     
  gAppRxPacket->u8MaxDataLength = gMaxSmacSDULength_c;
  
  uart.baud(115200);
    
  //Initialise SMAC
  InitSmac();
  //Tell SMAC who to call when it needs to pass a message to the application thread.
  Smac_RegisterSapHandlers((SMAC_APP_MCPS_SapHandler_t)smacToAppMcpsSap,(SMAC_APP_MLME_SapHandler_t)smacToAppMlmeSap,0);
  
  InitProject();
  
  InitApp_custom();
  
  ASP_Init(0, mAppSer);
  
  SMACFillHeader(&(gAppTxPacket->smacHeader), gBroadcastAddress_c);                   //@CMA, Conn Test. Start with broadcast address default
  (void)MLMEPAOutputAdjust(testPower);
  (void)MLMESetChannelRequest(testChannel);                                     //@CMA, Conn Test. Start Foperation at default channel
}

/************************************************************************************
*
* Connectivity Test State Machine
*
************************************************************************************/
void SerialUIStateMachine(void)
{
  if((gConnSelectTest_c == connState) && evTestParameters)
  {
#if CT_Feature_Calibration
    (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
    (void)MLMESetChannelRequest(testChannel);   
    (void)MLMEPAOutputAdjust(testPower);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  switch(connState)
  {
  case gConnIdleState_c:
    PrintMenu(cu8MainMenu, mAppSer);
    PrintTestParameters(FALSE);
    shortCutsEnabled = TRUE;           
    connState = gConnSelectTest_c;
    break;
  case gConnSelectTest_c:
    if(evDataFromUART){
      if('1' == gu8UartData)
      {
        cTxRxState = gCTxRxStateInit_c;
        connState = gConnContinuousTxRxState_c;
      }
      else if('2' == gu8UartData)
      {
        perTxState = gPerTxStateInit_c;
        perRxState = gPerRxStateInit_c;
        connState = gConnPerState_c;
      }
      else if('3' == gu8UartData)
      {
        rangeTxState = gRangeTxStateInit_c;
        rangeRxState = gRangeRxStateInit_c;
        connState = gConnRangeState_c;
      }
      else if('4' == gu8UartData)
      {
        cstcState = gCsTcStateInit_c;
        connState = gConnCSenseAndTCtrl_c;
      }
#if CT_Feature_Direct_Registers || CT_Feature_Indirect_Registers
      else if('5' == gu8UartData)
      {
        eRState = gERStateInit_c;
        connState = gConnRegEditState_c;
      }
#endif
#if CT_Feature_Bitrate_Select
      else if('6' == gu8UartData)
      {
        bsState = gBSStateInit_c;
        connState = gConnBitrateSelectState_c;
      }
#endif
#if CT_Feature_Calibration
      else if('7' == gu8UartData)
      {
        connState = gConnEDMeasCalib_c;
        edCalState= gEdCalStateInit_c;
      }
#endif
      else if('!' == gu8UartData)
      {
        ResetMCU();
      }
      evDataFromUART = FALSE;
      SelfNotificationEvent();
    }
    break;
  case gConnContinuousTxRxState_c:
    if(SerialContinuousTxRxTest()) 
    {
      connState = gConnIdleState_c;
      SelfNotificationEvent();
    }
    break;
  case gConnPerState_c:
    if(mTxOperation_c == testOpMode)
    {
      if(PacketErrorRateTx())
      {
        connState = gConnIdleState_c;
        SelfNotificationEvent();
      }
    }
    else
    {
      if(PacketErrorRateRx())
      {
        connState = gConnIdleState_c;
        SelfNotificationEvent();
      }
    }
    break;
  case gConnRangeState_c:
    if(mTxOperation_c == testOpMode)
    {
      if(RangeTx())
      {
        connState = gConnIdleState_c;
        SelfNotificationEvent();
      }
    }
    else
    {
      if(RangeRx())
      {
        connState = gConnIdleState_c;
        SelfNotificationEvent();
      }
    }
    break;
  case gConnRegEditState_c:
    if(EditRegisters()) 
    {
      connState = gConnIdleState_c;
      SelfNotificationEvent();
    }
    break;
#if CT_Feature_Bitrate_Select
  case gConnBitrateSelectState_c:
    if(Bitrate_Select()) 
    {
      connState = gConnIdleState_c;
    }
    break;
#endif
  case gConnCSenseAndTCtrl_c:
    if(CSenseAndTCtrl()) 
    {
      connState = gConnIdleState_c;
      SelfNotificationEvent();
    }
    break;
#if CT_Feature_Calibration
  case gConnEDMeasCalib_c:
    if(EDCalibrationMeasurement())
    {
      connState = gConnIdleState_c;
      SelfNotificationEvent();
    }
    break;
#endif
  default:
    break;
    
  }
  if(prevOpMode != testOpMode)
  {
    perTxState = gPerTxStateInit_c;
    perRxState = gPerRxStateInit_c;
    rangeTxState = gRangeTxStateInit_c;
    rangeRxState = gRangeRxStateInit_c;
    prevOpMode = testOpMode;
    SelfNotificationEvent();
  }
}

/************************************************************************************
*
* Continuous Tests State Machine
*
************************************************************************************/
bool_t SerialContinuousTxRxTest(void)
{
  bool_t bBackFlag = FALSE;
  uint8_t u8Index, u8TempEnergyValue, u8TempScanValue;
  
  if(evTestParameters)
  {
    (void)TestMode(gTestModeForceIdle_c);
#if CT_Feature_Calibration
    (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
    (void)MLMESetChannelRequest(testChannel);                                 
    (void)MLMEPAOutputAdjust(testPower);
    
    if(gTestModePRBS9_c == contTestRunning)
    {
      cTxRxState = gCTxRxStateRunningPRBS9Test_c;
    } 
    (void)TestMode(contTestRunning);
    
    if(gCTxRxStateSelectTest_c == cTxRxState)
    {
      PrintTestParameters(TRUE);
    }
    else
    {
      PrintTestParameters(FALSE);
      uart.printf("\r\n");     
    }
    
    if(gCTxRxStateRunnigRxTest_c == cTxRxState)
    {
      bRxDone = FALSE;
      gAppRxPacket->u8MaxDataLength = gMaxSmacSDULength_c;
      (void)MLMERXEnableRequest(gAppRxPacket, 0);
    }
    evTestParameters = FALSE;
  }
  
  switch(cTxRxState)
  {
  case gCTxRxStateIdle_c:
    if((evDataFromUART) && ('\r' == gu8UartData))
    {
      cTxRxState = gCTxRxStateInit_c;
      evDataFromUART = FALSE;  
      SelfNotificationEvent();
    }
    break;
  case gCTxRxStateInit_c:
    PrintMenu(cu8ShortCutsBar, mAppSer);
    PrintMenu(cu8ContinuousTestMenu, mAppSer); 
    //Phy in StandBy, smacstate in Idle. 
    (void)TestMode(gTestModeForceIdle_c);  
    while(MLMESetChannelRequest(testChannel));                                   
    uart.printf(cu8ContinuousTestTags[contTestRunning]);
    if(contTestRunning == gTestModeContinuousTxModulated_c)
    {
      uart.printf(cu8TxModTestTags[contTxModBitValue]);
    }
    (void)TestMode(contTestRunning);
    uart.printf("\r\n\r\n");       
    PrintTestParameters(FALSE);
    shortCutsEnabled = TRUE;           
    cTxRxState = gCTxRxStateSelectTest_c; 
    break;
  case gCTxRxStateSelectTest_c:
    if(evDataFromUART)
    {           
      if('1' == gu8UartData)
      {
        contTestRunning = gTestModeForceIdle_c;              
        cTxRxState = gCTxRxStateInit_c;
        SelfNotificationEvent();
      }
      else if('2' == gu8UartData)
      {
        shortCutsEnabled = FALSE;
        (void)TestMode(gTestModeForceIdle_c);
        contTestRunning = gTestModePRBS9_c;  
        MLMESetChannelRequest(testChannel);      
        uart.printf("\f\r\nPress [p] to stop the Continuous PRBS9 test\r\n");
        (void)TestMode(contTestRunning);
        cTxRxState = gCTxRxStateRunningPRBS9Test_c;
      }
      else if('3' == gu8UartData)
      {
        contTestRunning = gTestModeContinuousTxModulated_c;               
        cTxRxState = gCTxRxStateRunningTXModSelectOpt;
        //        uart.printf( "\f\r\n To use this mode shunt pins 3-4 on J18");
        uart.printf("\f\r\nPress 2 for PN9, 1 to modulate values of 1 and 0 to modulate values of 0");
        
      }
      else if('4' == gu8UartData)
      {
        if(gTestModeContinuousTxUnmodulated_c != contTestRunning) 
        { 
          contTestRunning = gTestModeContinuousTxUnmodulated_c;               
          cTxRxState = gCTxRxStateInit_c;
          SelfNotificationEvent();
        }
      }
      else if('5' == gu8UartData)
      {
        shortCutsEnabled = FALSE;
        (void)TestMode(gTestModeForceIdle_c);  
        MLMESetChannelRequest(testChannel);
        contTestRunning = gTestModeForceIdle_c;
        uart.printf("\f\r\nPress [p] to stop receiving broadcast packets \r\n");                                               
        bRxDone = FALSE;
        gAppRxPacket->u8MaxDataLength = gMaxSmacSDULength_c;
        (void)MLMERXEnableRequest(gAppRxPacket, 0);
        cTxRxState = gCTxRxStateRunnigRxTest_c;
      }
      else if('6' == gu8UartData)
      {
        (void)TestMode(gTestModeForceIdle_c);
        contTestRunning = gTestModeForceIdle_c;
        uart.printf("\f\r\nPress [p] to stop the Continuous ED test\r\n");               
        cTxRxState = gCTxRxStateRunnigEdTest_c;
        FlaggedDelay_ms(200);
      }
      else if('7' == gu8UartData)
      {
        (void)TestMode(gTestModeForceIdle_c);
        contTestRunning = gTestModeForceIdle_c;
        ChannelToScan= gDefaultChannelNumber_c;                            
        uart.printf("\f\r\nPress [p] to stop the Continuous SCAN test\r\n");
        bScanDone = FALSE;
        cTxRxState = gCTxRxStateRunnigScanTest_c;
        SelfNotificationEvent();
      }
      else if('8' == gu8UartData)
      {
        (void)TestMode(gTestModeForceIdle_c);                        
        uart.printf("\f\r\nPress [p] to stop the Continuous CCA test\r\n");
        contTestRunning = gTestModeForceIdle_c;                
        cTxRxState = gCTxRxStateRunnigCcaTest_c;
        FlaggedDelay_ms(100);
        MLMECcaRequest();
      }
#if CT_Feature_BER_Test
      else if ('9' == gu8UartData)
      {
        uart.printf( "\f\r\nPress [p] to stop the Continuous BER test\r\n");
        contTestRunning = gTestModeContinuousRxBER_c;               
        cTxRxState = gCTxRxStateInit_c;
        SelfNotificationEvent();
      }
#endif
      else if('p' == gu8UartData)
      { 
        (void)TestMode(gTestModeForceIdle_c);
        (void)MLMESetChannelRequest(testChannel); 
        AppDelayTmr.stop();
        timePassed = FALSE;
        bBackFlag = TRUE;
      }
      evDataFromUART = FALSE;
    }
    break;
  case gCTxRxStateRunningTXModSelectOpt:
    if(evDataFromUART)
    { 
      if(gu8UartData == '2')
        contTxModBitValue = gContTxModSelectPN9_c;
      else
        if(gu8UartData == '1')
          contTxModBitValue = gContTxModSelectOnes_c;
        else
          if(gu8UartData == '0')
            contTxModBitValue = gContTxModSelectZeros_c;
        
        evDataFromUART = FALSE;
        cTxRxState = gCTxRxStateInit_c;
        SelfNotificationEvent();
    }
    break;
  case gCTxRxStateRunningPRBS9Test_c:
    if(bTxDone || failedPRBS9)
    {
      failedPRBS9 = FALSE;
      bTxDone     = FALSE;
      PacketHandler_Prbs9();
    }
    if(evDataFromUART && 'p' == gu8UartData)
    {
      contTestRunning = gTestModeForceIdle_c;
      (void)TestMode(gTestModeForceIdle_c);
      (void)MLMESetChannelRequest(testChannel); 
      AppDelayTmr.stop();
      timePassed = FALSE;
      uart.printf("\r\n\r\n Press [enter] to go back to the Continuous test menu ");
      cTxRxState = gCTxRxStateIdle_c;
      evDataFromUART = FALSE;
      shortCutsEnabled = TRUE;
    }
    break;
  case gCTxRxStateRunnigRxTest_c:
    if(bRxDone)
    {
      if (gAppRxPacket->rxStatus == rxSuccessStatus_c)
      {
        uart.printf("New Packet: ");
        for(u8Index = 0; u8Index < (gAppRxPacket->u8DataLength); u8Index++){
          uart.printf( (const char *)(&(gAppRxPacket->smacPdu.smacPdu[u8Index])));
        }
        uart.printf(" \r\n");
      }
      bRxDone = FALSE;
      gAppRxPacket->u8MaxDataLength = gMaxSmacSDULength_c;
      (void)MLMERXEnableRequest(gAppRxPacket, 0);
    }
    if((evDataFromUART) && ('p' == gu8UartData))
    {
      (void)MLMERXDisableRequest();
      (void)TestMode(gTestModeForceIdle_c);
      uart.printf("\r\n\r\n Press [enter] to go back to the Continuous test menu ");
      cTxRxState = gCTxRxStateIdle_c;
      evDataFromUART = FALSE;
    }
    break;
  case gCTxRxStateRunnigEdTest_c:
    if(timePassed)
    {
      timePassed = FALSE;
      FlaggedDelay_ms(100);
      MLMEScanRequest(testChannel);
    }
    if((evDataFromUART) && ('p' == gu8UartData))
    {
      uart.printf("\r\n\r\n Press [enter] to go back to the Continuous test menu ");
      cTxRxState = gCTxRxStateIdle_c;
      evDataFromUART = FALSE;
      timePassed = FALSE;
      AppDelayTmr.stop();
    }
    
    break;
  case gCTxRxStateRunningEdTestGotResult_c:
    uart.printf("Energy on the Channel %d : ", (uint32_t)testChannel);
    u8TempEnergyValue = au8ScanResults[testChannel];
    if(u8TempEnergyValue != 0)
      uart.printf( "-");
    uart.printf("%d dBm\r\n ",(uint32_t)u8TempEnergyValue);      
    cTxRxState = gCTxRxStateRunnigEdTest_c;
    break; 
  case gCTxRxStateRunnigCcaTest_c:
    if(timePassed && gCCaGotResult)
    {
      gCCaGotResult = FALSE;
      timePassed = FALSE;
      MLMECcaRequest();
      FlaggedDelay_ms(100);
    }
    if((evDataFromUART) && ('p' == gu8UartData))
    {
      uart.printf("\r\n\r\n Press [enter] to go back to the Continuous test menu ");
      cTxRxState = gCTxRxStateIdle_c;
      evDataFromUART = FALSE;
      timePassed = FALSE;
      AppDelayTmr.stop();
    }
    break;
  case gCTxRxStateRunnigScanTest_c:
    if(bScanDone && timePassed)
    {                                              
      //Enters here until all channels have been scanned. Then starts to print.
      uart.printf("Results : ");
      for(u8Index = gMinChannel_c; u8Index <= gMaxChannel_c ; u8Index++)
      {                                                         
        u8TempScanValue= au8ScanResults[u8Index];
        if(u8TempScanValue != 0)
          uart.printf("-");
        uart.printf("%d, ", (uint32_t) u8TempScanValue);
      }
      uart.printf("\b \r\n");
      bScanDone = FALSE;                                                   
      ChannelToScan = gDefaultChannelNumber_c;                             // Restart channel count
      timePassed = FALSE;
    }                                                                         
    
    if((evDataFromUART) && ('p' == gu8UartData))
    {
      uart.printf("\r\n\r\n Press [enter] to go back to the Continuous test menu ");
      cTxRxState = gCTxRxStateIdle_c;
      evDataFromUART = FALSE;
    }
    else
    {
      if(ChannelToScan == gDefaultChannelNumber_c)
      {
        smacErrors_t err = MLMEScanRequest((channels_t)ChannelToScan);                                            
        if(err == gErrorNoError_c)
          ChannelToScan++;
      }
      //Each of the other channels is scanned after SMAC notifies us that 
      //it has obtained the energy value on the currently scanned channel 
      //(channel scanning is performed asynchronously). See IncrementChannelOnEdEvent().
    }
    break;
  default:
    break;
  }
  return bBackFlag;
}

/************************************************************************************
*
* PER Handler for board that is performing TX
*
************************************************************************************/
bool_t PacketErrorRateTx(void)
{
  const uint16_t u16TotalPacketsOptions[] = {1,25,100,500,1000,2000,5000,10000,65535};
  static uint16_t u16TotalPackets;
  static uint16_t u16SentPackets;
  static uint32_t miliSecDelay;
  static uint32_t u32MinDelay = 4;
  uint8_t u8Index;
  bool_t bBackFlag = FALSE;
  
  if(evTestParameters)
  {
    (void)MLMERXDisableRequest();
#if CT_Feature_Calibration
    (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
    (void)MLMESetChannelRequest(testChannel);                                 
    (void)MLMEPAOutputAdjust(testPower);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  
  switch(perTxState)
  {
  case gPerTxStateInit_c:
    PrintMenu(cu8ShortCutsBar, mAppSer);
    PrintMenu(cu8PerTxTestMenu, mAppSer);
    PrintTestParameters(FALSE);
    shortCutsEnabled = TRUE;           
    perTxState = gPerTxStateSelectPacketNum_c;
    miliSecDelay = 0;
    u32MinDelay = 4;
    (void)MLMERXDisableRequest();
    break;
  case gPerTxStateSelectPacketNum_c:
    if(evDataFromUART)
    {
      if((gu8UartData >= '0') && (gu8UartData <= '8'))
      {
        u16TotalPackets = u16TotalPacketsOptions[gu8UartData - '0'];
        shortCutsEnabled = FALSE;  
        u32MinDelay += (GetTransmissionTime(testPayloadLen, crtBitrate) / 1000);
        uart.printf("\r\n\r\n Please type TX interval in miliseconds ( > %d ms ) and press [ENTER]\r\n", u32MinDelay);
        perTxState = gPerTxStateInputPacketDelay_c;
      }
      else if('p' == gu8UartData)
      { 
        bBackFlag = TRUE;
      }
      evDataFromUART = FALSE;
    }
    break;
  case gPerTxStateInputPacketDelay_c:
    if(evDataFromUART)
    {
      if(gu8UartData == '\r')
      {
        if(miliSecDelay < u32MinDelay)
        {
          uart.printf("\r\n\tError: TX Interval too small\r\n");
          perTxState = gPerTxStateInit_c;
          SelfNotificationEvent();
        }
        else
        {
          perTxState = gPerTxStateStartTest_c;
          SelfNotificationEvent();
        }
      }
      else if((gu8UartData >= '0') && (gu8UartData <='9'))
      {
        miliSecDelay = miliSecDelay*10 + (gu8UartData - '0');
        uart.printf("%d", (uint32_t)(gu8UartData - '0'));
      }
      else if('p' == gu8UartData)
      { 
        perTxState = gPerTxStateInit_c;
        SelfNotificationEvent();
      }
      evDataFromUART = FALSE;
    }
    break;
  case gPerTxStateStartTest_c:
    gAppTxPacket->u8DataLength = testPayloadLen;
    u16SentPackets = 0;
    
    gAppTxPacket->smacPdu.smacPdu[0] = (u16TotalPackets >> 8);
    gAppTxPacket->smacPdu.smacPdu[1] = (uint8_t)u16TotalPackets;
    gAppTxPacket->smacPdu.smacPdu[2] = ((u16SentPackets+1) >> 8);
    gAppTxPacket->smacPdu.smacPdu[3] = (uint8_t)(u16SentPackets+1);
    FLib_MemCpy(&(gAppTxPacket->smacPdu.smacPdu[4]), (void*)"SMAC PER Demo",13);
    if(17 < testPayloadLen)
    {
      for(u8Index=17;u8Index<testPayloadLen;u8Index++)
      {     
        gAppTxPacket->smacPdu.smacPdu[u8Index] = (u8Index%10)+'0';            
      }
    }
    bTxDone = FALSE;
    (void)MCPSDataRequest(gAppTxPacket);
    u16SentPackets++;
    uart.printf("\f\r\n Running PER Tx, Sending %d Packets",(uint32_t)u16TotalPackets);
    perTxState = gPerTxStateRunningTest_c;
    FlaggedDelay_ms(miliSecDelay);
    break;
  case gPerTxStateRunningTest_c:
    if(bTxDone && timePassed)
    {
      uart.printf("\r\n Packet %d ",(uint32_t)u16SentPackets);
      if(u16SentPackets == u16TotalPackets)
      {    
        uart.printf("\r\n\r\nSending last %d  frames \r\n",(uint32_t)mTotalFinalFrames_c);
        FLib_MemCpy(&(gAppTxPacket->smacPdu.smacPdu[4]), (void *)"DONE",4);
        gAppTxPacket->u8DataLength = 8;
        u16SentPackets = 0;
        u16TotalPackets = mTotalFinalFrames_c;
        gAppTxPacket->u8DataLength = 8;
        perTxState = gPerTxStateSendingLastFrames_c;
      }
      else
      {
        gAppTxPacket->smacPdu.smacPdu[2] = ((u16SentPackets+1) >> 8);
        gAppTxPacket->smacPdu.smacPdu[3] = (uint8_t)(u16SentPackets+1);
        gAppTxPacket->u8DataLength = testPayloadLen;
      }
      bTxDone = FALSE;
      (void)MCPSDataRequest(gAppTxPacket);
      u16SentPackets++;
      timePassed = FALSE;
      FlaggedDelay_ms(miliSecDelay);
    }
    if(evDataFromUART && gu8UartData == ' ')
    {
      uart.printf("\r\n\r\n-Test interrupted by user. Press [ENTER] to continue\r\n\r\n");
      perTxState = gPerTxStateIdle_c;
    }
    break;
  case gPerTxStateSendingLastFrames_c:
    if(bTxDone && timePassed)
    {
      bTxDone = FALSE;
      timePassed = FALSE;
      uart.printf("\r\n Final Packet %d",(uint32_t)u16SentPackets);
      if(u16SentPackets == u16TotalPackets)
      {
        uart.printf( "\r\n PER Tx DONE \r\n");
        uart.printf( "\r\n\r\n Press [enter] to go back to the PER Tx test menu ");
        perTxState = gPerTxStateIdle_c;
        AppDelayTmr.stop();
        timePassed = FALSE;
      }
      else
      {
        gAppTxPacket->u8DataLength = 8;
        (void)MCPSDataRequest(gAppTxPacket);
        u16SentPackets++;
        FlaggedDelay_ms(miliSecDelay);
      } 
    }
    if(evDataFromUART && gu8UartData == ' ')
    {
      uart.printf("\r\n\r\n-Test interrupted by user. Press [ENTER] to continue\r\n\r\n");
      perTxState = gPerTxStateIdle_c;
    }
    break;  
  case gPerTxStateIdle_c:
    if((evDataFromUART) && ('\r' == gu8UartData))
    {
      perTxState = gPerTxStateInit_c;
      evDataFromUART = FALSE;
      SelfNotificationEvent();
    }
    break;
  default:
    break;
  }
  
  return bBackFlag;
}

/************************************************************************************
*
* PER Handler for board that is performing RX
*
************************************************************************************/
bool_t PacketErrorRateRx(void)
{
  static uint16_t u16ReceivedPackets;
  static uint16_t u16PacketsIndex;  
  static uint16_t u16TotalPackets;
  static uint16_t u16FinalPacketsCount;
  static uint32_t u32RssiSum;
  static uint8_t  u8AverageRssi;
  uint8_t u8TempRssivalue;
  
  bool_t bBackFlag = FALSE;
  if(evTestParameters)
  {
#if CT_Feature_Calibration
    (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
    (void)MLMESetChannelRequest(testChannel);                                 
    (void)MLMEPAOutputAdjust(testPower);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  switch(perRxState)
  {
  case gPerRxStateInit_c:
    u16TotalPackets = 0;
    u16ReceivedPackets = 0;
    u16PacketsIndex = 0;
    u32RssiSum = 0;
    PrintMenu(cu8ShortCutsBar, mAppSer);
    PrintMenu(cu8PerRxTestMenu, mAppSer);
    PrintTestParameters(FALSE);
    shortCutsEnabled = TRUE;           
    perRxState = gPerRxWaitStartTest_c;
    break;
  case gPerRxWaitStartTest_c:
    if(evDataFromUART)
    {
      if(' ' == gu8UartData)
      {
        uart.printf("\f\n\rPER Test Rx Running\r\n\r\n");
        SetRadioRxOnNoTimeOut();                                      
        shortCutsEnabled = FALSE;  
        perRxState = gPerRxStateStartTest_c;
      }
      else if('p' == gu8UartData)
      { 
        bBackFlag = TRUE;
      }
      evDataFromUART = FALSE;
    }
    break;
  case gPerRxStateStartTest_c:
    if(bRxDone)
    {
      if (gAppRxPacket->rxStatus == rxSuccessStatus_c)
      {
        if(stringComp((uint8_t*)"SMAC PER Demo",&gAppRxPacket->smacPdu.smacPdu[4],13))
        {
          u16TotalPackets = ((uint16_t)gAppRxPacket->smacPdu.smacPdu[0] <<8) + gAppRxPacket->smacPdu.smacPdu[1];
          u16PacketsIndex = ((uint16_t)gAppRxPacket->smacPdu.smacPdu[2] <<8) + gAppRxPacket->smacPdu.smacPdu[3];
          u16ReceivedPackets++;
          u8TempRssivalue= u8LastRxRssiValue;                               //@CMA, Conn Test. New line
          u32RssiSum += u8TempRssivalue;
          u8AverageRssi = (uint8_t)(u32RssiSum/u16ReceivedPackets);
          uart.printf("Packet %d . Packet index: %d . Rssi during RX: - %d\r\n",(uint32_t)u16ReceivedPackets, (uint32_t)u16PacketsIndex, (uint32_t)u8LastRxRssiValue);
          if(u16PacketsIndex == u16TotalPackets)
          {
            u16FinalPacketsCount = 0; 
            perRxState = gPerRxStateReceivingLastFrames_c;
          }
        }
        else if(stringComp((uint8_t*)"DONE",&gAppRxPacket->smacPdu.smacPdu[4],4))
        {
          u16FinalPacketsCount = 0; 
          perRxState = gPerRxStateReceivingLastFrames_c;
        }
      }
      else
      { 
        if(u16TotalPackets)
        {
          u16PacketsIndex++;
          if(u16PacketsIndex == u16TotalPackets)
          {
            u16FinalPacketsCount = 0; 
            perRxState = gPerRxStateReceivingLastFrames_c;
          }
        }
      }
      
      SetRadioRxOnNoTimeOut();
    }
    if(evDataFromUART)
    {
      if(' ' == gu8UartData)
      {
        (void)MLMERXDisableRequest();
        uart.printf("\r\nAverage Rssi during PER: -");
        uart.printf("%d",(uint32_t)u8AverageRssi);
        uart.printf(" dBm\r\n");
        uart.printf( "\n\rPER Test Rx Stopped\r\n\r\n");
        PrintPerRxFinalLine(u16ReceivedPackets,u16TotalPackets);
        perRxState = gPerRxStateIdle_c;
      } 
      evDataFromUART = FALSE;
    }         
    break;
  case gPerRxStateReceivingLastFrames_c:
    if(bRxDone)
    {
      u16FinalPacketsCount++; 
      if(mTotalFinalFrames_c == u16FinalPacketsCount)
      {
        uart.printf("\r\nAverage Rssi during PER: -");
        uart.printf("%d",(uint32_t)u8AverageRssi);
        uart.printf(" dBm\r\n");
        uart.printf( "\n\rPER Test Finished\r\n\r\n");
        PrintPerRxFinalLine(u16ReceivedPackets,u16TotalPackets);              
        perRxState = gPerRxStateIdle_c;
      }
      else
      {  
        SetRadioRxOnNoTimeOut();
      }
    }
    if(evDataFromUART)
    {
      if(' ' == gu8UartData)
      {
        (void)MLMERXDisableRequest();
        uart.printf("\r\nAverage Rssi during PER: -");
        uart.printf("%d",(uint32_t)u8AverageRssi);
        uart.printf(" dBm\r\n");
        uart.printf( "\n\rPER Test Rx Stopped\r\n\r\n");
        PrintPerRxFinalLine(u16ReceivedPackets,u16TotalPackets);
        perRxState = gPerRxStateIdle_c;
      } 
      evDataFromUART = FALSE;
    } 
    break;
  case gPerRxStateIdle_c:
    if((evDataFromUART) && ('\r' == gu8UartData))
    {
      perRxState = gPerRxStateInit_c;
      SelfNotificationEvent();
    }
    evDataFromUART = FALSE;
    break;
  default:
    break;
  }
  return bBackFlag;
}

/************************************************************************************
*
* Range Test Handler for board that is performing TX
*
************************************************************************************/
bool_t RangeTx(void)
{
  bool_t bBackFlag = FALSE;
  static uint32_t u32RSSISum;
  static uint16_t u16ReceivedPackets;
  static uint16_t u16PacketsDropped;
  uint8_t  u8AverageRSSI;
  uint8_t  u8CurrentRSSI;
  
  if(evTestParameters)
  {
#if CT_Feature_Calibration
    (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
    (void)MLMESetChannelRequest(testChannel);                               
    (void)MLMEPAOutputAdjust(testPower);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  
  switch(rangeTxState)
  {
  case gRangeTxStateInit_c:
    u32RSSISum = 0;
    u16ReceivedPackets = 0;
    u16PacketsDropped = 0;
    PrintMenu(cu8ShortCutsBar, mAppSer);
    PrintMenu(cu8RangeTxTestMenu, mAppSer);
    PrintTestParameters(FALSE);
    shortCutsEnabled = TRUE;           
    rangeTxState = gRangeTxWaitStartTest_c;
    break;
  case gRangeTxWaitStartTest_c:
    if(evDataFromUART)
    {
      if(' ' == gu8UartData)
      {
        shortCutsEnabled = FALSE; 
        uart.printf( "\f\r\nRange Test Tx Running\r\n");
        rangeTxState = gRangeTxStateStartTest_c;
        FlaggedDelay_ms(200);
      }
      else if('p' == gu8UartData)
      { 
        bBackFlag = TRUE;
      }
      evDataFromUART = FALSE;
    }
    break;
  case gRangeTxStateStartTest_c:
    if(!timePassed) //waiting 200 ms
      break;
    timePassed = FALSE;
    bTxDone = FALSE;
    gAppTxPacket->u8DataLength = 16;
    gAppTxPacket->smacPdu.smacPdu[0]  = 0;
    FLib_MemCpy(&(gAppTxPacket->smacPdu.smacPdu[1]), (void*)"SMAC Range Demo",15);
    MLMERXDisableRequest();                                                
    //RangeTestTmr.stop();                                           //@CMA, Conn Test. Stop Rx timer
    (void)MCPSDataRequest(gAppTxPacket);
    rangeTxState = gRangeTxStateRunningTest_c;
    break;
  case gRangeTxStateRunningTest_c:
    if(bTxDone)
    {                                         
      RangeTestTmr.attach_us(RangeTest_Timer_CallBack, 80000);  //@CMA, Conn Test. Start Timer
      SetRadioRxOnNoTimeOut();
      rangeTxState = gRangeTxStatePrintTestResults_c;
    }
    break;
  case gRangeTxStatePrintTestResults_c:
    if(bRxDone)
    {                                                       
      if(gAppRxPacket->rxStatus == rxSuccessStatus_c)
      { 
        if(stringComp((uint8_t*)"SMAC Range Demo",&gAppRxPacket->smacPdu.smacPdu[1],15))
        {
          u8CurrentRSSI = (gAppRxPacket->smacPdu.smacPdu[0]); 
          u32RSSISum += u8CurrentRSSI;  
          u16ReceivedPackets++;
          u8AverageRSSI = (uint8_t)(u32RSSISum/u16ReceivedPackets);
          uart.printf( "\r\n RSSI = -");
          uart.printf("%d", (uint32_t)u8CurrentRSSI);   
          uart.printf(" dBm");
        }
        else
        {                                   
          RangeTestTmr.attach_us(RangeTest_Timer_CallBack, 80000);    //@CMA, Conn Test. Start Timer 
          SetRadioRxOnNoTimeOut();                                             
        }
      }
      else
      {
        u16PacketsDropped++;
        uart.printf( "\r\nPacket Dropped");
        bRxDone= FALSE;                                                  //@CMA, Conn Test. Added
      }
      if(evDataFromUART && (' ' == gu8UartData))
      {
        uart.printf( "\n\r\n\rRange Test Tx Stopped\r\n\r\n");
        u8AverageRSSI = (uint8_t)(u32RSSISum/u16ReceivedPackets);
        uart.printf( "Average RSSI     -");
        uart.printf("%d", (uint32_t)u8AverageRSSI); 
        uart.printf(" dBm");
        uart.printf( "\r\nPackets dropped ");
        uart.printf("%d", (uint32_t)u16PacketsDropped);  
        uart.printf( "\r\n\r\n Press [enter] to go back to the Range Tx test menu");
        rangeTxState = gRangeTxStateIdle_c;
        (void)MLMERXDisableRequest();
        AppDelayTmr.stop();
        timePassed = FALSE;
      }
      else
      {
        rangeTxState = gRangeTxStateStartTest_c;
        FlaggedDelay_ms(200);
      }
      evDataFromUART = FALSE;
    } 
    
    break;
  case gRangeTxStateIdle_c:
    if((evDataFromUART) && ('\r' == gu8UartData))
    {
      rangeTxState = gRangeTxStateInit_c;
      SelfNotificationEvent();
    }
    evDataFromUART = FALSE;
    break;
  default:
    break;
  }
  return bBackFlag;
}

/************************************************************************************
*
* Range Test Handler for board that is performing RX
*
************************************************************************************/
bool_t RangeRx(void)
{
  bool_t bBackFlag = FALSE;
  static uint32_t u32RSSISum;
  static uint16_t u16ReceivedPackets;
  uint8_t  u8AverageRSSI, u8TempRSSIvalue;
  uint8_t  u8CurrentRSSI;
  
  if(evTestParameters)
  {
#if CT_Feature_Calibration
    (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
    (void)MLMESetChannelRequest(testChannel);                                
    (void)MLMEPAOutputAdjust(testPower);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  
  switch(rangeRxState)
  {
  case gRangeRxStateInit_c:
    u32RSSISum = 0;
    u16ReceivedPackets = 0;
    PrintMenu(cu8ShortCutsBar, mAppSer);
    PrintMenu(cu8RangeRxTestMenu, mAppSer);
    PrintTestParameters(FALSE);
    shortCutsEnabled = TRUE;           
    rangeRxState = gRangeRxWaitStartTest_c;
    break;
  case gRangeRxWaitStartTest_c:
    if(evDataFromUART)
    {
      if(' ' == gu8UartData)
      {
        shortCutsEnabled = FALSE; 
        uart.printf( "\f\r\nRange Test Rx Running\r\n");
        rangeRxState = gRangeRxStateStartTest_c;
      }
      else if('p' == gu8UartData)
      { 
        bBackFlag = TRUE;
      }
      evDataFromUART = FALSE;
      SelfNotificationEvent();
    }
    break;
  case gRangeRxStateStartTest_c:
    SetRadioRxOnNoTimeOut();
    rangeRxState = gRangeRxStateRunningTest_c;
    break;
  case gRangeRxStateRunningTest_c:
    if(evDataFromUART && (' ' == gu8UartData))
    {             
      (void)MLMERXDisableRequest();
      uart.printf( "\n\r\n\rRange Test Rx Stopped\r\n\r\n");
      u8AverageRSSI = (uint8_t)(u32RSSISum/u16ReceivedPackets);
      uart.printf( "Average RSSI     ");
      if(u8AverageRSSI != 0)
      {
        uart.printf("-");
      }
      uart.printf("%d", (uint32_t)u8AverageRSSI);  
      uart.printf(" dBm");
      uart.printf( "\r\n\r\n Press [enter] to go back to the Range Rx test menu");
      rangeRxState = gRangeRxStateIdle_c;
    }
    evDataFromUART = FALSE;
    if(bRxDone)
    {
      if(gAppRxPacket->rxStatus == rxSuccessStatus_c)
      { 
        if(stringComp((uint8_t*)"SMAC Range Demo",&gAppRxPacket->smacPdu.smacPdu[1],15))
        {
          bRxDone = FALSE;
          FlaggedDelay_ms(4);
        }
        else
        {
          SetRadioRxOnNoTimeOut();
        }
      }
      else
      {
        SetRadioRxOnNoTimeOut();
      }
    }
    if(timePassed)
    {
      timePassed = FALSE;
      bTxDone = FALSE;
      u8TempRSSIvalue= u8LastRxRssiValue;                                      
      gAppTxPacket->smacPdu.smacPdu[0] = u8TempRSSIvalue;
      FLib_MemCpy(&(gAppTxPacket->smacPdu.smacPdu[1]), (void *)"SMAC Range Demo",15);
      gAppTxPacket->u8DataLength = 16;
      (void)MCPSDataRequest(gAppTxPacket);
      rangeRxState = gRangeRxStatePrintTestResults_c;
    }
    break;
  case gRangeRxStatePrintTestResults_c:
    if(bTxDone)
    {       
      u8CurrentRSSI= u8LastRxRssiValue;                                   
      u32RSSISum += u8CurrentRSSI;
      u16ReceivedPackets++;
      u8AverageRSSI = (uint8_t)(u32RSSISum/u16ReceivedPackets);
      uart.printf( "\r\n RSSI = -");
      uart.printf("%d", (uint32_t)u8CurrentRSSI);   
      uart.printf(" dBm");
      rangeRxState = gRangeRxStateStartTest_c;
      SelfNotificationEvent();
    }
    break;
  case gRangeRxStateIdle_c:
    if((evDataFromUART) && ('\r' == gu8UartData))
    {
      rangeRxState = gRangeRxStateInit_c;
      SelfNotificationEvent();
    }
    evDataFromUART = FALSE;
    break;
  default:
    break;
  }
  return bBackFlag;
}

/************************************************************************************
*
* Handler for viewing/modifying XCVR registers
*
************************************************************************************/
bool_t EditRegisters(void)
{
  bool_t bBackFlag = FALSE;
  if(evTestParameters)
  {
#if CT_Feature_Calibration
    (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
    (void)MLMESetChannelRequest(testChannel);                                
    (void)MLMEPAOutputAdjust(testPower);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  
  switch(eRState)
  {
  case gERStateInit_c:
    PrintMenu(cu8ShortCutsBar, mAppSer);
    PrintMenu(cu8RadioRegistersEditMenu, mAppSer);
    PrintTestParameters(FALSE);
    shortCutsEnabled = TRUE;           
    eRState = gERWaitSelection_c;
    break;
  case gERWaitSelection_c:
    if(evDataFromUART)
    {
#if CT_Feature_Direct_Registers
      if('1' == gu8UartData)
      {
        bIsRegisterDirect = TRUE;
        oRState = gORStateInit_c;
        eRState = gERStateOverrideReg_c;
        SelfNotificationEvent();
      }
      else if('2' == gu8UartData)
      {
        bIsRegisterDirect = TRUE;
        rRState = gRRStateInit_c;
        eRState = gERStateReadReg_c;
        SelfNotificationEvent();
      }
#if CT_Feature_Indirect_Registers
      else if('3' == gu8UartData)
      {
        bIsRegisterDirect = FALSE;
        oRState = gORStateInit_c;
        eRState = gERStateOverrideReg_c;
        SelfNotificationEvent();
      }
      else if('4' == gu8UartData)
      {
        bIsRegisterDirect = FALSE;
        rRState = gRRStateInit_c;
        eRState = gERStateReadReg_c;
        SelfNotificationEvent();
      }
      else if('5' == gu8UartData)
      {
        dRState = gDRStateInit_c;
        eRState  = gERStateDumpAllRegs_c;
        SelfNotificationEvent();
      }
#else
      else if('3' == gu8UartData)
      {
        dRState = gDRStateInit_c;
        eRState  = gERStateDumpAllRegs_c;
        SelfNotificationEvent();
      }
#endif
      else
#endif
      if('p' == gu8UartData)
      { 
        bBackFlag = TRUE;
      }
      evDataFromUART = FALSE;
    }
    break;
  case gERStateOverrideReg_c:
    if(OverrideRegisters()) 
    {
      eRState = gERStateInit_c;
      SelfNotificationEvent();
    }    
    break;
  case gERStateReadReg_c:
    if(ReadRegisters()) 
    {
      eRState = gERStateInit_c;
      SelfNotificationEvent();
    }    
    break;
  case gERStateDumpAllRegs_c:
    if(DumpRegisters()) {
      eRState = gERStateInit_c;
      SelfNotificationEvent();
    }
    break;
  default:
    break;
  }
  return bBackFlag;
}

/************************************************************************************
*
* Dump registers
*
************************************************************************************/
bool_t DumpRegisters(void)
{
  bool_t bBackFlag = FALSE;
  
  switch(dRState)
  {
  case gDRStateInit_c:
    uart.printf( "\f\r\rDump Registers\r\n");   
    uart.printf( "\r\n-Press [space] to dump registers\r\n");
    uart.printf( "\r\n-Press [p] Previous Menu\r\n");
    shortCutsEnabled = FALSE;   
    dRState = gDRStateDumpRegs_c;
    SelfNotificationEvent();
    break;
  case gDRStateDumpRegs_c:
    if(evDataFromUART){
      if(gu8UartData == 'p') 
      {
        bBackFlag = TRUE;
      }
      else if (gu8UartData == ' ') 
      {
        uart.printf( "\r\n -Dumping registers... \r\n");
        const registerLimits_t* interval = registerIntervals;
        
        while(!((*interval).regStart == 0 && (*interval).regEnd == 0))
        {
          uart.printf( "\r\n -Access type: ");
          if( (*interval).bIsRegisterDirect )
            uart.printf("direct\r\n");
          else
            uart.printf("indirect\r\n");
          bIsRegisterDirect = (*interval).bIsRegisterDirect;
          ReadRFRegs((*interval).regStart, (*interval).regEnd);
          interval++;
        }
        dRState = gDRStateInit_c;
        SelfNotificationEvent();
      }
    }
    evDataFromUART = FALSE;
    break;
  default:
    break;
  }
  return bBackFlag;  
}

/************************************************************************************
*
* Read and print register values with addresses from u8RegStartAddress 
* to u8RegStopAddress
*
************************************************************************************/
void ReadRFRegs(registerAddressSize_t rasRegStartAddress, registerAddressSize_t rasRegStopAddress)
{ 
  
  static uint16_t rasRegAddress; 
  registerSize_t rsRegValue; 
  uart.printf( " ---------------------------------------  "); 
  for(rasRegAddress = rasRegStartAddress; rasRegAddress <= rasRegStopAddress; rasRegAddress+=(gRegisterSize_c))
  { 
    uart.printf( "\r\n|    Address : 0x"); 
    uart.printf("%x", (uint8_t*)&rasRegAddress); 
    aspTestRequestMsg.msgType = aspMsgTypeXcvrReadReq_c;
    aspTestRequestMsg.msgData.aspXcvrData.addr = (uint16_t)rasRegAddress;
    aspTestRequestMsg.msgData.aspXcvrData.len  = gRegisterSize_c;
    aspTestRequestMsg.msgData.aspXcvrData.mode = !bIsRegisterDirect;
    
    APP_ASP_SapHandler(&aspTestRequestMsg, 0);
    rsRegValue = *((registerSize_t*)aspTestRequestMsg.msgData.aspXcvrData.data);              
    uart.printf( " Data value : 0x");                
    uart.printf("%x", (uint8_t*)&rsRegValue);  
    uart.printf( "   |");
  }    
  uart.printf( "\r\n ---------------------------------------  \r\n"); 
}

/************************************************************************************
*
* Read register
*
************************************************************************************/
bool_t ReadRegisters(void)
{
  bool_t bBackFlag = FALSE;
  static uint8_t au8RxString[5];
  static uint8_t u8Index;
  static registerAddressSize_t rasRegAddress;
  static registerSize_t rsRegValue;
  static char    auxToPrint[2];
  
  switch(rRState)
  {
  case gRRStateInit_c:
    uart.printf( "\f\r\rRead Registers\r\n");           
    uart.printf( "\r\n-Press [p] Previous Menu\r\n");
    shortCutsEnabled = FALSE;   
    rRState = gRRStateStart_c;
    SelfNotificationEvent();
    break;
  case gRRStateStart_c:
    uart.printf( "\r\n -write the Register address in Hex and [enter]: 0x");
    u8Index = 0;
    rRState = gRRWaitForTheAddress_c; 
    break;
  case gRRWaitForTheAddress_c:
    if(evDataFromUART)
    {
      if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData))
      {
        if('p' == gu8UartData)
        { 
          bBackFlag = TRUE;
        }
        else
        {
          uart.printf( "\r\n -Invalid Character!! ");
          rRState = gRRStateStart_c; 
          SelfNotificationEvent();
        }
      }
      else if((gRegisterAddressASCII_c == u8Index) && ('\r' != gu8UartData))
      { 
        uart.printf( "\r\n -Value out of Range!! ");
        rRState = gRRStateStart_c;
        SelfNotificationEvent();
      }
      else if(isAsciiHex(gu8UartData))
      {
        au8RxString[u8Index++] = gu8UartData;
        auxToPrint[0] = gu8UartData;
        auxToPrint[1] = '\0';
        uart.printf( auxToPrint);
      }
      else
      {
        au8RxString[u8Index] = 0;
        rasRegAddress = (registerAddressSize_t)HexString2Dec(au8RxString);
        aspTestRequestMsg.msgType = aspMsgTypeXcvrReadReq_c;
        aspTestRequestMsg.msgData.aspXcvrData.addr = (uint16_t)rasRegAddress;
        aspTestRequestMsg.msgData.aspXcvrData.len  = gRegisterSize_c;
        aspTestRequestMsg.msgData.aspXcvrData.mode = !bIsRegisterDirect;
        APP_ASP_SapHandler(&aspTestRequestMsg, 0);
        rsRegValue = *((registerSize_t*)aspTestRequestMsg.msgData.aspXcvrData.data);
        
        uart.printf( "\r\n -Register value : 0x");
        uart.printf("%x", (uint8_t*)&rsRegValue);
        uart.printf( "\r\n");
        
        rRState = gRRStateStart_c; 
        SelfNotificationEvent();
      }
      evDataFromUART = FALSE;
    }
    break;
  default:
    break;
  }
  return bBackFlag;  
}

/************************************************************************************
*
* Override Register
*
************************************************************************************/
bool_t OverrideRegisters(void)
{
  bool_t bBackFlag = FALSE;
  static uint8_t au8RxString[5];
  static uint8_t u8Index;
  static registerAddressSize_t rasRegAddress;
  static registerSize_t rsRegValue;
  static char auxToPrint[2];
  
  switch(oRState)
  {
  case gORStateInit_c:
    uart.printf("\f\r\nWrite Registers\r\n");          
    uart.printf("\r\n-Press [p] Previous Menu\r\n");
    shortCutsEnabled = FALSE;   
    oRState = gORStateStart_c;
    SelfNotificationEvent();
    break;
  case gORStateStart_c:
    uart.printf("\r\n -write the Register address in Hex and [enter]: 0x");
    u8Index = 0;
    oRState = gORWaitForTheAddress_c; 
    break;
  case gORWaitForTheAddress_c:
    if(evDataFromUART){
      if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData))
      {
        if('p' == gu8UartData)
        { 
          bBackFlag = TRUE;
        }
        else
        {
          uart.printf("\r\n -Invalid Character!! ");
          oRState = gORStateStart_c;  
          SelfNotificationEvent();
        }
      }
      else if((gRegisterAddressASCII_c == u8Index) && ('\r' != gu8UartData))
      { 
        uart.printf("\r\n -Value out of Range!! ");
        oRState = gORStateStart_c;
        SelfNotificationEvent();
      }
      else if(isAsciiHex(gu8UartData))
      {
        au8RxString[u8Index++] = gu8UartData;
        auxToPrint[0] = gu8UartData;
        auxToPrint[1] = '\0';
        uart.printf(auxToPrint);
      }
      else
      {
        au8RxString[u8Index] = 0;
        rasRegAddress = (registerAddressSize_t)HexString2Dec(au8RxString);
        uart.printf("\r\n -write the Register value to override in Hex and [enter]: 0x");
        u8Index = 0;
        oRState = gORWaitForTheValue_c; 
      }
      evDataFromUART = FALSE;
    }
    break;
  case gORWaitForTheValue_c:
    if(evDataFromUART)
    {
      if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData))
      {
        if('p' == gu8UartData)
        { 
          bBackFlag = TRUE;
        }
        else
        {
          uart.printf("\r\n -Invalid Character!! ");
          oRState = gORStateStart_c;  
          SelfNotificationEvent();
        }
      }
      else if((2 == u8Index) && ('\r' != gu8UartData))
      { 
        uart.printf("\r\n -Value out of Range!! ");
        oRState = gORStateStart_c;  
        SelfNotificationEvent();
      }
      else if(isAsciiHex(gu8UartData))
      {
        au8RxString[u8Index++] = gu8UartData;
        auxToPrint[0] = gu8UartData;
        auxToPrint[1] = '\0';
        uart.printf(auxToPrint);
      }
      else
      {
        au8RxString[u8Index] = 0;
        rsRegValue = (registerSize_t)HexString2Dec(au8RxString);
        aspTestRequestMsg.msgType = aspMsgTypeXcvrWriteReq_c;
        aspTestRequestMsg.msgData.aspXcvrData.addr = (uint16_t)rasRegAddress;
        aspTestRequestMsg.msgData.aspXcvrData.len  = gRegisterAddress_c;
        aspTestRequestMsg.msgData.aspXcvrData.mode = !bIsRegisterDirect;
        FLib_MemCpy(aspTestRequestMsg.msgData.aspXcvrData.data, &rsRegValue, gRegisterSize_c);
        APP_ASP_SapHandler(&aspTestRequestMsg, 0);
        
        uart.printf("\r\n Register overridden \r\n");
        u8Index = 0;
        oRState = gORStateStart_c; 
        SelfNotificationEvent();
      }
      evDataFromUART = FALSE;
    }
    break;
  default:
    break;
  }
  return bBackFlag;  
}

/************************************************************************************
*
* Handler for Carrier Sense Test and Transmission Control Test
*
************************************************************************************/
bool_t CSenseAndTCtrl(void)
{
  bool_t bBackFlag = FALSE;
  static uint8_t testSelector = 0;
  
  if(evTestParameters){
    (void)MLMESetChannelRequest(testChannel);   
#if CT_Feature_Calibration
    (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
    (void)MLMEPAOutputAdjust(testPower);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  
  switch(cstcState)
  {
  case gCsTcStateInit_c:
    TestMode(gTestModeForceIdle_c);
    PrintMenu(cu8ShortCutsBar, mAppSer);
    PrintMenu(cu8RadioCSTCSelectMenu, mAppSer);
    PrintTestParameters(FALSE);
    shortCutsEnabled = TRUE;
    bTxDone = FALSE;
    bScanDone = FALSE;
    timePassed = FALSE;
    
    cstcState = gCsTcStateSelectTest_c;
    break;
  case gCsTcStateSelectTest_c:
    if(evDataFromUART)
    {
      if('1' == gu8UartData)
      {
        cstcState = gCsTcStateCarrierSenseStart_c;
        testSelector = 1;
        SelfNotificationEvent();
      }
      else if ('2' == gu8UartData)
      {
        cstcState = gCsTcStateTransmissionControlStart_c;
        testSelector = 2;
        SelfNotificationEvent();
      }
      else if( 'p' == gu8UartData)
      {
        cstcState = gCsTcStateInit_c;
        testSelector = 0;
        bBackFlag = TRUE;
      }
      evDataFromUART = FALSE;
    }
    break;
  default:
    if(testSelector == 1)
      CarrierSenseHandler();
    else if(testSelector == 2)
      TransmissionControlHandler();
    break;
  }
  return bBackFlag;  
}

/************************************************************************************
*
* Handler for Transmission Control Test called by above function
*
************************************************************************************/
void TransmissionControlHandler(void)
{
  const uint16_t u16TotalPacketsOptions[] = {1,25,100,500,1000,2000,5000,10000,65535};
  static uint16_t u16TotalPackets;
  static uint16_t u16PacketCounter = 0;
  static uint16_t miliSecDelay = 0;
  static phyTime_t startTime;
  int8_t fillIndex = 0;
  uint8_t* smacPduPtr;
  uint32_t totalTimeMs;
  
  switch(cstcState)
  {
  case gCsTcStateTransmissionControlStart_c:
    PrintMenu(cu8ShortCutsBar, mAppSer);
    PrintMenu(cu8CsTcTestMenu, mAppSer);
    PrintTestParameters(FALSE);
    miliSecDelay = 0;
    u16TotalPackets = 0;
    u16PacketCounter = 0;
    fillIndex = testPayloadLen / gPrbs9BufferLength_c;
    
    while(fillIndex > 0)
    {
      fillIndex--;
      smacPduPtr = gAppTxPacket->smacPdu.smacPdu + fillIndex * gPrbs9BufferLength_c;
      FLib_MemCpy(smacPduPtr, u8Prbs9Buffer, gPrbs9BufferLength_c);
    }
    smacPduPtr = gAppTxPacket->smacPdu.smacPdu + ((testPayloadLen / gPrbs9BufferLength_c)*gPrbs9BufferLength_c);
    FLib_MemCpy(smacPduPtr, u8Prbs9Buffer, (testPayloadLen % gPrbs9BufferLength_c));
    
    gAppTxPacket->u8DataLength = testPayloadLen;
    
    cstcState = gCsTcStateTransmissionControlSelectNumOfPackets_c;
    break;
  case gCsTcStateTransmissionControlSelectNumOfPackets_c:
    if(evDataFromUART)
    {
      if((gu8UartData >= '0') && (gu8UartData <= '8'))
      {
        u16TotalPackets = u16TotalPacketsOptions[gu8UartData - '0'];  
        cstcState = gCsTcStateTransmissionControlSelectInterpacketDelay_c;
        uart.printf("\r\n\r\n Please type InterPacket delay in miliseconds and press [ENTER]");
        uart.printf("\r\n(During test, exit by pressing [SPACE])\r\n\r\n");
        SelfNotificationEvent();
      }
      else if('p' == gu8UartData)
      { 
        cstcState = gCsTcStateInit_c;
        SelfNotificationEvent();
      }
      evDataFromUART = FALSE;
    }
    break;
  case gCsTcStateTransmissionControlSelectInterpacketDelay_c:
    if(evDataFromUART)
    {
      if(gu8UartData == '\r' && miliSecDelay != 0)
      {
        cstcState = gCsTcStateTransmissionControlPerformingTest_c;
        startTime = GetTimestampUS();
        (void)MLMEScanRequest(testChannel);
      }
      else if((gu8UartData >= '0') && (gu8UartData <='9'))
      {
        miliSecDelay = miliSecDelay*10 + (gu8UartData - '0');
        uart.printf("%d",(uint32_t)(gu8UartData - '0'));
      }
      else if('p' == gu8UartData)
      { 
        cstcState = gCsTcStateInit_c;
        SelfNotificationEvent();
      }
      evDataFromUART = FALSE;
    }
    break;
  case gCsTcStateTransmissionControlPerformingTest_c:
    if(bScanDone)
    {
      bScanDone = FALSE;
      (void)MCPSDataRequest(gAppTxPacket);
    }
    if(bTxDone)
    {
      bTxDone = FALSE;                                                  
      u16PacketCounter++;
      uart.printf("\r\n\tPacket number: ");
      uart.printf("%d", (uint32_t)(u16PacketCounter));
      uart.printf("; RSSI value: -");
      uart.printf("%d",(uint32_t)au8ScanResults[testChannel]);
      uart.printf(" dBm\r\n");
      if(u16PacketCounter < u16TotalPackets)
      {
        totalTimeMs  = (uint32_t)(GetTimestampUS() - startTime);
        totalTimeMs -= GetTransmissionTime(testPayloadLen, crtBitrate);
        totalTimeMs = (totalTimeMs % 1000 < 500) ? totalTimeMs/1000 : (totalTimeMs/1000)+1;
        if(totalTimeMs > miliSecDelay)
        {
          uart.printf( " Overhead + Transmission + ED = ~");
          uart.printf("%d", totalTimeMs);
          uart.printf("ms\r\n Interpacket delay too small (Press [ENTER] to continue)\r\n");
          cstcState = gCsTcStateTransmissionControlEndTest_c;
          SelfNotificationEvent();
          break;
        }
        FlaggedDelay_ms(miliSecDelay - totalTimeMs);
      }
      else
      {        
        uart.printf("\r\n\r\nFinished transmitting ");
        uart.printf("%d", (uint32_t)u16TotalPackets);
        uart.printf(" packets!\r\n\r\n");
        uart.printf("\r\n -Press [ENTER] to end Transmission Control Test");
        cstcState = gCsTcStateTransmissionControlEndTest_c;
      }
    }
    if(timePassed)
    {
      timePassed = FALSE;
      startTime = GetTimestampUS();
      (void)MLMEScanRequest(testChannel);
    }
    if(evDataFromUART && gu8UartData == ' ')
    {
      uart.printf("\r\n\r\n-Test interrupted by user. Press [ENTER] to continue\r\n\r\n");
      cstcState = gCsTcStateTransmissionControlEndTest_c;
    }
    break;
  case gCsTcStateTransmissionControlEndTest_c:    
    if(evDataFromUART && gu8UartData == '\r')
    {
      cstcState = gCsTcStateInit_c;
      SelfNotificationEvent();
    }
    evDataFromUART = FALSE;
    break;
  }
}
/************************************************************************************
*
* Handler for Carrier Sense Test
*
************************************************************************************/
void CarrierSenseHandler(void)
{
  int8_t fillIndex = 0;
  uint8_t* smacPduPtr;
  switch(cstcState)
  {
  case gCsTcStateCarrierSenseStart_c:
#if CT_Feature_Calibration
    if( gMode1Bitrate_c == crtBitrate )
    {
      (void)MLMESetAdditionalRFOffset(gOffsetIncrement + 30);
    }
    else
    {
      (void)MLMESetAdditionalRFOffset(gOffsetIncrement + 60);
    }
#endif
    (void)MLMESetChannelRequest(testChannel);
    
    uart.printf( "\r\n\r\n Press [SPACE] to begin/interrupt test");
    uart.printf(  "\r\n Press [p] to return to previous menu");
//    PrintTestParameters(FALSE);
    shortCutsEnabled = FALSE;
    uart.printf("\r\n");
    
    fillIndex = testPayloadLen / gPrbs9BufferLength_c;
    while(fillIndex > 0)
    {
      fillIndex--;
      smacPduPtr = gAppTxPacket->smacPdu.smacPdu + fillIndex * gPrbs9BufferLength_c;
      FLib_MemCpy(smacPduPtr, u8Prbs9Buffer, gPrbs9BufferLength_c);
    }
    smacPduPtr = gAppTxPacket->smacPdu.smacPdu + ((testPayloadLen / gPrbs9BufferLength_c)*gPrbs9BufferLength_c);
    FLib_MemCpy(smacPduPtr, u8Prbs9Buffer, (testPayloadLen % gPrbs9BufferLength_c));
    
    gAppTxPacket->u8DataLength = testPayloadLen;
    
    cstcState = gCsTcStateCarrierSenseSelectType_c;
    break;
  case gCsTcStateCarrierSenseSelectType_c:
    if(evDataFromUART)
    {
      if(' ' == gu8UartData)
      {
        cstcState = gCsTcStateCarrierSensePerformingTest_c;
        (void)MLMEScanRequest(testChannel);
      }
      else if ('p' == gu8UartData)
      {
#if CT_Feature_Calibration
        (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
        (void)MLMESetChannelRequest(testChannel);
        cstcState = gCsTcStateInit_c;
        SelfNotificationEvent();
      }
      evDataFromUART = FALSE;
    }
    break;
  case gCsTcStateCarrierSensePerformingTest_c:
    if(bScanDone)
    {
      bScanDone = FALSE;
      uart.printf( "\r\n\tSampling done. RSSI value: -");
      uart.printf("%d", (uint32_t) au8ScanResults[testChannel]);
      uart.printf( "dBm");
      if(au8ScanResults[testChannel] > ccaThresh)
      {
        (void)MCPSDataRequest(gAppTxPacket);
      }
      else
      {
          (void)MLMEScanRequest(testChannel);
      }
    }
    if(bTxDone)
    {
      bTxDone = FALSE;

      uart.printf("\r\n Transmission Performed\r\n");
      uart.printf("\r\n -Press [ENTER] to end Carrier Sense Test");
      cstcState = gCsTcStateCarrierSenseEndTest_c;
    }
    if(evDataFromUART && gu8UartData == ' ')
    {
      uart.printf("\r\n\r\n-Test interrupted by user. Press [ENTER] to continue\r\n\r\n");
      cstcState = gCsTcStateCarrierSenseEndTest_c;
    }
    break;
  case gCsTcStateCarrierSenseEndTest_c:
    if(evDataFromUART && gu8UartData == '\r')
    {
#if CT_Feature_Calibration
      (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
      (void)MLMESetChannelRequest(testChannel);
      cstcState = gCsTcStateInit_c;
      SelfNotificationEvent();
    }
    evDataFromUART = FALSE;
    break;
  }
}

/************************************************************************************
*
* Auxiliary Functions
*
************************************************************************************/

/**************************************************************************************/
void SetRadioRxOnNoTimeOut(void)
{
  bRxDone = FALSE;
  gAppRxPacket->u8MaxDataLength = gMaxSmacSDULength_c;
  (void)MLMERXEnableRequest(gAppRxPacket, 0);
}

/**************************************************************************************/
void PrintPerRxFinalLine(uint16_t u16Received, uint16_t u16Total)
{
  uart.printf("Received ");
  uart.printf("%d",(uint32_t)u16Received);
  uart.printf(" of ");
  uart.printf("%d",(uint32_t)u16Total);
  uart.printf(" packets transmitted \r\n");
  uart.printf("\r\n Press [enter] to go back to the Per Rx test menu");
}

/************************************************************************************
* 
* 
* By employing this function, users can execute a test of the radio. Test mode 
* implements the following:
*   -PRBS9 Mode, 
*   -Force_idle, 
*   -Continuos TX without modulation, 
*   -Continuos TX with modulation.(0's,1's and PN patterns)
*
************************************************************************************/
smacErrors_t TestMode
(
smacTestMode_t  mode  /*IN: The test mode to start.*/
)
{
  aspTestRequestMsg.msgType = aspMsgTypeTelecTest_c;
  
#if(TRUE == smacParametersValidation_d)
  if(gMaxTestMode_c <= mode)
  {
    return gErrorOutOfRange_c;
  }
#endif
  
  if(gTestModeForceIdle_c == mode)
  {
    aspTestRequestMsg.msgData.aspTelecTest.mode = gTestForceIdle_c;
  }                                                                             
  else if(gTestModeContinuousTxModulated_c == mode)
  {
    /*if(contTxModBitValue==gContTxModSelectOnes_c)
    {
      aspTestRequestMsg.msgData.aspTelecTest.mode = gTestContinuousTxModOne_c;
    }
    else if(contTxModBitValue == gContTxModSelectZeros_c)
    {
      aspTestRequestMsg.msgData.aspTelecTest.mode = gTestContinuousTxModZero_c;
    }
    else */if(contTxModBitValue == gContTxModSelectPN9_c)
    {
#ifdef gPHY_802_15_4g_d
      aspTestRequestMsg.msgData.aspTelecTest.mode = gTestContinuousTxContPN9_c;
#else
      aspTestRequestMsg.msgData.aspTelecTest.mode = gTestPulseTxPrbs9_c;
#endif
    }
  } 
  else if(gTestModeContinuousTxUnmodulated_c == mode)
  { 
    aspTestRequestMsg.msgData.aspTelecTest.mode = gTestContinuousTxNoMod_c;
  } 
  else if(gTestModeContinuousRxBER_c == mode)
  {
    aspTestRequestMsg.msgData.aspTelecTest.mode = gTestContinuousRx_c;
  }
  else if(gTestModePRBS9_c == mode)
  {   
    /*Set Data Mode*/
    gAppTxPacket->u8DataLength = gPrbs9BufferLength_c;
    FLib_MemCpy(gAppTxPacket->smacPdu.smacPdu, u8Prbs9Buffer, gPrbs9BufferLength_c);
    PacketHandler_Prbs9(); 
  }
  if(gTestModePRBS9_c != mode)
    (void)APP_ASP_SapHandler(&aspTestRequestMsg, 0);
  
  return gErrorNoError_c;
}

/************************************************************************************
* PacketHandler_Prbs9
* 
* This function sends OTA the content of a PRBS9 polynomial of 65 bytes of payload.
*
*
************************************************************************************/

void PacketHandler_Prbs9(void)                                                  
{
  smacErrors_t err;
  /*@CMA, Need to set Smac to Idle in order to get PRBS9 to work after a second try on the Conn Test menu*/
  (void)MLMERXDisableRequest();                                               
  (void)MLMETXDisableRequest();
  err = MCPSDataRequest(gAppTxPacket);  
  if(err != gErrorNoError_c)
  {
    failedPRBS9 = TRUE;
    SelfNotificationEvent(); //in case data isn't sent, no confirm event will fire. 
    //this way we need to make sure the application will not freeze.
  }
}

/*****************************************************************************
* UartRxCallBack function
*
* Interface assumptions:
* This callback is triggered when a new byte is received over the UART
*
* Return Value:
* None
*****************************************************************************/
void UartRxCallBack(void) 
{
  gu8UartData = uart.getc();
  gTaskEventFlags |= gUART_RX_EVENT_c;
  mainTask->signal_set(gEventsAny_c);  
}

/*@CMA, Conn Test. Range Test CallBack*/
void RangeTest_Timer_CallBack ()
{
  gTaskEventFlags |= gRangeTest_EVENT_c;
  mainTask->signal_set(gEventsAny_c);  
}



/************************************************************************************
*
* Increments channel on the ED Confirm event and fires a new ED measurement request
*
************************************************************************************/
void IncrementChannelOnEdEvent()
{
  bScanDone = FALSE;
  smacErrors_t err;
  if (ChannelToScan <= gMaxChannel_c)                              
  {                                                                  
    err = MLMEScanRequest((channels_t)ChannelToScan);                                          
    if(err == gErrorNoError_c)
      ChannelToScan++;                                                //Increment channel to scan
  }
  else 
  {
    bScanDone = TRUE;                                               //PRINT ALL CHANNEL RESULTS
    FlaggedDelay_ms(300);                                           //Add delay between channel scanning series.
  } 
}

/***********************************************************************
*********************Utilities Software********************************
************************************************************************/

bool_t stringComp(uint8_t * au8leftString, uint8_t * au8RightString, uint8_t bytesToCompare)
{
  do
  {
  }while((*au8leftString++ == *au8RightString++) && --bytesToCompare);
  return(0 == bytesToCompare);
}

uint32_t HexString2Dec(uint8_t* au8String)
{
  uint8_t u8LocIndex=0;
  uint8_t u8LocIndex2=0;
  uint32_t u32DecValue = 0;
  
  while(au8String[u8LocIndex]){
    u8LocIndex++;
  }
  
  while(u8LocIndex--){
    if((au8String[u8LocIndex] >= '0') && (au8String[u8LocIndex] <= '9'))
      u32DecValue |= ((uint32_t)(au8String[u8LocIndex] - '0'))<<(u8LocIndex2*4);
    else if((au8String[u8LocIndex] >= 'A') && (au8String[u8LocIndex] <= 'F')){
      u32DecValue |= ((uint32_t)(au8String[u8LocIndex] - 'A' + 0x0A))<<(u8LocIndex2*4);    
    }else{
      u32DecValue |= ((uint32_t)(au8String[u8LocIndex] - 'a' + 0x0A))<<(u8LocIndex2*4);        
    }
    u8LocIndex2++;
  }
  
  return u32DecValue;
}

static void DelayTimeElapsed()
{
  timePassed = TRUE;
  gTaskEventFlags |= gTimePassed_EVENT_c;
  mainTask->signal_set(gEventsAny_c);  
}

/***********************************************************************************
*
* PrintMenu
*
************************************************************************************/
void PrintMenu(char * const pu8Menu[], uint8_t port)
{
  uint8_t u8Index = 0;
  (void)port;
  while(pu8Menu[u8Index]){
    uart.printf(pu8Menu[u8Index]);
    u8Index++;
  }
}

/***********************************************************************
************************************************************************/

/************************************************************************************
*
* PrintTestParameters
*
************************************************************************************/
void PrintTestParameters(bool_t bEraseLine)
{
  uint8_t u8lineLen = 63;
  uint8_t u8Index;
  
  if(bEraseLine)
  {
    for(u8Index = 0;u8Index<u8lineLen;u8Index++)
    {
      uart.printf("\b");
    }
  }
  
  uart.printf("Mode ");
  if(mTxOperation_c == testOpMode)
  {
    uart.printf("Tx");
  }
  else
  {
    uart.printf("Rx");
  }
  uart.printf(", Channel ");
  uart.printf("%d", (uint32_t)testChannel);
  uart.printf(", Power ");
  uart.printf("%d",(uint32_t)testPower);
  uart.printf(", Payload ");
  uart.printf("%d", (uint32_t)testPayloadLen);
  uart.printf(", CCA Thresh ");
  if(ccaThresh != 0)
  {
    uart.printf("-");
  }
  uart.printf("%d", (uint32_t)ccaThresh);
  uart.printf("dBm");
  uart.printf(" >");
}
/*****************************************************************************/

int main()
{
  mainTask = new Thread(main_task);
  while(1)
  {
    
  }
  return 0;
}