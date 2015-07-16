/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file App_Custom.c
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
#include "Connectivity_Test_Platform.h"

/************************************************************************************
*************************************************************************************
* Macros
*************************************************************************************
************************************************************************************/
#define gCTSelf_EVENT_c (1<<7)
#define SelfNotificationEvent() (OSA_EventSet(&gTaskEvent, gCTSelf_EVENT_c));

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
const registerLimits_t registerIntervals[] = 
{
  {.regStart=0x00 , .regEnd=0x26, TRUE},
  {.regStart=0x28 , .regEnd=0x3F, TRUE},
  {.regStart=0x00 , .regEnd=0x28, FALSE},
  {.regStart=0x2A , .regEnd=0x2E, FALSE},
  {.regStart=0x30 , .regEnd=0x32, FALSE},
  {.regStart=0x34 , .regEnd=0x45, FALSE},
  {.regStart=0x47 , .regEnd=0x56, FALSE},
  {.regStart=0x58 , .regEnd=0x5B, FALSE},
  {.regStart=0x5D , .regEnd=0x6B, FALSE},
  {.regStart=0x6E , .regEnd=0x71, FALSE},
  {.regStart=0x74 , .regEnd=0x75, FALSE},
  {.regStart=0x78 , .regEnd=0x83, FALSE},
  {.regStart=0x86 , .regEnd=0x86, FALSE},
  {.regStart=0x89 , .regEnd=0x8A, FALSE},
  {.regStart=0x8D , .regEnd=0x8E, FALSE},
  {.regStart=0x91 , .regEnd=0x97, FALSE},
  {.regStart=0x9A , .regEnd=0xA1, FALSE},
  {.regStart=0xA3 , .regEnd=0xA8, FALSE},
  {.regStart=0xAA , .regEnd=0xAF, FALSE},
  {.regStart=0xB2 , .regEnd=0xB6, FALSE},
  {.regStart=0xFE , .regEnd=0xFF, FALSE},
  {.regStart=0x00 , .regEnd=0x00, FALSE}
};

uint8_t u8Prbs9Buffer[] =
{
  0x42,
  0xff,0xc1,0xfb,0xe8,0x4c,0x90,0x72,0x8b,0xe7,0xb3,0x51,0x89,0x63,0xab,0x23,0x23,  
  0x02,0x84,0x18,0x72,0xaa,0x61,0x2f,0x3b,0x51,0xa8,0xe5,0x37,0x49,0xfb,0xc9,0xca,
  0x0c,0x18,0x53,0x2c,0xfd,0x45,0xe3,0x9a,0xe6,0xf1,0x5d,0xb0,0xb6,0x1b,0xb4,0xbe,
  0x2a,0x50,0xea,0xe9,0x0e,0x9c,0x4b,0x5e,0x57,0x24,0xcc,0xa1,0xb7,0x59,0xb8,0x87
};
/************************************************************************************
*************************************************************************************
* Private function prototypes
*************************************************************************************
************************************************************************************/

/************************************************************************************
*
* InitApp_custom
*
************************************************************************************/
void InitApp_custom()
{
}

/************************************************************************************
*
* InitProject_custom
*
************************************************************************************/
void InitProject_custom()
{
}

/************************************************************************************
*
* ShortCutsParser
*
* Performs changes in different menus whenever shortcuts are allowed
*
************************************************************************************/
void ShortCutsParser(uint8_t u8UartData)
{
  evTestParameters = TRUE;
  evDataFromUART = FALSE;
  switch(u8UartData){
  case 't':
    testOpMode = mTxOperation_c;
    break;
  case 'r':
    testOpMode = mRxOperation_c;
    break;
  case 'q': 
    if(testChannel == (channels_t)gTotalChannels)
    {
      testChannel = gChannel11_c;
    }
    else
    {
      testChannel++;
    }
    break;
  case 'w':
    if(testChannel == gChannel11_c)
    {
      testChannel = (channels_t)gTotalChannels;
    }
    else
    {
      testChannel--;
    }
    break;
  case 'a':
    testPower++;
    if(gMaxOutputPower_c < testPower)
    {
      testPower = gMinOutputPower_c;
    }
    break;
  case 's':
    if(testPower == gMinOutputPower_c)
    {
      testPower = gMaxOutputPower_c;
    }
    else
    {
      testPower--;  
    }
    break;
  case 'n':
    testPayloadLen++;
    if(gMaxSmacSDULength_c < testPayloadLen)
    {
      testPayloadLen = 17;
    }    
    break;
  case 'm':
    testPayloadLen--;
    if(17 > testPayloadLen)
    {
      testPayloadLen = gMaxSmacSDULength_c;
    }    
    break;
  case 'k':
    ccaThresh++;
    if(ccaThresh > gMaxCCAThreshold_c)
    {
      ccaThresh = gMinCCAThreshold_c;
    }
    break;
  case 'l':
    ccaThresh--;
    if(ccaThresh > gMaxCCAThreshold_c)
    {
      ccaThresh = gMaxCCAThreshold_c;
    }
    break;
  default:
    evDataFromUART = TRUE;
    evTestParameters = FALSE;
    break;
  }
}