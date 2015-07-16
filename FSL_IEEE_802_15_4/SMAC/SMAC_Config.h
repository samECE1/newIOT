/**************************************************************************************************
* This header contains SMAC basic configuration.
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

#ifndef SMAC_CONFIG_H_
#define SMAC_CONFIG_H_

/* BEGIN SMAC Config Options Definition */

#define gNodeAddress_c             (0xBEAD)
#define gDefaultPanID_c            (0xFACE)
#define CCA_BEFORE_TX              (FALSE)
#define gMaxRetriesAllowed_c       ( 0x05 )
//Make sure gMaxBackoffTime_c is of value (1<<n)-1. Ex: (1<<3)-1 = 7, (1<<4)-1 = 15;
#define gMaxBackoffTime_c          ( 0x1F )
#define gMinBackoffTime_c          ( 0x00 )
/* END SMAC Config Options Definition */

/* BEGIN Configuration Parameters Definition */ 
#define gMaxOutputPower_c          ( 0x1F ) 
#define gMinOutputPower_c              ( 0x03 )
#define gDefaultOutputPower_c          ( 0x0F )               
#define gDefaultChannelNumber_c         gChannel11_c   
#define gDefaultAddress_c              (gBroadcastAddress_c)
/* END Configuration Parameters Definition */

#endif /* SMAC_CONFIG_H_ */
