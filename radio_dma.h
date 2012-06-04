/*
 * Copyright (c) 2010, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Software module for AT86RF231 (SPI)
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanley S. Baek      2010-06-05    Initial release
 *  Humphrey Hu		 2011-06-06    Added DMA and asynchronous support
 *                      
 * Notes:
 *  SPI1 is used for AT86RF231
 *  SPI2 should be used if you run this module on
 *  MikroElektronika dev board because RB2 is used for LCD.
 *
 *  This module uses DMA channels 4 and 5 as defined. These channels can
 *  be changed by changing the relevant definitions below
 * 
 */

#ifndef __RADIO_DMA_H
#define __RADIO_DMA_H

#include "mac_packet.h"
#include "payload.h"

void radioSetup(unsigned int tx_queue_length, unsigned int rx_queue_length);

void radioSetAntDiversity(char enable);
void radioSetSrcAddr(unsigned int src_addr);
void radioSetSrcPanID(unsigned int src_pan_id);
void radioSetDestPanID(unsigned int dest_pan_id);
void radioSetChannel(char channel);

void radioReadTrxId(unsigned char *id);
unsigned char radioGetTrxState(void);
int radioIsTxQueueFull(void);
int radioIsRxQueueEmpty(void);
void radioDeleteQueues(void);

/*****************************************************************************
* Function Name : radioCreateMacPacket
* Description   : 
* Parameters    : 
* Return Value  : None                                                     
*****************************************************************************/
MacPacket radioCreateMacPacket(unsigned int dest_addr, unsigned char data_size);


void radioDeleteMacPacket(MacPacket packet);


/*****************************************************************************
* Function Name : radioGetRxFrame
* Description   : . 
* Parameters    : None
* Return Value  : A character read from the radio.                                                     
*****************************************************************************/
MacPacket radioGetRxPacket(void);



/*****************************************************************************
* Function Name : radioSendPayload
* Description   : transmit a payload
* Parameters    : payload to be tramsmitted, an integer value of destination 
*                 address, and an integer value of destination PAN ID.
* Return Value  : It returns 1 if successful. Otherwise, it returns 0.
*****************************************************************************/
unsigned char radioSendPacket(MacPacket packet);



#endif // __RADIO_DMA_H
