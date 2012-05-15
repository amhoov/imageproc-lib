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

#include "ports.h"      // for external interrupt
#include "utils.h"
#include "spi.h"
#include "radio_dma.h"
#include "at86rf231.h"
#include "payload.h"
#include "queue.h"
#include "generic_typedefs.h"
#include "mac_packet.h"
#include "dma.h"

#include "lcd.h"
#include <stdio.h>
#include <stdlib.h>


#if defined(__MIKRO)

    #define SPI_BUF         SPI2BUF
    #define SPI_CON1bits    SPI2CON1bits
    #define SPI_CON2        SPI2CON2
    #define SPI_STATbits    SPI2STATbits
    #define SPI_CS          _LATG9
    #define SLPTR           _LATF0      //_LATB15
    #define DMAxREQ_VAL     0x021     // SPI2 Transfer Done Interrupt

#elif defined(__EXP16DEV)

    #define SPI_BUF         SPI2BUF
    #define SPI_CON1bits    SPI2CON1bits
    #define SPI_CON2        SPI2CON2
    #define SPI_STATbits    SPI2STATbits
    #define SPI_CS          _LATG9
    #define SLPTR           _LATB1
    #define DMAxREQ_VAL     0x021     // SPI2 Transfer Done Interrupt

#elif defined(__BASESTATION) || defined(__BASESTATION2)

    #define SPI_BUF         SPI2BUF
    #define SPI_CON1bits    SPI2CON1bits
    #define SPI_CON2        SPI2CON2
    #define SPI_STATbits    SPI2STATbits
    #define SPI_CS          _LATG9
    #define SLPTR           _LATE5
    #define DMAxREQ_VAL     0x021     // SPI2 Transfer Done Interrupt

#elif defined(__IMAGEPROC2)

    #define SPI_BUF         SPI1BUF
    #define SPI_CON1bits    SPI1CON1bits
    #define SPI_CON2        SPI1CON2
    #define SPI_STATbits    SPI1STATbits
    #define SPI_CS          _LATB2
    #define SLPTR           _LATB15
    #define DMAxREQ_VAL     0x00A     // SPI1 Transfer Done Interrupt

#else

//    #error "SPI is not defined on this project

#endif


// DMA Peripheral to RAM Read Channel
#define DMAR_CONbits        DMA4CONbits
#define DMAR_CNT	    DMA4CNT
#define DMAR_STA	    DMA4STA
#define DMAR_STB	    DMA4STB
#define DMAR_PAD	    DMA4PAD
#define DMAR_REQbits	    DMA4REQbits
#define DMAR_IF		    DMA4IF
#define DMAR_IE		    DMA4IE

// DMA RAM to Peripheral Write Channel
#define DMAW_CONbits        DMA5CONbits
#define DMAW_CNT	    DMA5CNT
#define DMAW_STA	    DMA5STA
#define DMAW_STB	    DMA5STB
#define DMAW_PAD	    DMA5PAD
#define DMAW_REQbits	    DMA5REQbits
#define DMAW_IF		    DMA5IF
#define DMAW_IE		    DMA5IE


#define TRX_CMD_RW          (0xC0) // Register Write
#define TRX_CMD_RR          (0x80) // Register Read 
#define TRX_CMD_FW          (0x60) // Frame Transmit Mode
#define TRX_CMD_FR          (0x20) // Frame Receive Mode
#define TRX_CMD_SW          (0x40) // SRAM Write.
#define TRX_CMD_SR          (0x00) // SRAM Read.

#define MAX_FRAME_LEN       (127)
#define DMA_FRAME_SIZE	    (128) // Number of bytes allocated to DMA frame


// Based on 16-bit addressing for PAN and device and no 
// auxiliary security header
#define MAC_HEADER_LENGTH       9
#define CRC_LENGTH              2


// default value for MAC HEADER
//#define DEFAULT_CHANNEL         0x15
//#define DEFAULT_DEST_PAN_ID     0x1000
//#define DEFAULT_SRC_PAN_ID      0x1000
//#define DEFAULT_DEST_ADDR       0x1020   
//#define DEFAULT_SRC_ADDR        0x1101
#define DEFAULT_CHANNEL         0x14
#define DEFAULT_DEST_PAN_ID     0x0000
#define DEFAULT_SRC_PAN_ID      0x0000
#define DEFAULT_DEST_ADDR       0x0100   
#define DEFAULT_SRC_ADDR        0x0110


// packet types
#define PACKET_TYPE_BEACON      0x00
#define PACKET_TYPE_DATA        0x01
#define PACKET_TYPE_ACK         0x02
#define PACKET_TYPE_COMMAND     0x03
#define PACKET_TYPE_RESERVE     0x04

// ACK
#define PACKET_NO_ACK_REQ       0
#define PACKET_ACK_REQ          1

typedef enum {
    DMA_TRX_IDLE = 0,
    DMA_TRX_READ,
    DMA_TRX_WRITE,
} DMAState;


typedef enum {
    STATE_SLEEP = 0,
    STATE_TRX_OFF,
    STATE_PLL_ON,
    STATE_RX_ON,
    STATE_RX_AACK_ON,
    STATE_TX_ARET_ON,
    STATE_BUSY_TX_ARET,
} RadioState;


/*-----------------------------------------------------------------------------
 *          Static Variables
-----------------------------------------------------------------------------*/

volatile char currentState;
static unsigned char radioLqi;

static Queue txQueue;
static Queue rxQueue;

static unsigned char txPacketSqn;

static DMAState dmarState;
static DMAState dmawState;

// DMA memory for radio read/write
unsigned char rxFrame[DMA_FRAME_SIZE] __attribute__((space(dma)));
unsigned char txFrame[DMA_FRAME_SIZE] __attribute__((space(dma)));

static unsigned int srcAddr;
static unsigned int srcPanID;
static unsigned int destPanID;


/*-----------------------------------------------------------------------------
 *      Declaration of static functions (Transceiver-specific functions)
-----------------------------------------------------------------------------*/
// IRQ handlers
static void trxHandleISR(void);


static void trxReceivePacket(void);
static void trxSendPacket(void);
static void trxReadFrame(void);

// Transceiver function helpers
static inline void trxSetSlptr(char val);
static unsigned char trxReadReg(unsigned char addr);
static void trxWriteReg(unsigned char addr, unsigned char val);
static unsigned char trxReadBit(unsigned char addr, unsigned char mask, 
                    unsigned char pos);
static void trxWriteBit(unsigned char addr, unsigned char mask, 
            unsigned char pos, unsigned char val);

//static unsigned char trxReadSram(unsigned char addr, unsigned char length);
//static void trxWriteSram(unsigned char addr, unsigned char* data);
static unsigned char trxReadByte(void);
static unsigned char trxWriteByte(unsigned char dout);

// Setup functions
static void trxSetup(void);
static void trxSetupPeripheral(void);
static void trxSetupDma(void);



/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void radioSetup(unsigned int tx_queue_length, unsigned int rx_queue_length) {

    trxSetup();
    trxSetupDma();	// Setup DMA channels

    currentState = STATE_RX_AACK_ON;
    txQueue = queueInit(tx_queue_length);
    rxQueue = queueInit(rx_queue_length);

    txPacketSqn = 0;
        
    srcAddr = DEFAULT_SRC_ADDR;
    srcPanID = DEFAULT_SRC_PAN_ID;
    destPanID = DEFAULT_DEST_PAN_ID;
    
    ConfigINT4(RISING_EDGE_INT & EXT_INT_ENABLE & EXT_INT_PRI_5); // Radio    

}


void radioSetAntDiversity(char enable) {

    trxWriteBit(SR_ANT_DIV_EN, enable);
    trxWriteBit(SR_ANT_EXT_SW_EN, enable);
}


void radioSetSrcAddr(unsigned int src_addr) {

    trxWriteReg(RG_SHORT_ADDR_0, (src_addr & 0xff));
    trxWriteReg(RG_SHORT_ADDR_1, ((src_addr >> 8) & 0xff));
    srcAddr = src_addr;
}

void radioSetSrcPanID(unsigned int src_pan_id) {
    // set PAN ID
    trxWriteReg(RG_PAN_ID_0, (src_pan_id & 0xff));
    trxWriteReg(RG_PAN_ID_1, ((src_pan_id >> 8) & 0xff));
    srcPanID = src_pan_id;
}

void radioSetDestPanID(unsigned int dest_pan_id) {
    destPanID = dest_pan_id;
}


void radioSetChannel(char channel) {

    trxWriteBit(SR_CHANNEL, channel);
    
}


/*****************************************************************************
* Function Name : radioReadId
* Description   : This function reads ID number of AT86RF231 chip
* Parameters    : A character array of size 4 to hold id value
* Return Value  : None
*****************************************************************************/
void radioReadTrxId(unsigned char *id) {
    SPI_CS = 1;     // just to make sure to set chip-select
    id[0] = trxReadReg(RG_PART_NUM);      // should be 3
    id[1] = trxReadReg(RG_VERSION_NUM);   // should be 2
    id[2] = trxReadReg(RG_MAN_ID_1);      // should be 0x1F
    id[3] = trxReadReg(RG_MAN_ID_0);      // should be 0

}

void radioResetPacketSqn(void) {
    txPacketSqn = 0;
}



unsigned char radioGetTrxState(void) {
    return trxReadBit(SR_TRX_STATUS);
}


int radioIsTxQueueFull(void) {
    return queueIsFull(txQueue);
}

int radioIsRxQueueEmpty(void){
    return queueIsEmpty(rxQueue);
}


void radioDeleteQueues(void) {

    MacPacket packet;

    while (!queueIsEmpty(txQueue)) {
        packet = (MacPacket)queuePop(txQueue);    
        radioDeleteMacPacket(packet);
    }

    while (!queueIsEmpty(rxQueue)) {
        packet = (MacPacket)queuePop(rxQueue);    
        radioDeleteMacPacket(packet);
    }

}



MacPacket radioCreateMacPacket(unsigned int dest_addr, unsigned char data_size) {

    MacPacket packet = (MacPacket)malloc(sizeof(MacPacketStruct));
    if(packet == NULL) {
	return NULL;
    }

    Payload pld = payCreateEmpty(data_size);
    if(pld == NULL) {
        free(packet);
        return NULL;
    }

    packet->payload = pld;
    packet->payload_length = payGetPayloadLength(pld);

    packet->frame_ctrl.bits.packet_type = PACKET_TYPE_DATA;
    packet->frame_ctrl.bits.sec_en = 0;
    packet->frame_ctrl.bits.frm_pending = 0;
    packet->frame_ctrl.bits.ack_req = PACKET_ACK_REQ;
    packet->frame_ctrl.bits.pan_id_comp = 1;
    packet->frame_ctrl.bits.reserved = 0;
    packet->frame_ctrl.bits.dest_addr_mode = 2;
    packet->frame_ctrl.bits.frm_version = 1;
    packet->frame_ctrl.bits.src_addr_mode = 2;
    packet->seq_num = 0;
    packet->dest_pan_id.val = destPanID;
    packet->src_pan_id.val = srcPanID;
    packet->dest_addr.val = dest_addr;
    packet->src_addr.val = srcAddr;

    return packet;

}


void radioDeleteMacPacket(MacPacket packet) {
	payDelete(packet->payload);
	free(packet);
}


MacPacket radioGetRxPacket(void) {
    if (queueIsEmpty(rxQueue)) {
        return NULL;
    } else {
        return (MacPacket)queuePop(rxQueue);
    }
}

unsigned char radioSendPacket(MacPacket packet) {

    if (queueIsFull(txQueue)) {
        return 0;
    } else {
        packet->seq_num = txPacketSqn++;
        queuePush(txQueue, packet);
        trxSendPacket();
        return 1;
    }

}


/*-----------------------------------------------------------------------------
 * The functions below have been developed for a specific radio transceiver, 
 * AT86RF231.
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

// ISR for DMA4 interrupt, SPI data has moved to rxBuffer.   
void __attribute__((interrupt, no_auto_psv)) _DMA4Interrupt(void) {
    SPI_CS = 1;
    if(dmarState == DMA_TRX_READ) {
        trxReadFrame();
    }
    dmarState = DMA_TRX_IDLE;
    IFS2bits.DMA4IF = 0;	
}	

// ISR for DMA5 interrupt, txBuffer data has moved to SPI.
void __attribute__((interrupt, no_auto_psv)) _DMA5Interrupt(void) {
	SPI_CS = 1;
	dmawState = DMA_TRX_IDLE;
	IFS3bits.DMA5IF = 0;	
}	


/*****************************************************************************
* Function Name : _INT4Interrupt
* Description   : Interrupt hander for 802.15.4 radio
* Parameters    : None
* Return Value  : None
*****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _INT4Interrupt(void) {

    trxHandleISR();
    _INT4IF = 0;    // Clear the interrupt flag

}

/*****************************************************************************
* Function Name : trxHandleISR
* Description   : The function is called by interrupt, IRQ_TRX_END. EXT_INT4 
*                 should be enabled and RD11 should be set as input.
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static void trxHandleISR(void) {

    unsigned char irq_cause;

    irq_cause = trxReadReg(RG_IRQ_STATUS);

    if (irq_cause == TRX_IRQ_TRX_END) {
        if (currentState == STATE_RX_AACK_ON) { // new packet arrived
            trxReceivePacket();            // receive the packet   
        } else {    // transmit is completed.

            /*
            if (trxReadBit(SR_TRAC_STATUS) != TRAC_SUCCESS) {  // No ACK.
                // do something.... 
                // write codes here.....
                // LED_2 = 1;
            } */

            if (queueIsEmpty(txQueue)) {  // tx queue is empty
                trxWriteReg(RG_TRX_STATE, CMD_PLL_ON); // change state to PLL_ON
                trxWriteReg(RG_TRX_STATE, CMD_RX_AACK_ON);  // change state to RX
                currentState = STATE_RX_AACK_ON;
            } else {
                currentState = STATE_TX_ARET_ON;
                trxSendPacket();
            }

        }
    }


}


/*****************************************************************************
* Function Name : trxSendPacket
* Description   : send out data
* Parameters    : length - number of bytes to send
*                 frame - pointer to an array of bytes to be sent. Users
*                 must define the array before this function call.
* Return Value  : None                                                     
*****************************************************************************/
static void trxSendPacket(void) {
    
    if (currentState ==  STATE_BUSY_TX_ARET) {
        return;
    }

    unsigned char state;

    while (1) {     // wait until radio is not busy
        state = trxReadBit(SR_TRX_STATUS);
        if (state == CMD_TX_ARET_ON) {
            break;
        } else if (state == CMD_RX_AACK_ON) {
            trxWriteReg(RG_TRX_STATE, CMD_PLL_ON); 
            trxWriteReg(RG_TRX_STATE, CMD_TX_ARET_ON); 
            break;
        }
    }

    MacPacket tx_packet = (MacPacket)queuePop(txQueue); 

    tx_packet->payload_length = payGetPayloadLength(tx_packet->payload);

    currentState = STATE_BUSY_TX_ARET;
    trxSetSlptr(1);
    trxSetSlptr(0);

    int i = 0;

    txFrame[i++] = tx_packet->payload_length + MAC_HEADER_LENGTH + CRC_LENGTH;
    txFrame[i++] = tx_packet->frame_ctrl.val.byte.LB;
    txFrame[i++]= tx_packet->frame_ctrl.val.byte.HB;
    txFrame[i++] = tx_packet->seq_num;
    txFrame[i++] = tx_packet->dest_pan_id.byte.LB;
    txFrame[i++] = tx_packet->dest_pan_id.byte.HB;
    txFrame[i++] = tx_packet->dest_addr.byte.LB;
    txFrame[i++] = tx_packet->dest_addr.byte.HB;
    //txFrame[i++] = tx_packet->src_pan_id.byte.LB;
    //txFrame[i++] = tx_packet->src_pan_id.byte.HB;
    txFrame[i++] = tx_packet->src_addr.byte.LB;
    txFrame[i++] = tx_packet->src_addr.byte.HB;

    int j = 0;
    payInitIterator(tx_packet->payload);
    for (j = 0; j < tx_packet->payload_length; j++) {
        txFrame[i++] = payNextElement(tx_packet->payload);
    }

    radioDeleteMacPacket(tx_packet);


    while(dmarState != DMA_TRX_IDLE);	// Wait for DMA read channel to finish
    while(dmawState != DMA_TRX_IDLE);	// Wait for DMA write channel to finish
    //while(SPI_CS != 1);			// Wait for SPI bus to become available

    dmarState = DMA_TRX_WRITE;
    dmawState = DMA_TRX_WRITE;

    // DMAR must read one extra byte for SPI command transmit
    DMAR_CNT = tx_packet->payload_length + MAC_HEADER_LENGTH + 2;
    DMAW_CNT = tx_packet->payload_length + MAC_HEADER_LENGTH + 1;
    DMAR_CONbits.CHEN = 1;
    DMAW_CONbits.CHEN = 1;

    SPI_CS = 0;     // begin SPI
    SPI_BUF = TRX_CMD_FW;


}

/*****************************************************************************
* Function Name : trxRecievePacket
* Description   : recieve data from radio without interrupt. 
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static void trxReceivePacket(void) {

    if (!trxReadBit(SR_RX_CRC_VALID)) {  // check CRC
        return;                          // CRC invalid
    }

    while(dmarState != DMA_TRX_IDLE);	// Wait for DMA read channel to finish
    while(dmawState != DMA_TRX_IDLE);	// Wait for DMA write channel to finish
    //while(SPI_CS != 1);	        // Wait for SPI bus to become available

    dmarState = DMA_TRX_READ;	
    dmawState = DMA_TRX_READ;

    DMAR_CNT = MAX_FRAME_LEN;   // Start reading max frame length
    DMAW_CNT = MAX_FRAME_LEN;

    SPI_CS = 0;                 // Select transceiver 

    trxWriteByte(TRX_CMD_FR);	// Start frame read and read off garbage byte

    DMAR_CONbits.CHEN = 1;	// Enable and force start DMA
    DMAW_CONbits.CHEN = 1;
    DMAW_REQbits.FORCE = 1;

    while(DMAR_STA == __builtin_dmaoffset(rxFrame)); // Wait until first byte has been read

    unsigned char length = rxFrame[0] - CRC_LENGTH;	 // Set DMA length
    
    // Must read one extra byte for radio LQI
    DMAR_CNT = length + 1;
    DMAW_CNT = length + 1;

}


/*****************************************************************************
* Function Name : trxReadFrame
* Description   : Writes to macRxFrame from radio read and pushes payload to queue
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static void trxReadFrame(void) {

    unsigned char data_size;

    data_size = rxFrame[0] - MAC_HEADER_LENGTH - CRC_LENGTH - PAYLOAD_HEADER_LENGTH;

    MacPacket rx_packet = radioCreateMacPacket(0, data_size);

    if(rx_packet == NULL) { // if no more space,   
	return;             // just drop the packet
    }

    rx_packet->payload_length = data_size + - PAYLOAD_HEADER_LENGTH;
    rx_packet->frame_ctrl.val.byte.LB = rxFrame[1];
    rx_packet->frame_ctrl.val.byte.HB = rxFrame[2];
    rx_packet->seq_num = rxFrame[3];
    rx_packet->dest_pan_id.byte.LB = rxFrame[4];
    rx_packet->dest_pan_id.byte.HB = rxFrame[5];
    rx_packet->dest_addr.byte.LB = rxFrame[6];
    rx_packet->dest_addr.byte.HB = rxFrame[7];

    unsigned char i = 8;

    if (!rx_packet->frame_ctrl.bits.pan_id_comp) {
        rx_packet->src_pan_id.byte.LB = rxFrame[i++];
        rx_packet->src_pan_id.byte.HB = rxFrame[i++];
    }
    rx_packet->src_addr.byte.LB = rxFrame[i++];
    rx_packet->src_addr.byte.HB = rxFrame[i++];

    paySetStatus(rx_packet->payload, rxFrame[i++]);
    paySetType(rx_packet->payload, rxFrame[i++]);
    paySetData(rx_packet->payload, data_size, rxFrame + i);
    radioLqi = rxFrame[i++];
    queuePush(rxQueue, rx_packet);


}

/*****************************************************************************
* Function Name : trxSetSlptr
* Description   : Set the level of the SLP_TR pin.
* Parameters    : val -> 0 for LOW or 1 for HIGH level of the pin
* Return Value  : None                                                     
*****************************************************************************/
static inline void trxSetSlptr(char val) {  	
    SLPTR = val;
    Nop();
    Nop();
}


/*****************************************************************************
* Function Name : trxReadReg                                           
* Description   : Read the value from a register.
* Parameters    : addr - the offset of the register
* Return Value  : register value                                                      
*****************************************************************************/
static unsigned char trxReadReg(unsigned char addr) {
    unsigned char c;
    SPI_CS = 0;
    trxWriteByte(TRX_CMD_RR | addr);
    c = trxReadByte();
    SPI_CS = 1;
    return c;
}


/*****************************************************************************
* Function Name : trxWriteReg                                           
* Description   : Write a value to a register.
* Parameters    : addr 	 the offset of the register
*    	          val 	 the value to be written
* Return Value  : None                                                     
*****************************************************************************/
static void trxWriteReg(unsigned char addr, unsigned char val) {
    SPI_CS = 0;
    trxWriteByte(TRX_CMD_RW | addr);
    trxWriteByte(val);
    SPI_CS = 1;
}



/*****************************************************************************
* Function Name : trxReadBit                                           
* Description   : Read a bit from a register.
* Parameters    : use sub-registers defined in at86rf231.h
* Return Value  : register bit                                                      
*****************************************************************************/
static unsigned char trxReadBit(unsigned char addr,unsigned char mask, unsigned char pos) {
    unsigned char data;
    data = trxReadReg(addr);
    data &= mask;
    data >>= pos;
    return data;
}

/*****************************************************************************
* Function Name : trxWriteBit                                           
* Description   : Write a bit to a register.
* Parameters    : use sub-registers defined in at86rf231.h
* Return Value  : None                                                     
*****************************************************************************/
static void trxWriteBit(unsigned char addr, unsigned char mask, 
                unsigned char pos, unsigned char val) {
    unsigned char temp;
    temp = trxReadReg(addr);
    temp &= ~mask;
    val <<= pos;
    val &= mask;
    val |= temp;
    trxWriteReg(addr, val);
}


/******************************************************************************
* Function Name : trxReadSram                                          
* Description   : Read bytes within frame buffer
* Parameters    : addr - the first position of the bytes 
*                 length - the length of the bytes
* Return Value  :                                                   
******************************************************************************/
/*
static unsigned char trxReadSram(unsigned char addr, unsigned char length) {

    SPI_CS = 0;
    trxWriteByte(0x00);
    trxWriteByte(addr);

    // write code for frame buffer reading....
    
    SPI_CS = 1;
    return 0;

}
*/



/*****************************************************************************
* Function Name : trxWriteSram                                          
* Description   : Write bytes to frame buffer.
* Parameters    : addr - the first position of bytes in frame buffer
*    	          data - the pointer of the data to be written
* Return Value  : None                                                     
*****************************************************************************/
/*
static void trxWriteSram(unsigned char addr, unsigned char* data) {
    SPI_CS = 0;
    trxWriteByte(0x40);
    trxWriteByte(addr);

    // write code for frame buffer writing....

    SPI_CS = 1;
}
*/

/******************************************************************************
* Function Name :   trxReadByte 
* Description   :   This function will read single byte from SPI bus. 
* Parameters    :   None 
* Return Value  :   contents of SPIBUF register                           
******************************************************************************/
static unsigned char trxReadByte(void) {
    SPI_STATbits.SPIROV = 0;
    SPI_BUF = 0x00;     // initiate bus cycle
    while(SPI_STATbits.SPITBF);
    while(!SPI_STATbits.SPIRBF);
    return (SPI_BUF & 0xff);    // return byte read 
}

/******************************************************************************
* Function Name : trxWriteByte
* Description   : This routine writes a single byte to SPI bus.                                 
* Parameters    : Single data byte for SPI bus          
* Return Value  : contents of SPIBUF register
*******************************************************************************/
static unsigned char trxWriteByte(unsigned char dout) {   
    unsigned char c;
    SPI_BUF = dout;   // initiate SPI bus cycle by byte write 
    while(SPI_STATbits.SPITBF);
    while(!SPI_STATbits.SPIRBF);
    c = SPI_BUF;    // read out to avoid overflow 
    return c;
}


/******************************************************************************
* Function Name : trxSetup
* Description   : This routine initialize the transceiver.
* Parameters    : None
* Return Value  : None
*******************************************************************************/

void trxSetup(void) {

    trxSetupPeripheral();

    SPI_CS = 1;     // set chip-select

    // transition to trx_off
    trxWriteReg(RG_TRX_STATE, CMD_FORCE_TRX_OFF); 

    // interrupt at the end of frame send/receive
    trxWriteReg(RG_IRQ_MASK, TRX_IRQ_TRX_END); 

    // automatic CRC generation for tx operation
    trxWriteBit(SR_TX_AUTO_CRC_ON, 1); 

    // no clock on clkm pin
    trxWriteBit(SR_CLKM_CTRL, CLKM_NO_CLOCK); 

    // set default radio channel 
    trxWriteBit(SR_CHANNEL, DEFAULT_CHANNEL);

    // clear any pending iterrupt
    trxReadReg(RG_IRQ_STATUS);

    // set address
    trxWriteReg(RG_SHORT_ADDR_0, (DEFAULT_SRC_ADDR & 0xff));
    trxWriteReg(RG_SHORT_ADDR_1, ((DEFAULT_SRC_ADDR >> 8) & 0xff));
    
    // set PAN ID
    trxWriteReg(RG_PAN_ID_0, (DEFAULT_SRC_PAN_ID & 0xff));
    trxWriteReg(RG_PAN_ID_1, ((DEFAULT_SRC_PAN_ID >> 8) & 0xff));

    // number of attempts until giving up sending a frame sucessfully
    trxWriteBit(SR_MAX_FRAME_RETRIES, 2);  // 3 attempts (2 retries)

    // number of max CSMA attempts until giving up sending a frame
    trxWriteBit(SR_MAX_CSMA_RETRIES, 2);  // 3 attempts (2 retries)

    // transition to rx_on
    trxWriteReg(RG_TRX_STATE,  CMD_RX_AACK_ON); 

}


/******************************************************************************
* Function Name : trxSetupPeripheral
* Description   : This routine sets up SPI bus for this module
* Parameters    : None
* Return Value  : None
*******************************************************************************/
static void trxSetupPeripheral(void) {

    // SPI interrupt is not used.
    //IFS0bits.SPI2IF = 0; // Clear the Interrupt Flag
    
    IEC2bits.SPI2IE = 0; // Disable the Interrupt

    // SPI1CON1 Register Settings
    SPI_CON1bits.MSTEN = 1; // Master mode Enabled
    SPI_CON1bits.DISSCK = 0; // Internal Serial Clock is Enabled
    SPI_CON1bits.DISSDO = 0; // SDOx pin is controlled by the module
    SPI_CON1bits.MODE16 = 0; // Communication is byte-wide (8 bits)
    SPI_CON1bits.SMP = 0; // Input data is sampled at middle of data output time
    SPI_CON1bits.SSEN = 0; // SSx pin is used
    SPI_CON1bits.CKE = 1; // Serial output data changes on transition
                        // from active clock state to idle clock state
    SPI_CON1bits.CKP = 0; // Idle state for clock is a low level;
                            // active state is a high level

    // Set up SCK frequency of 6.667Mhz for 40 MIPS
    SPI_CON1bits.SPRE = 0b010; // Secondary prescale    6:1
    SPI_CON1bits.PPRE = 0b11; // Primary prescale       1:1

    // SPI2CON2 Register Settings
    SPI_CON2 = 0x0000; // Framed SPI2 support disabled

    // SPI2STAT Register Settings
    SPI_STATbits.SPISIDL = 0; // Discontinue module when device enters idle mode
    SPI_STATbits.SPIROV = 0; // Clear Overflow
    SPI_STATbits.SPIEN = 1; // Enable SPI module

}


/******************************************************************************
* Function Name : trxSetupDma
* Description   : This routine sets up DMA read/write for the radio SPI
* Parameters    : None
* Return Value  : None
*******************************************************************************/
void trxSetupDma(void) {

    DMAR_CONbits.AMODE = 0x00;		// Register indirect w/ post-increment
    DMAR_CONbits.MODE = 1;	        // One-Shot mode, ping-pong disabled	
    DMAR_CONbits.DIR = 0;               // Peripheral to RAM transfer direction
    DMAR_CONbits.SIZE = 1;	        // Transfer bytes
    DMAR_CONbits.NULLW = 0;	        // Enable null write
						
    DMAR_REQbits.IRQSEL = DMAxREQ_VAL;	            // SPI Transfer Done Interrupt
    DMAR_PAD = (volatile unsigned int) &SPI_BUF;    // SPI peripheral register
    DMAR_STA= __builtin_dmaoffset(rxFrame);	    // Set to write to RxFrame

    dmarState = DMA_TRX_IDLE;		// Start idle

    IFS2bits.DMAR_IF  = 0;		// Clear DMA interrupt
    IEC2bits.DMAR_IE  = 1;		// Enable DMA interrupt
    DMAR_CONbits.CHEN = 0;		// Disable DMA channel


    DMAW_CONbits.AMODE = 0x00;		// Register indirect w/ post-increment
    DMAW_CONbits.MODE = 1;		// One-Shot mode, ping-pong disabled		    
    DMAW_CONbits.DIR = 1;		// RAM to Peripheral transfer direction
    DMAW_CONbits.SIZE = 1;		// Transfer bytes
    DMAW_CONbits.NULLW = 0;		// No null write
    								
    DMAW_REQbits.IRQSEL = DMAxREQ_VAL;	            // SPI Transfer Done Interrupt			    
    DMAW_PAD = (volatile unsigned int) &SPI_BUF;    // SPI peripheral register
    DMAW_STA= __builtin_dmaoffset(txFrame);	    // Set to read from TxFrame
    
    dmawState = DMA_TRX_IDLE;		// Start idle
    IFS3bits.DMAW_IF  = 0;		// Clear DMA interrupt
    IEC3bits.DMAW_IE  = 1;		// Enable DMA interrupt
    DMAW_CONbits.CHEN = 0;		// Disable DMA Channel	

}


