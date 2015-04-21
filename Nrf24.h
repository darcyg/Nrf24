/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2014 Nasir Ahmad.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 *  @brief NRF24L01+ RF module library for LeafLabs Maple
 */
 
 
#ifndef _NRF24_H_
#define _NRF24_H_


#include "libmaple_types.h"
#include "HardwareSPI.h"
#include "io.h"
#include "delay.h"
#include "wirish.h"


/* NRF24 Commands */
#define NRF24_CMD_R_REGISTER         0x00
#define NRF24_CMD_W_REGISTER         0x20
#define NRF24_CMD_R_RX_PAYLOAD       0x61
#define NRF24_CMD_W_TX_PAYLOAD       0xA0
#define NRF24_CMD_FLUSH_TX           0xE1
#define NRF24_CMD_FLUSH_RX           0xE2
#define NRF24_CMD_REUSE_TX_PL        0xE3
#define NRF24_CMD_R_RX_PL_WID        0x60
#define NRF24_CMD_W_ACK_PAYLOAD      0xA8
#define NRF24_CMD_W_TX_PAYLOAD_NOACK 0xB0
#define NRF24_CMD_NOP                0xFF
#define NRF24_DUMMY_BYTE         	 0x00


/* NRF24 Register addresses */
#define  NRF24_REG_CONFIG       0x00
#define  NRF24_REG_EN_AA        0x01
#define  NRF24_REG_EN_RXADDR    0x02
#define  NRF24_REG_SETUP_AW     0x03
#define  NRF24_REG_SETUP_RETR   0x04
#define  NRF24_REG_RF_CH        0x05
#define  NRF24_REG_RF_SETUP     0x06
#define  NRF24_REG_STATUS       0x07
#define  NRF24_REG_OBSERVE_TX   0x08
#define  NRF24_REG_CD           0x09
#define  NRF24_REG_RX_ADDR_P0   0x0A
#define  NRF24_REG_RX_ADDR_P1   0x0B
#define  NRF24_REG_RX_ADDR_P2   0x0C
#define  NRF24_REG_RX_ADDR_P3   0x0D
#define  NRF24_REG_RX_ADDR_P4   0x0E
#define  NRF24_REG_RX_ADDR_P5   0x0F
#define  NRF24_REG_TX_ADDR      0x10
#define  NRF24_REG_RX_PW_P0     0x11
#define  NRF24_REG_RX_PW_P1     0x12
#define  NRF24_REG_RX_PW_P2     0x13
#define  NRF24_REG_RX_PW_P3     0x14
#define  NRF24_REG_RX_PW_P4     0x15
#define  NRF24_REG_RX_PW_P5     0x16
#define  NRF24_REG_FIFO_STATUS  0x17
#define  NRF24_REG_DYNPD        0x1C
#define  NRF24_REG_FEATURE      0x1D



/* NRF24 CONFIG register bits */
#define  NRF24_CONFIG_PRIM_RX		(1 << 0)
#define  NRF24_CONFIG_PWR_UP		(1 << 1)
#define  NRF24_CONFIG_CRCO			(1 << 2)
#define  NRF24_CONFIG_EN_CRC		(1 << 3)
#define  NRF24_CONFIG_MASK_MAX_RT	(1 << 4)
#define  NRF24_CONFIG_MASK_TX_DS	(1 << 5)
#define  NRF24_CONFIG_MASK_RX_DR	(1 << 6)


/* Nrf24 STATUS register bits */
#define  NRF24_STATUS_TX_FULL   (1 << 0)
#define  NRF24_STATUS_RX_P_NO   ((1 << 1) | (1 << 2) | (1 << 3))
#define  NRF24_STATUS_MAX_RT    (1 << 4)
#define  NRF24_STATUS_TX_DS     (1 << 5)
#define  NRF24_STATUS_RX_DR     (1 << 6)


/* Nrf24 FIFO_STATUS register bits */
#define  NRF24_FIFO_STATUS_RX_EMPTY		(1 << 0)
#define  NRF24_FIFO_STATUS_RX_FULL		(1 << 1)
#define  NRF24_FIFO_STATUS_TX_EMPTY		(1 << 4)
#define  NRF24_FIFO_STATUS_TX_FULL		(1 << 5)
#define  NRF24_FIFO_STATUS_TX_REUSE		(1 << 6)

typedef enum {
	PRIM_RX		= 0,
	PWR_UP		= 1,
	CRCO		= 2,
	EN_CRC	 	= 3,
	MASK_MAX_RT = 4,
	MASK_TX_DS	= 5,
	MASK_RX_DR	= 6
} NRF24_CONFIG;

typedef enum {
	NRF24_MODE_TX, 
	NRF24_MODE_RX, 
	NRF24_MODE_STANDBY1, 
	NRF24_MODE_STANDBY2, 
	NRF24_MODE_OFF
} Nrf24Mode;

typedef enum {
	NRF24_DR_256KBPS, 
	NRF24_DR_1MBPS, 
	NRF24_DR_2MBPS
} Nrf24DataRate;
	
typedef enum{
	NRF24_WRITE = 0,
	NRF24_READ  = 1
} Nrf24RW;

typedef enum{
	NRF24_TX_SENDING = 0,
	NRF24_TX_SENT,
	NRF24_TX_FAILED,
	NRF24_TX_FIFO_FULL,
	NRF24_TX_FIFO_EMPTY,
} Nrf24TxStatus;

class Nrf24 {	
	public:
		typedef	struct {
			HardwareSPI* 	spi;
			uint8			csn;
			uint8			ce;
			uint8			irq;
		} Port;	
		
		Nrf24(HardwareSPI*, uint8, uint8);
		void begin(Nrf24Mode);
		
		/* SPI functions */
		void readRegister(byte, byte*, uint8);
		byte readRegister(byte);
		void writeRegister(byte, byte*, uint8);
		void writeRegister(byte, uint8);
		
		/* Configure device */
		void setAutoRetransmit(uint8, uint16);
		void setDateRate(Nrf24DataRate);
		void setPayloadSize(uint8, uint8);
		void setPipe(uint8, bool);
		void enableDynamicPayload(bool);
		
		/* Data send/receive */
		void send(byte*, uint8);	
		void read(byte*, uint8);
		uint8 txDataSent();
		uint8 available();
		
		void execCmd(byte, Nrf24RW, byte*, uint8);
		void flushTx(void);
		void flushRx(void);
		void setMode(Nrf24Mode);
		
		/* Interrupt */
		void onInterrupt();		
		uint8 getIRQPin();
		void useIRQPin(int8);		
		void attachOnPayloadReceiveCallback(void (*)(void));
		byte getStatus();
		Port _port;
		
	private:

		volatile bool _dynamicPayload;
		volatile bool _useInterrupt;		
		volatile byte _status;		
		void (*_onPayloadReceiveCallback)(void);
		void _setBit(byte*, uint8);
		void _unsetBit(byte*, uint8);		
};


#endif