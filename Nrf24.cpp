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


#include "Nrf24.h"

Nrf24::Nrf24(HardwareSPI *spi_dev, uint8 csn_pin, uint8 ce_pin){		
	_dynamicPayload = false;
	_useInterrupt = false;
	_onPayloadReceiveCallback = NULL;
	
	_port.csn = csn_pin;
	_port.ce = ce_pin;
	_port.spi = spi_dev;
	
	pinMode(_port.csn, OUTPUT);
	pinMode(_port.ce, OUTPUT);
	
	digitalWrite(_port.csn, HIGH);
	digitalWrite(_port.ce, LOW);			
	_port.spi->begin(SPI_9MHZ, MSBFIRST, 0);	
	
}


void Nrf24::begin(Nrf24Mode mode){	
	//By default set pipe0 length for the receiver
	setPipe(0, 32);
	setMode(mode);
}


uint8 Nrf24::getIRQPin(){
	return _port.irq;
}

void Nrf24::onInterrupt(){
	if (!_useInterrupt) return;
	//Avoid triggering in the middle of high to low transition
	delayMicroseconds(2);
	if(digitalRead(_port.irq) == 0){	
		getStatus();
	}
}

void Nrf24::useIRQPin(int8 pin){
	if (pin > 0){
		_useInterrupt = true;
		_port.irq = pin;	
		pinMode(_port.irq, INPUT);
		writeRegister(NRF24_REG_CONFIG, (readRegister(NRF24_REG_CONFIG) & 0x0F));
	} else {
		_useInterrupt = false;		
	}
}


uint8 Nrf24::txDataSent(){	
	if(!_useInterrupt){
		//delayMicroseconds(50);		
		getStatus();
	}
	//Make sure the device is connected properly
	if (_status != 0xFF){
		return (_status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT));
		//writeRegister(NRF24_REG_STATUS, NRF24_STATUS_MAX_RT | NRF24_STATUS_TX_DS);				
	}
	return 0;
}

void Nrf24::attachOnPayloadReceiveCallback(void (*callback)(void)){
	_onPayloadReceiveCallback = callback;
}
 
byte Nrf24::getStatus(){
	//_fifoStatus = readRegister(NRF24_REG_FIFO_STATUS);	
	_status = readRegister(NRF24_REG_STATUS);	
	return _status;	
}

uint8 Nrf24::available(){
	if(!_useInterrupt)		
		getStatus();		
	
	//Make sure the device is connected properly
	if ((_status != 0xFF) && (_status & NRF24_STATUS_RX_DR)){
		uint8 pipe = _status & NRF24_STATUS_RX_P_NO; //range = 0 - 5
		uint8 length;								 //range = 1 - 32		
		if (_dynamicPayload){			
			execCmd(NRF24_CMD_R_RX_PL_WID, NRF24_READ, &length, 1);			
		} else {
			length = readRegister(NRF24_REG_RX_PW_P0 + pipe);
		}		
		return length;
	}
	return 0;
}

void Nrf24::setAutoRetransmit(uint8 count, uint16 delay_us){
	byte value = ((count > 15) ? 15 : count) & 0x0F;
	writeRegister(NRF24_REG_SETUP_RETR, value);		
}

void Nrf24::read(byte *buffer, uint8 length){
	execCmd(NRF24_CMD_R_RX_PAYLOAD, NRF24_READ, buffer, length);
	writeRegister(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);		
}

void Nrf24::send(byte* buffer, uint8 length){
	
	writeRegister(NRF24_REG_STATUS, NRF24_STATUS_MAX_RT | NRF24_STATUS_TX_DS);
	
	digitalWrite(_port.ce, LOW);
	digitalWrite(_port.csn, LOW);
	_port.spi->write(NRF24_CMD_W_TX_PAYLOAD);
	
	while (length--){
	 _port.spi->write(*buffer++);
	}	 
	digitalWrite(_port.csn, HIGH);

	//Toggle CE pin low to high for >10uS to initiate sending from tx buffer
	digitalWrite(_port.ce, HIGH);
	delayMicroseconds(15);
	digitalWrite(_port.ce, LOW);	 
}


void Nrf24::setPayloadSize(uint8 pipe, uint8 length){
	//Pipe starts from 0 to 5
	if (pipe < 6)
		writeRegister(NRF24_REG_RX_PW_P0 + pipe, length);
}

void Nrf24::setPipe(uint8 pipe, bool enable){
	//Pipe starts from 0 to 5
	uint8 val = readRegister(NRF24_REG_EN_RXADDR);	
	if (pipe < 6){
		if (enable)
			val |= (1 << pipe);
		else
			val &= ~(1 << pipe);		
		writeRegister(NRF24_REG_EN_RXADDR, val);
	}
}

void Nrf24::enableDynamicPayload(bool enable){
	if (enable){
		//DPL_P0 must be enabled
		writeRegister(NRF24_REG_DYNPD, 0xFF);

		//Set EN_DPL bit
		writeRegister(NRF24_REG_FEATURE, (1<<2));
	}	
	_dynamicPayload = enable;
}


void Nrf24::flushTx(void){
	execCmd(NRF24_CMD_FLUSH_TX, NRF24_WRITE, 0, 0);
}

void Nrf24::flushRx(void){
	execCmd(NRF24_CMD_FLUSH_RX, NRF24_WRITE, 0, 0);
}

void Nrf24::readRegister(byte reg_address, byte* buffer, uint8 length){
     //Execute R_REGISTER command; Returns maximum 5 bytes
	 length = (length > 5) ? 5 : length;
     execCmd(NRF24_CMD_R_REGISTER + reg_address, NRF24_READ, buffer, length);
}
byte Nrf24::readRegister(byte reg_address){
	//Single register read
	byte buffer;
	execCmd(NRF24_CMD_R_REGISTER + reg_address, NRF24_READ, &buffer, 1);
	return buffer;
	
}
void Nrf24::writeRegister(byte reg_address, byte data){
	//Single register write
    execCmd(NRF24_CMD_W_REGISTER + reg_address, NRF24_WRITE, &data, 1);
}

void Nrf24::writeRegister(byte reg_address, byte* data, uint8 length){
     //Execute W_REGISTER command; Returns maximum 5 bytes
	 length = (length > 5) ? 5 : length;
     execCmd(NRF24_CMD_W_REGISTER + reg_address, NRF24_WRITE, data, length);
}


void Nrf24::execCmd(byte cmd, Nrf24RW read_write, byte* buffer, uint8 length){
	digitalWrite(_port.csn, LOW);	 
	
	//Used "transfer" to skip the first byte (STATUS)
	_port.spi->transfer(cmd);	
	
	if (buffer){
		if (read_write == NRF24_READ){
			while(length--)
				*buffer++ = _port.spi->transfer(0x00);	//Sending dummy byte to generate clock
		} else if (read_write == NRF24_WRITE){
			while(length--)
				_port.spi->write(*buffer++);
		}
	}
	digitalWrite(_port.csn, HIGH);	 
}

void Nrf24::_setBit(byte* reg, uint8 bit){
	*reg |= (1 << bit);
}
void Nrf24::_unsetBit(byte* reg, uint8 bit){
	*reg &= ~(1 << bit);
}

void Nrf24::setMode(Nrf24Mode mode){   
	byte config = readRegister(NRF24_REG_CONFIG); 
	uint8 isPoweredUp = config & NRF24_CONFIG_PWR_UP;	//Initial power status	 
	
	//At first go to standby-I
	digitalWrite(_port.ce, LOW);
	
    if (mode == NRF24_MODE_OFF){
        config &= ~NRF24_CONFIG_PWR_UP;
    } else {
        config |= NRF24_CONFIG_PWR_UP;
        switch (mode){
           case NRF24_MODE_RX :
                config |= NRF24_CONFIG_PRIM_RX;
				writeRegister(NRF24_REG_CONFIG, config);				
				digitalWrite(_port.ce, HIGH);
                break;

           case NRF24_MODE_STANDBY2 :
                flushTx();
				config &= ~NRF24_CONFIG_PRIM_RX;
				writeRegister(NRF24_REG_CONFIG, config);
                digitalWrite(_port.ce, HIGH);
                break;
		   
		   default:
           case NRF24_MODE_STANDBY1 :                
           case NRF24_MODE_TX :
                config &= ~NRF24_CONFIG_PRIM_RX;
				writeRegister(NRF24_REG_CONFIG, config);
                break;				
        }
     }
     
    //  According to datasheet the device needs at least 
	//	1.5mS to go to standby-I from power down mode otherwise 130uS	 
     if(isPoweredUp)
         delayMicroseconds(200);
     else
         delay(2);
}