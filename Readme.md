NRF24L01+ library for LeafLabs Maple
==============================================

This library provides everything you need to utilize NRF24L01(+) RF module in your project. 


Licence
========
The MIT License

Copyright (c) 2014 Nasir Ahmad.

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use, copy,
modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. 


Installation
============

* Download as zip file and extract
* Copy "Nrf24" to the MAPLE_INSTALLATION_DIR/libraries/ folder


Example
=========

Transmitter
-----------
`````````````````````````````````````````````````````````````````````````````
#include <Nrf24.h>

#define CSN_PIN 26
#define CE_PIN  27
#define IRQ_PIN	24

byte buffer[24];
HardwareSPI spi(2);
uint8 count;

Nrf24 nrf24(&spi, CSN_PIN, CE_PIN);


void nrf24Interrupt(){
	nrf24.onInterrupt();
}

void setup(){
	nrf24.begin(NRF24_MODE_TX);
	
	// Uncomment to use dynamic payload
	// nrf24.enableDynamicPayload(true);
	
	// Uncomment if you want to use IRQ pin
	// nrf24.useIRQPin(IRQ_PIN);
	// attachInterrupt(nrf24.getIRQPin(), nrf24Interrupt, CHANGE);

}

void loop(){
	
	if (SerialUSB.available()){
	
		// Send 24byte at a time from console (max 32)
		for (count=0; SerialUSB.available() && (count < 24); count++){
			buffer[count] = (char) SerialUSB.read();	
		}
		
		// Length of the data is 24 byte
		nrf24.send(buffer, 24);
		
		// Wait until data is sent. Returns 0:none, 1:success, 2:failed
		while(!nrf24.txDataSent());			
	}
	
	// Take a little delay
	delay(100);
}

`````````````````````````````````````````````````````````````````````````````


Receiver
-----------
`````````````````````````````````````````````````````````````````````````````
#include <Nrf24.h>

#define CSN_PIN 26
#define CE_PIN  27
#define IRQ_PIN	24

uint8 payloadSize, count;
byte buffer[24];
HardwareSPI spi(2);
Nrf24 nrf24(&spi, CSN_PIN, CE_PIN);

void nrf24Interrupt(){
	nrf24.onInterrupt();
}

void setup(){
	nrf24.begin(NRF24_MODE_RX);
	
	// Uncomment to use dynamic payload
	// nrf24.enableDynamicPayload(true);
	
	// Uncomment if you want to use IRQ pin
	// nrf24.useIRQPin(IRQ_PIN);
	// attachInterrupt(nrf24.getIRQPin(), nrf24Interrupt, CHANGE);

}

void loop(){	
	//Check whether any data is available	
	payloadSize = nrf24.available();	

	if (payloadSize > 0){
		nrf24.read(buffer, payloadSize);
		for(count = 0; count < payloadSize; count++){
			//Print as character			
			SerialUSB.print((char)buffer[count]);		
		}		
		SerialUSB.println(" ");
	}
	
	// Take a little delay
	delay(50);
}

`````````````````````````````````````````````````````````````````````````````