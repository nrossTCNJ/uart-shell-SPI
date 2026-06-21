/*
 *  mfrc522.c
 *	File to startup and control the MFRC522
 *  Created on: Jun 18, 2026
 *      Author: nate
 */

#include "mfrc522.h"
#include <stdio.h>

uint8_t mfrc522_init(void)
{
	GPIOC->BSRR = (1 << 23);	// hold RST LOW
	HAL_Delay(1);
	GPIOC->BSRR = (1 << 7);		// set RST high
	HAL_Delay(50);				// allow oscillator to stabilize

	uint8_t result;

	uint8_t status = mfrc522_read_reg(VERSION_REG, &result);
	if(status == SPI_ERROR)
		return SPI_ERROR;

	// make sure chip is working
	if(result == 0x00 || result == 0xFF)
		return SPI_ERROR;

	// start antenna drivers
	uint8_t txcr_val = 0;
	if(mfrc522_read_reg(TX_CTRL_REG, &txcr_val) == SPI_ERROR)
		return SPI_ERROR;

	mfrc522_write_reg(TX_CTRL_REG, txcr_val | 0b11); // set bits 0 and 1
	return SPI_OK;

}

uint8_t mfrc522_read_uid(uint8_t *uid)
{
	uint8_t atqa;

	if(mfrc522_detect_card(&atqa) == SPI_ERROR)	// send REQA byte
		return SPI_ERROR;

	mfrc522_write_reg(COMMAND_REG, 0x00);		// idle
	mfrc522_write_reg(COM_IRQ_REG, 0x7F);		// clear IRQ flags
	mfrc522_write_reg(FIFO_LEVEL_REG, 0x80);	// flush FIFO
	mfrc522_write_reg(FIFO_DATA_REG, 0x93);		// anticollision command
	mfrc522_write_reg(FIFO_DATA_REG, 0x20);		// anticollision command
	mfrc522_write_reg(COMMAND_REG, 0x0C);		// transceive
	mfrc522_write_reg(BITFRAME_REG, 0x00);		// full 8-bit frame
	mfrc522_write_reg(BITFRAME_REG, 0x80);		// start transmission

	uint32_t timeout = 10000;
	uint8_t val = 0;
	while(!(val & (1 << 5))) 					// wait for RxIRq set
	{
		if(timeout-- == 0)
			return SPI_ERROR;
		if(mfrc522_read_reg(COM_IRQ_REG, &val) == SPI_ERROR)
			return SPI_ERROR;
	}

	uint8_t bcc;								// FIFO read
	for(int i = 0; i < 5; i++)
	{
		if(i < 4)
			mfrc522_read_reg(FIFO_DATA_REG, &uid[i]);
		else
			mfrc522_read_reg(FIFO_DATA_REG, &bcc);
	}

	if((uid[0] ^ uid[1] ^ uid[2] ^ uid[3]) != bcc) // bit check
		return SPI_ERROR;
	return SPI_OK;
}

uint8_t mfrc522_detect_card(uint8_t *atqa)
{
	mfrc522_write_reg(COMMAND_REG, 0x00);		// idle
	mfrc522_write_reg(COM_IRQ_REG, 0x7F);		// clear IRQ flags
	mfrc522_write_reg(FIFO_LEVEL_REG, 0x80);	// flush FIFO
	mfrc522_write_reg(FIFO_DATA_REG, 0x26);		// load REQA byte
	mfrc522_write_reg(COMMAND_REG, 0x0C);		// transceive
	mfrc522_write_reg(BITFRAME_REG, 0x87);		// start transmission

	uint32_t timeout = 10000;
	uint8_t val = 0;
	while(!(val & (1 << 5))) 				// wait for RxIRq set
	{
		if(timeout-- == 0)
			return SPI_ERROR;
		if(mfrc522_read_reg(COM_IRQ_REG, &val) == SPI_ERROR)
			return SPI_ERROR;
	}

	if(mfrc522_read_reg(FIFO_DATA_REG, atqa) == SPI_ERROR)
		return SPI_ERROR;


	return SPI_OK;
}

uint8_t mfrc522_read_reg(uint8_t reg, uint8_t *result)
{
	cs_low();					// pull CS low

	uint8_t rx_dummy;

	// transfer address byte
	// read: shift left, clear bit 0, set bit 7
	uint8_t status = spi1_transfer( ((reg << 1) & 0x7E) | 0x80, &rx_dummy);
	if(status == SPI_ERROR)
	{
		cs_high();				// pull CS high
		return SPI_ERROR;
	}

	// receive data byte
	status = spi1_receive(result);
	if(status == SPI_ERROR)
	{
		cs_high();				// pull CS high
		return SPI_ERROR;
	}

	cs_high();					// pull CS high
	return SPI_OK;
}

uint8_t mfrc522_write_reg(uint8_t reg, uint8_t value)
{
	cs_low();					// pull CS low

	uint8_t rx_dummy;

	// transfer address byte
	// write: shift left, clear bits 0 and 7
	uint8_t status = spi1_transfer((reg << 1) & 0x7E, &rx_dummy);
	if(status == SPI_ERROR)
	{
		cs_high();
		return SPI_ERROR;
	}

	// send data byte
	status = spi1_transfer(value, &rx_dummy);
	if(status == SPI_ERROR)
	{
		cs_high();
		return SPI_ERROR;
	}

	cs_high();
	return SPI_OK;
}

