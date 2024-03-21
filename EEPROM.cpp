/*
 * EEPROM.cpp
 *
 *  Created on: Mar. 13, 2024
 *      Author: benja
 */

#include <main.h>
#include "EEPROM.h"

EEPROM_SPI::EEPROM_SPI(SPI_HandleTypeDef *h_spi, uint16_t CSN_Pin, GPIO_TypeDef *CSN_Port,
		uint16_t WP_Pin, GPIO_TypeDef *WP_Port, uint16_t HOLD_Pin, GPIO_TypeDef *HOLD_Port)
{
	pspi = h_spi;
	chipSelectPin = CSN_Pin;
	chipSelectPort = CSN_Port;
	writeProtectPin = WP_Pin;
	writeProtectPort = WP_Port;
	holdPin = HOLD_Pin;
	holdPort = HOLD_Port;

	HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET); //Disabling hold.
	HAL_GPIO_WritePin(wirteProtect, writeProtectPin, GPIO_PIN_RESET);  //Enabling write protect

}

void EEPROM_SPI::EEPROM_SPItransaction() //SPI transaction function for EEPROM memory
{
	//Setting chip-select pin low to enable communication.
	HAL_GPIO_WritePin(CSN_EEPROM_GPIO_Port, CSN_EEPROM_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(pspi, send_array, rec_array, EEPROM_DATA_SIZE, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CSN_EEPROM_GPIO_Port, CSN_EEPROM_Pin, GPIO_PIN_SET);
	//Setting the chip-select pin high to disable communication.

}

void EEPROM_SPI::EEPROM_Write_Enable()
{ //Write enable, must be done before all Write_Reg and Write_Status_reg

	//Disabling write protect to change the status register
	HAL_GPIO_WritePin(writeProtectPort, writeProtectPin, GPIO_PIN_SET);
	send_array[0] = EEPROM_Instr::WREN;//OPcode Write enable command 0x06
	//Processing the SPI communication
	EEPROM_SPItransaction();
}

void EEPROM_SPI::EEPROM_write(uint16_t address, uint32_t data)
{	// Writes to the EEPROM given a 16-bit starting address and 4-byte array

	uint8_t data_array[EEPROM_DATA_SIZE];
	data_array[0] = ((data & 0xFF000000) >> 24);
	data_array[1] = ((data & 0xFF0000) >> 16);
	data_array[2] = ((data & 0xFF00) >> 8);
	data_array[3] = ((data & 0xFF));

	for(int i = 0; i < 4; i++)
	{
		EEPROM_Write_Enable();

		send_array[0] = EEPROM_Instr::WRIT;//OPcode WRITE command 0x02
		send_array[1] = (((address + i) & 0xFF0000) >> 16); //Write Addr high byte (A15- A8)
		send_array[2] = (((address + i) & 0xFF00) >> 8); //Write Addr low byte (A7- A0)
		send_array[3] = data_array[i]; //Byte to be written to with data

		//Processing the SPI communication
		EEPROM_SPItransaction();
	}
}

uint32_t EEPROM_SPI::EEPROM_read(uint16_t address){
	uint32_t value = 0;

	send_array[0] = EEPROM_Instr::READ;//OPcode READ command 0x03
	send_array[1] = ((address & 0xFF0000) >> 16); //Write Addr high byte (A15- A8)
	send_array[2] = ((address & 0xFF00) >> 8); //Write Addr low byte (A7- A0)

	//Setting chip-select pin low to enable communication.
	HAL_GPIO_WritePin(CSN_EEPROM_GPIO_Port, CSN_EEPROM_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(pspi, send_array, EEPROM_DATA_SIZE, HAL_MAX_DELAY);



	HAL_SPI_Receive(pspi, rec_array, EEPROM_DATA_SIZE, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(CSN_EEPROM_GPIO_Port, CSN_EEPROM_Pin, GPIO_PIN_SET);
	//Setting the chip-select pin high to disable communication.

	value |= (uint32_t)rec_array[1] << 24;
	value |= (uint32_t)rec_array[2] << 16;
	value |= (uint32_t)rec_array[3] << 8;
	value |= (uint32_t)rec_array[4];


	return value;
}
void EEPROM_SPI::EEPROM_protect(){
	EEPROM_Write_Enable();
	send_array[0] = EEPROM_Instr::WRSR;//OPcode Write Status Register command 0x01
	send_array[1] = 0x0C; //Write 0b00001100 bit 2 and 3 high for full memory protect, WPEN is low

	//Processing the SPI communication
	EEPROM_SPItransaction();
	HAL_GPIO_WritePin(writeProtectPort, writeProtectPin, GPIO_PIN_RESET); //Enabling write protect

}
void EEPROM_SPI::EEPROM_expose(){
	EEPROM_Write_Enable();
	send_array[0] = EEPROM_Instr::WRSR;//OPcode Write Status Register command 0x01
	send_array[1] = 0x00; //Write 0b00000000 bit 2 and 3 high for full memory protect, WPEN is low

	6//Processing the SPI communication
	EEPROM_SPItransaction();

}


