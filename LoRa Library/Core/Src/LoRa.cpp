
#include "LoRa.h"
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>

//Register Adress
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

//Modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
#define MODE_CAD                 0x07

//Diverse
#define MAX_PKT_LENGHT




//Constructor & destructor
LoRa::LoRa(SPI_HandleTypeDef *hspi_in,UART_HandleTypeDef *huart_in)
{
	hspi = hspi_in;
	huart = huart_in;
}


LoRa::~LoRa()
{

}


//Code

void LoRa::begin()
{
	reset();

	Sleep();

	SetLoRaMode();

	Standby();

	setFrequency();

	setOCP(15);

	setLNA(1); //do nothing

	_IsInitialized = 1;


}

void LoRa::writeRegister(uint8_t address,uint8_t value)
{
	address |= 0x80;
	HAL_GPIO_WritePin(CS_BANK,CS_PIN,GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi,&address, 1,HAL_MAX_DELAY);
	HAL_SPI_Transmit(hspi,&value,1,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_BANK, CS_PIN, GPIO_PIN_SET);
}


uint8_t LoRa::readRegister(uint8_t address)
{
	address &= 0x7F;
	uint8_t data_in;

	HAL_GPIO_WritePin(CS_BANK, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &address, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(hspi, &data_in, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_BANK, CS_PIN, GPIO_PIN_SET);

	return data_in;
}

void LoRa::Burst_write(uint8_t address,uint8_t* data,uint8_t lenght)
{
	address |= 0x80;
	HAL_GPIO_WritePin(CS_BANK, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &address,1,HAL_MAX_DELAY);
	for(uint8_t i=0;i<lenght;i++)
	{
		HAL_SPI_Transmit(hspi, &data[i],1,HAL_MAX_DELAY);
	}
	HAL_GPIO_WritePin(CS_BANK,CS_PIN,GPIO_PIN_SET);
}

void LoRa::_SPISetting()
{
	//For initializing SPI setting (to code later on)
}


void LoRa::Sleep()
{
	if(_IsInitialized==0)
	{
		writeRegister(REG_OP_MODE,MODE_SLEEP);
	}
	else
	{
		uint8_t read_reg = readRegister(REG_OP_MODE);

		writeRegister(REG_OP_MODE,(read_reg & 0xF8)|MODE_SLEEP);
	}

}

void LoRa::Standby()
{
	uint8_t read_reg = readRegister(REG_OP_MODE);

	writeRegister(REG_OP_MODE,(read_reg & 0xF8)|MODE_STDBY);
}

void LoRa::reset()
{
	HAL_GPIO_WritePin(RST_BANK,RST_PIN,GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RST_BANK,RST_PIN,GPIO_PIN_SET);
	HAL_Delay(100);
}

void LoRa::TxMode()
{
	uint8_t read_reg = readRegister(REG_OP_MODE);

	writeRegister(REG_OP_MODE,(read_reg & 0xF8)|MODE_TX);
}

void LoRa::SetLoRaMode()
{
	writeRegister(REG_OP_MODE,MODE_LONG_RANGE_MODE);
}

void LoRa::setFrequency()
{
	//For the moment let to default
}
void LoRa::setPower(uint8_t power)
{
	//default
}

void LoRa::setOCP(uint8_t current)
{
	uint8_t	OcpTrim = 0;

		if(current<45)
			current = 45;
		if(current>240)
			current = 240;

		if(current <= 120)
			OcpTrim = (current - 45)/5;
		else if(current <= 240)
			OcpTrim = (current + 30)/10;
	OcpTrim = OcpTrim + (1<<5);
	writeRegister(REG_OCP, OcpTrim);
}

void LoRa::setLNA(uint8_t gain)
{
	//default for the moment
}

uint8_t LoRa::LoRa_transmit(uint8_t *data,uint8_t length,uint16_t timeout)
{
	//set FifoPtrAddr to base
	uint8_t TxBase = readRegister(REG_FIFO_TX_BASE_ADDR);
	uint8_t IrqFlags;
	writeRegister(REG_FIFO_ADDR_PTR,TxBase);
	writeRegister(REG_PAYLOAD_LENGTH,length);
	Burst_write(REG_FIFO, data, length);
	TxMode();
	while(1)
	{
		IrqFlags = readRegister(REG_IRQ_FLAGS);
		char data[7];
		sprintf(data,"0x%x\r\n",IrqFlags);
		HAL_UART_Transmit(huart,(uint8_t*)data, sizeof(data),100);

		if((IrqFlags & 0x08)!=0)
		{
			writeRegister(REG_IRQ_FLAGS,0xFF);
			return 1;

		}
		else if(--timeout==0)
		{
			Standby();
			return 0;
		}
		HAL_Delay(1);
	}
}

void LoRa::startReceiving()
{
	uint8_t read_reg = readRegister(REG_OP_MODE);

	writeRegister(REG_OP_MODE,(read_reg & 0xF8)|MODE_RX_CONTINUOUS);
}

uint8_t LoRa::LoRa_receive(uint8_t* data,uint8_t length)
{

	uint8_t read;
	uint8_t number_of_bytes;
	uint8_t min = 0;

	for(int i = 0;i<length;i++)
	{
		data[i]=0;
	}

	read = readRegister(REG_IRQ_FLAGS);
	if((read & 0x40)!=0)
	{
		writeRegister(REG_IRQ_FLAGS, 0xFF);
		number_of_bytes = readRegister(REG_RX_NB_BYTES);
		//read = readRegister(REG_FIFO_RX_CURRENT_ADDR);
		writeRegister(REG_FIFO_ADDR_PTR, 0x00);
		min = length >= number_of_bytes ? number_of_bytes : length;
		for(int i=0;i<min;i++)
		{
			data[i] = readRegister(REG_FIFO);
		}
	}

	return min;
}











