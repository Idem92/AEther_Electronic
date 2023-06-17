#ifndef LORA_H
#define LORA_H

#include "stm32f7xx_hal.h"


//Config
#define CS_BANK GPIOA
#define CS_PIN GPIO_PIN_4

#define RST_BANK GPIOD
#define RST_PIN GPIO_PIN_15



class LoRa {
public:
	LoRa(SPI_HandleTypeDef *hspi_in,UART_HandleTypeDef *huart_in);
	~LoRa();

	void writeRegister(uint8_t address,uint8_t value);
	uint8_t readRegister(uint8_t address);
	void Burst_write(uint8_t address,uint8_t* data,uint8_t lenght);

	void begin();

	void Sleep();
	void Standby();
	void reset();
	void TxMode();

	void setFrequency();
	void setPower(uint8_t power);
	void setOCP(uint8_t current);
	void setLNA(uint8_t gain);
	uint8_t LoRa_transmit(uint8_t *data,uint8_t length,uint16_t timeout);
	void startReceiving();
	uint8_t LoRa_receive(uint8_t* data,uint8_t length);




private:
	SPI_HandleTypeDef *hspi;
	UART_HandleTypeDef *huart; //for debug

	int _IsInitialized = 0;

	void _SPISetting();
	void SetLoRaMode();


};





#endif
