#ifndef SPI_H
#define SPI_H

#include "stm32f4xx.h"
#include "stm32f446xx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// Struktura konfiguracji SPI
typedef struct {
    SPI_TypeDef* spi;
    DMA_TypeDef* dma;
    uint8_t dmastreamtx;
    uint8_t dmastreamrx;
    uint8_t dmachanneltx;
    uint8_t dmachannelrx;
    uint8_t BRpre;
    GPIO_TypeDef* CSPort;
    uint16_t CSPin;
    GPIO_TypeDef* MISOPort;
    uint16_t MISOPin;
    GPIO_TypeDef* MOSIPort;
    uint16_t MOSIPin;
    GPIO_TypeDef* SCKPort;
    uint16_t SCKPin;
} spi_config_t;

// Struktura obsługi SPI z DMA
typedef struct {
    volatile bool mTXbusy;
    volatile bool mRXbusy;
    const spi_config_t* config; 
    DMA_Stream_TypeDef* mdmastreamtx;
    DMA_Stream_TypeDef* mdmastreamrx;
} spi_t;

// Funkcje publiczne
void spi_init_default(spi_t* spi);
void spi_init_config(spi_t* spi, const spi_config_t* sconf);
void spi_init_peripheral(const spi_t* spi);
void spi_send(const spi_t* spi, uint8_t data);
uint8_t spi_read(const spi_t* spi);
void spi_dma_send(spi_t* spi, const uint8_t* txData, uint16_t size);
void spi_dma_read(spi_t* spi, const uint8_t* rxData, uint16_t size);
void spi_enable_spi_int(const spi_t* spi, uint8_t mode);
bool spi_is_dma_tx_busy(spi_t* spi);
bool spi_is_dma_rx_busy(spi_t* spi);
bool spi_is_txe(const spi_t* spi);
bool spi_is_rxne(const spi_t* spi);
void spi_clear_dma_tx_busy(spi_t* spi);
void spi_clear_dma_rx_busy(spi_t* spi);
void spi_set_cs(const spi_t* spi, bool state);
void spi_enable(const spi_t* spi);
void spi_disable(const spi_t* spi);
IRQn_Type spi_get_irqn_stream_type(const DMA_Stream_TypeDef* dmastream);

// Funkcje pomocnicze (prywatne)
void spi_clear_ifcr(const DMA_Stream_TypeDef* stream);

#endif // SPI_H
