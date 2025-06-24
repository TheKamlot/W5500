#include "spi.h"

// Domyślna konfiguracja (statyczna)
static const spi_config_t default_config = {
    .spi = SPI1,
    .dma = DMA2,
    .dmastreamtx = 3,
    .dmastreamrx = 2,
    .dmachanneltx = 3,
    .dmachannelrx = 3,
    .BRpre = 2,
    .CSPort = GPIOA,
    .CSPin = 15,
    .MISOPort = GPIOA,
    .MISOPin = 6,
    .MOSIPort = GPIOA,
    .MOSIPin = 7,
    .SCKPort = GPIOA,
    .SCKPin = 5
};

// Inicjalizacja z domyślnymi parametrami
void spi_init_default(spi_t* spi) {
    spi->mTXbusy = false;
    spi->mRXbusy = false;
    spi->config = &default_config;
    
    // Obliczenie adresów strumieni DMA
    spi->mdmastreamtx = (DMA_Stream_TypeDef*)((uint32_t)spi->config->dma + 0x10 + 0x18 * spi->config->dmastreamtx);
    spi->mdmastreamrx = (DMA_Stream_TypeDef*)((uint32_t)spi->config->dma + 0x10 + 0x18 * spi->config->dmastreamrx);
}

// Inicjalizacja z konfiguracją
void spi_init_config(spi_t* spi, const spi_config_t* sconf) {
    spi->mTXbusy = false;
    spi->mRXbusy = false;
    
    // Walidacja SPI
    if (sconf->spi == SPI1 || sconf->spi == SPI2 || sconf->spi == SPI3) {
        // OK
    } else {
        printf("Wrong SPI");
        __disable_irq();
        while(1){};
    }
    
    // Walidacja DMA
    if (sconf->dma == DMA1 || sconf->dma == DMA2) {
        // OK
    } else {
        printf("Wrong DMA");
        __disable_irq();
        while(1){};
    }
    
    // Walidacja kanałów i strumieni DMA
    bool conds1 = (sconf->spi == SPI1) && (sconf->dma == DMA2) && 
                  ((sconf->dmastreamrx == 0) || (sconf->dmastreamrx == 2)) &&
                  ((sconf->dmastreamtx == 3) || (sconf->dmastreamtx == 7)) &&
                  (sconf->dmachanneltx == 3) && (sconf->dmachannelrx == 3);
    
    bool conds2 = (sconf->spi == SPI2) && (sconf->dma == DMA1) &&
                  (sconf->dmastreamrx == 3) && (sconf->dmastreamtx == 5) &&
                  (sconf->dmachanneltx == 0) && (sconf->dmachannelrx == 0);
    
    bool conds3 = (sconf->spi == SPI3) && (sconf->dma == DMA1) &&
                  ((sconf->dmastreamrx == 0) || (sconf->dmastreamrx == 2)) &&
                  ((sconf->dmastreamtx == 5) || (sconf->dmastreamtx == 6)) &&
                  (sconf->dmachanneltx == 0) && (sconf->dmachannelrx == 0);
    
    if (!(conds1 || conds2 || conds3)) {
        printf("Wrong DMA channel/streams");
        __disable_irq();
        while(1){};
    }
    
    // Walidacja preskalera
    if (sconf->spi == SPI1) {
        if (sconf->BRpre > 8) {
            printf("Wrong BoudRatePrescaler Value");
            __disable_irq();
            while(1){};
        }
    } else if (sconf->spi == SPI2 || sconf->spi == SPI3) {
        if (sconf->BRpre > 8) {
            printf("Wrong BoudRatePrescaler Value");
            __disable_irq();
            while(1){};
        }
    }
    
    // Walidacja GPIO CS
    if (!(sconf->CSPort == GPIOA || sconf->CSPort == GPIOB || 
          sconf->CSPort == GPIOC || sconf->CSPort == GPIOD)) {
        printf("Wrong GPIO port");
        __disable_irq();
        
    }
    
    if (sconf->CSPin >= 16) {
        printf("Wrong GPIO pin");
        __disable_irq();
        while(1){};
    }
    
    // Walidacja MISO
    bool misocondition1 = (sconf->spi == SPI1) && 
                         (((sconf->MISOPort == GPIOA) && (sconf->MISOPin == 6)) ||
                          ((sconf->MISOPort == GPIOB) && (sconf->MISOPin == 4)));
    
    bool misocondition2 = (sconf->spi == SPI2) &&
                         (((sconf->MISOPort == GPIOC) && (sconf->MISOPin == 2)) ||
                          ((sconf->MISOPort == GPIOB) && (sconf->MISOPin == 14)));
    
    bool misocondition3 = (sconf->spi == SPI3) &&
                         (((sconf->MISOPort == GPIOD) && (sconf->MISOPin == 6)) ||
                          ((sconf->MISOPort == GPIOB) && (sconf->MISOPin == 4)) ||
                          ((sconf->MISOPort == GPIOC) && (sconf->MISOPin == 11)));
    
    if (!(misocondition1 || misocondition2 || misocondition3)) {
        printf("Wrong MISO port");
        __disable_irq();
        while(1){};
    }
    
    // Walidacja MOSI
    bool mosicondition1 = (sconf->spi == SPI1) && 
                         (((sconf->MOSIPort == GPIOA) && (sconf->MOSIPin == 7)) ||
                          ((sconf->MOSIPort == GPIOB) && (sconf->MOSIPin == 5)));
    
    bool mosicondition2 = (sconf->spi == SPI2) &&
                         (((sconf->MOSIPort == GPIOC) && (sconf->MOSIPin == 1)) ||
                          ((sconf->MOSIPort == GPIOB) && (sconf->MOSIPin == 15)) ||
                          ((sconf->MOSIPort == GPIOC) && (sconf->MOSIPin == 3)));
    
    bool mosicondition3 = (sconf->spi == SPI3) &&
                         (((sconf->MOSIPort == GPIOD) && (sconf->MOSIPin == 0)) ||
                          ((sconf->MOSIPort == GPIOB) && (sconf->MOSIPin == 5)) ||
                          ((sconf->MOSIPort == GPIOC) && (sconf->MOSIPin == 12)) ||
                          ((sconf->MOSIPort == GPIOC) && (sconf->MOSIPin == 1)));
    
    if (!(mosicondition1 || mosicondition2 || mosicondition3)) {
        printf("Wrong MOSI port");
        __disable_irq();
        while(1){};
    }
    
    // Walidacja SCK
    bool sckcondition1 = (sconf->spi == SPI1) && 
                        (((sconf->SCKPort == GPIOA) && (sconf->SCKPin == 5)) ||
                         ((sconf->SCKPort == GPIOB) && (sconf->SCKPin == 3)));
    
    bool sckcondition2 = (sconf->spi == SPI2) &&
                        (((sconf->SCKPort == GPIOB) && (sconf->SCKPin == 10)) ||
                         ((sconf->SCKPort == GPIOB) && (sconf->SCKPin == 13)) ||
                         ((sconf->SCKPort == GPIOC) && (sconf->SCKPin == 7)) ||
                         ((sconf->SCKPort == GPIOD) && (sconf->SCKPin == 3)));
    
    bool sckcondition3 = (sconf->spi == SPI3) &&
                        (((sconf->SCKPort == GPIOB) && (sconf->SCKPin == 3)) ||
                         ((sconf->SCKPort == GPIOC) && (sconf->SCKPin == 10)));
    
    if (!(sckcondition1 || sckcondition2 || sckcondition3)) {
        printf("Wrong SCK port");
        __disable_irq();
        while(1){};
    }
    
    // Przypisanie konfiguracji
    spi->config = sconf;
    
    // Obliczenie adresów strumieni DMA
    spi->mdmastreamtx = (DMA_Stream_TypeDef*)((uint32_t)spi->config->dma + 0x10 + 0x18 * spi->config->dmastreamtx);
    spi->mdmastreamrx = (DMA_Stream_TypeDef*)((uint32_t)spi->config->dma + 0x10 + 0x18 * spi->config->dmastreamrx);
}

// Inicjalizacja peryferiów
void spi_init_peripheral(const spi_t* spi) {
    // GPIO INIT
    if (spi->config->CSPort == GPIOA || spi->config->MISOPort == GPIOA || 
        spi->config->MOSIPort == GPIOA || spi->config->SCKPort == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    }
    if (spi->config->CSPort == GPIOB || spi->config->MISOPort == GPIOB || 
        spi->config->MOSIPort == GPIOB || spi->config->SCKPort == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    }
    if (spi->config->CSPort == GPIOC || spi->config->MISOPort == GPIOC || 
        spi->config->MOSIPort == GPIOC || spi->config->SCKPort == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    }
    if (spi->config->CSPort == GPIOD || spi->config->MISOPort == GPIOD || 
        spi->config->MOSIPort == GPIOD || spi->config->SCKPort == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    }
    
    // CS Default Value
    spi->config->CSPort->BSRR = (1 << spi->config->CSPin);
    
    // CS
    spi->config->CSPort->MODER &= ~(0x3 << (spi->config->CSPin * 2));
    spi->config->CSPort->MODER |= (0x1 << (spi->config->CSPin * 2));
    spi->config->CSPort->OTYPER &= ~(0x1 << spi->config->CSPin);
    spi->config->CSPort->OSPEEDR |= (0x3 << (spi->config->CSPin * 2));
    spi->config->CSPort->PUPDR &= ~(0x3 << (spi->config->CSPin * 2));
    
    // MISO
    spi->config->MISOPort->MODER &= ~(0x3 << (spi->config->MISOPin * 2));
    spi->config->MISOPort->MODER |= (0x2 << (spi->config->MISOPin * 2));
    spi->config->MISOPort->OSPEEDR |= (0x3 << (spi->config->MISOPin * 2));
    spi->config->MISOPort->PUPDR &= ~(0x3 << (spi->config->MISOPin * 2));
    spi->config->MISOPort->OTYPER &= ~(0x1 << spi->config->MISOPin);
    
    if (spi->config->MISOPin < 8) {
        spi->config->MISOPort->AFR[0] &= ~(0xf << (spi->config->MISOPin * 4));
        spi->config->MISOPort->AFR[0] |= (0x5 << (spi->config->MISOPin * 4));
    } else {
        spi->config->MISOPort->AFR[1] &= ~(0xf << ((spi->config->MISOPin - 8) * 4));
        spi->config->MISOPort->AFR[1] |= (0x5 << ((spi->config->MISOPin - 8) * 4));
    }
    
    // MOSI
    spi->config->MOSIPort->MODER &= ~(0x3 << (spi->config->MOSIPin * 2));
    spi->config->MOSIPort->MODER |= (0x2 << (spi->config->MOSIPin * 2));
    spi->config->MOSIPort->OSPEEDR |= (0x3 << (spi->config->MOSIPin * 2));
    spi->config->MOSIPort->PUPDR &= ~(0x3 << (spi->config->MOSIPin * 2));
    spi->config->MOSIPort->OTYPER &= ~(0x1 << spi->config->MOSIPin);
    
    if (spi->config->MOSIPin < 8) {
        spi->config->MOSIPort->AFR[0] &= ~(0xf << (spi->config->MOSIPin * 4));
        spi->config->MOSIPort->AFR[0] |= (0x5 << (spi->config->MOSIPin * 4));
    } else {
        spi->config->MOSIPort->AFR[1] &= ~(0xf << ((spi->config->MOSIPin - 8) * 4));
        spi->config->MOSIPort->AFR[1] |= (0x5 << ((spi->config->MOSIPin - 8) * 4));
    }
    
    // SCK
    spi->config->SCKPort->MODER &= ~(0x3 << (spi->config->SCKPin * 2));
    spi->config->SCKPort->MODER |= (0x2 << (spi->config->SCKPin * 2));
    spi->config->SCKPort->OSPEEDR |= (0x3 << (spi->config->SCKPin * 2));
    spi->config->SCKPort->PUPDR &= ~(0x3 << (spi->config->SCKPin * 2));
    spi->config->SCKPort->OTYPER &= ~(0x1 << spi->config->SCKPin);
    
    if (spi->config->SCKPin < 8) {
        spi->config->SCKPort->AFR[0] &= ~(0xf << (spi->config->SCKPin * 4));
        spi->config->SCKPort->AFR[0] |= (0x5 << (spi->config->SCKPin * 4));
    } else {
        spi->config->SCKPort->AFR[1] &= ~(0xf << ((spi->config->SCKPin - 8) * 4));
        spi->config->SCKPort->AFR[1] |= (0x5 << ((spi->config->SCKPin - 8) * 4));
    }
    
    // SPI INIT
    if (spi->config->spi == SPI1) {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    } else if (spi->config->spi == SPI2) {
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    } else if (spi->config->spi == SPI3) {
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    }
    
    spi->config->spi->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    spi->config->spi->CR1 |= (spi->config->BRpre << SPI_CR1_BR_Pos);
    spi->config->spi->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
    spi->config->spi->CR1 |= SPI_CR1_SPE;
    
    // DMA INIT
    if (spi->config->dma == DMA1) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    } else if (spi->config->dma == DMA2) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    }
    
    // RX TX CHANNELS
    spi->mdmastreamtx->CR &= ~DMA_SxCR_EN;
    spi->mdmastreamtx->CR &= ~DMA_SxCR_CHSEL;
    spi->mdmastreamtx->CR |= (spi->config->dmachanneltx << DMA_SxCR_CHSEL_Pos);
    
    spi->mdmastreamrx->CR &= ~DMA_SxCR_CHSEL;
    spi->mdmastreamrx->CR |= (spi->config->dmachannelrx << DMA_SxCR_CHSEL_Pos);
    
    // INTERRUPTS
    spi->mdmastreamtx->CR |= DMA_SxCR_TCIE;
    spi->mdmastreamrx->CR |= DMA_SxCR_TCIE;
    
    IRQn_Type txIRQ = spi_get_irqn_stream_type(spi->mdmastreamtx);
    IRQn_Type rxIRQ = spi_get_irqn_stream_type(spi->mdmastreamrx);
    
    NVIC_EnableIRQ(txIRQ);
    NVIC_SetPriority(txIRQ, 2);
    NVIC_EnableIRQ(rxIRQ);
    NVIC_SetPriority(rxIRQ, 2);
    
    spi_clear_ifcr(spi->mdmastreamtx);
    spi_clear_ifcr(spi->mdmastreamrx);
}

// Wysyłanie pojedynczego bajtu
void spi_send(const spi_t* spi, const uint8_t data) {
    while (!(spi->config->spi->SR & SPI_SR_TXE)){};
    *(volatile uint8_t*)&spi->config->spi->DR = data;
    while (!(spi->config->spi->SR & SPI_SR_RXNE)){};
    while (spi->config->spi->SR & SPI_SR_BSY){};
    (void)(*(volatile uint8_t*)&spi->config->spi->DR);
}

// Odczyt pojedynczego bajtu
uint8_t spi_read(const spi_t* spi) {
    *(volatile uint8_t*)&spi->config->spi->DR = 0x00;
    while (!(spi->config->spi->SR & SPI_SR_TXE)){};
    while (!(spi->config->spi->SR & SPI_SR_RXNE)){};
    while (spi->config->spi->SR & SPI_SR_BSY){};
    return (*(volatile uint8_t*)&spi->config->spi->DR);
}

// Transmisja z DMA
void spi_dma_send(spi_t* spi, const uint8_t* txData, const uint16_t size) {
    while (spi->mTXbusy){};
    spi->mTXbusy = true;
    
    spi->mdmastreamtx->CR &= ~DMA_SxCR_EN;
    while (spi->mdmastreamtx->CR & DMA_SxCR_EN){};
    
    spi->mdmastreamtx->M0AR = (uint32_t)txData;
    spi->mdmastreamtx->PAR = (uint32_t)&spi->config->spi->DR;
    spi->mdmastreamtx->NDTR = size;
    spi->mdmastreamtx->CR |= DMA_SxCR_DIR_0;
    spi->mdmastreamtx->CR |= DMA_SxCR_MINC;
    spi->mdmastreamtx->CR &= ~DMA_SxCR_PINC;
    spi->mdmastreamtx->CR |= DMA_SxCR_TCIE;
    spi->mdmastreamtx->CR |= DMA_SxCR_EN;
    
    spi_set_cs(spi, true);
    spi->config->spi->CR2 |= SPI_CR2_TXDMAEN;
}

// Odbiór z DMA
void spi_dma_read(spi_t* spi, const uint8_t* rxData, const uint16_t size) {
    while (spi->mRXbusy){};
    spi->mRXbusy = true;
    
    static uint8_t txData = 0x00;
    
    spi->mdmastreamrx->CR &= ~DMA_SxCR_EN;
    while (spi->mdmastreamrx->CR & DMA_SxCR_EN){};
    
    spi->mdmastreamtx->CR &= ~DMA_SxCR_EN;
    while (spi->mdmastreamtx->CR & DMA_SxCR_EN){};
    
    // DMA RX configuration
    spi->mdmastreamrx->M0AR = (uint32_t)rxData;
    spi->mdmastreamrx->PAR = (uint32_t)&spi->config->spi->DR;
    spi->mdmastreamrx->NDTR = size;
    spi->mdmastreamrx->CR &= ~DMA_SxCR_DIR;
    spi->mdmastreamrx->CR |= DMA_SxCR_MINC;
    spi->mdmastreamrx->CR &= ~DMA_SxCR_PINC;
    
    // DMA TX configuration (dummy bytes)
    spi->mdmastreamtx->M0AR = (uint32_t)&txData;
    spi->mdmastreamtx->PAR = (uint32_t)&spi->config->spi->DR;
    spi->mdmastreamtx->NDTR = size;
    spi->mdmastreamtx->CR |= DMA_SxCR_DIR_0;
    spi->mdmastreamtx->CR &= ~DMA_SxCR_MINC;
    spi->mdmastreamtx->CR &= ~DMA_SxCR_PINC;
    spi->mdmastreamtx->CR &= ~DMA_SxCR_TCIE;
    
    spi->mdmastreamtx->CR |= DMA_SxCR_EN;
    spi->mdmastreamrx->CR |= DMA_SxCR_EN;
    
    spi->config->spi->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
    spi_set_cs(spi, true);
}

// Włączenie przerwań SPI
void spi_enable_spi_int(const spi_t* spi, const uint8_t mode) {
    switch (mode) {
        case 0:
            spi->config->spi->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
            break;
        case 1:
            spi->config->spi->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
            spi->config->spi->CR2 |= SPI_CR2_TXEIE;
            break;
        case 2:
            spi->config->spi->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
            spi->config->spi->CR2 |= SPI_CR2_RXNEIE;
            break;
        case 3:
            spi->config->spi->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
            spi->config->spi->CR2 |= (SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
            break;
        default:
            break;
    }
    
    if (mode < 4) {
        IRQn_Type spiIRQ = NonMaskableInt_IRQn;
        if (spi->config->spi == SPI1) {
            spiIRQ = SPI1_IRQn;
        } else if (spi->config->spi == SPI2) {
            spiIRQ = SPI2_IRQn;
        } else if (spi->config->spi == SPI3) {
            spiIRQ = SPI3_IRQn;
        }
        
        if (mode != 0) {
            NVIC_SetPriority(spiIRQ, 1);
            NVIC_EnableIRQ(spiIRQ);
        } else {
            NVIC_DisableIRQ(spiIRQ);
        }
    }
}

// Pozostałe funkcje pomocnicze
bool spi_is_dma_tx_busy(spi_t* spi) {
    return spi->mTXbusy;
}

bool spi_is_dma_rx_busy(spi_t* spi) {
    return spi->mRXbusy;
}

bool spi_is_txe(const spi_t* spi) {
    return (spi->config->spi->SR & SPI_SR_TXE);
}

bool spi_is_rxne(const spi_t* spi) {
    return (spi->config->spi->SR & SPI_SR_RXNE);
}

void spi_clear_dma_tx_busy(spi_t* spi) {
    while (!(spi->config->spi->SR & SPI_SR_TXE)){};
    if (!spi->mRXbusy) {
        while (spi->config->spi->SR & SPI_SR_BSY){};
    }
    spi_clear_ifcr(spi->mdmastreamtx);
    spi->mTXbusy = false;
    if (!spi->mTXbusy && !spi->mRXbusy) {
        spi->config->CSPort->BSRR = (1 << (spi->config->CSPin + 16));
    }
    spi->config->spi->CR2 &= ~SPI_CR2_TXDMAEN;
}

void spi_clear_dma_rx_busy(spi_t* spi) {
    volatile bool status;
    spi->config->spi->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    
    do {
        status = (spi->config->spi->SR & SPI_SR_RXNE);
    } while (status);
    
    if (!spi->mTXbusy) {
        do {
            status = (spi->config->spi->SR & SPI_SR_BSY);
        } while (status);
    }
    
    spi_clear_ifcr(spi->mdmastreamrx);
    spi->mRXbusy = false;
    if (!spi->mTXbusy && !spi->mRXbusy) {
        spi->config->CSPort->BSRR = (1 << (spi->config->CSPin + 16));
    }
}

void spi_set_cs(const spi_t* spi, bool state) {
    if (state) {
        spi->config->CSPort->BSRR = (1 << (spi->config->CSPin + 16));
    } else {
        spi->config->CSPort->BSRR = (1 << spi->config->CSPin);
    }
}

void spi_enable(const spi_t* spi) {
    spi->config->spi->CR1 |= SPI_CR1_SPE;
}

void spi_disable(const spi_t* spi) {
    spi->config->spi->CR1 &= ~SPI_CR1_SPE;
}

// Funkcje pomocnicze dla DMA
void spi_clear_ifcr(const DMA_Stream_TypeDef* stream) {
    uint8_t streamindex = 0;
    DMA_TypeDef* dma;
    
    if (stream >= DMA1_Stream0 && stream <= DMA1_Stream7) {
        dma = DMA1;
    } else if (stream >= DMA2_Stream0 && stream <= DMA2_Stream7) {
        dma = DMA2;
    } else {
        return;
    }
    
    const uint32_t LIFCR_FLAGS[4] = {
        DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0,
        DMA_LIFCR_CTCIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1,
        DMA_LIFCR_CTCIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2,
        DMA_LIFCR_CTCIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3
    };
    
    const uint32_t HIFCR_FLAGS[4] = {
        DMA_HIFCR_CTCIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4,
        DMA_HIFCR_CTCIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5,
        DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6,
        DMA_HIFCR_CTCIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7
    };
    
    if (stream == DMA1_Stream0 || stream == DMA2_Stream0) streamindex = 0;
    else if (stream == DMA1_Stream1 || stream == DMA2_Stream1) streamindex = 1;
    else if (stream == DMA1_Stream2 || stream == DMA2_Stream2) streamindex = 2;
    else if (stream == DMA1_Stream3 || stream == DMA2_Stream3) streamindex = 3;
    else if (stream == DMA1_Stream4 || stream == DMA2_Stream4) streamindex = 4;
    else if (stream == DMA1_Stream5 || stream == DMA2_Stream5) streamindex = 5;
    else if (stream == DMA1_Stream6 || stream == DMA2_Stream6) streamindex = 6;
    else if (stream == DMA1_Stream7 || stream == DMA2_Stream7) streamindex = 7;
    else return;
    
    if (streamindex < 4) {
        dma->LIFCR |= LIFCR_FLAGS[streamindex];
    } else {
        dma->HIFCR |= HIFCR_FLAGS[streamindex - 4];
    }
}

IRQn_Type spi_get_irqn_stream_type(const DMA_Stream_TypeDef* dmastream) {
    if (dmastream == DMA1_Stream0) return DMA1_Stream0_IRQn;
    else if (dmastream == DMA1_Stream1) return DMA1_Stream1_IRQn;
    else if (dmastream == DMA1_Stream2) return DMA1_Stream2_IRQn;
    else if (dmastream == DMA1_Stream3) return DMA1_Stream3_IRQn;
    else if (dmastream == DMA1_Stream4) return DMA1_Stream4_IRQn;
    else if (dmastream == DMA1_Stream5) return DMA1_Stream5_IRQn;
    else if (dmastream == DMA1_Stream6) return DMA1_Stream6_IRQn;
    else if (dmastream == DMA1_Stream7) return DMA1_Stream7_IRQn;
    else if (dmastream == DMA2_Stream0) return DMA2_Stream0_IRQn;
    else if (dmastream == DMA2_Stream1) return DMA2_Stream1_IRQn;
    else if (dmastream == DMA2_Stream2) return DMA2_Stream2_IRQn;
    else if (dmastream == DMA2_Stream3) return DMA2_Stream3_IRQn;
    else if (dmastream == DMA2_Stream4) return DMA2_Stream4_IRQn;
    else if (dmastream == DMA2_Stream5) return DMA2_Stream5_IRQn;
    else if (dmastream == DMA2_Stream6) return DMA2_Stream6_IRQn;
    else if (dmastream == DMA2_Stream7) return DMA2_Stream7_IRQn;
    else return NonMaskableInt_IRQn;
}
