#include "W5500.h"

void W5500_InitDefault(w5500_t* w5500, const spi_t* spi){
    // Walidacja parametrów (podobnie jak w spi_init_config)
    if (!w5500 || !spi) {
        return;
    }

    // Wyzerowanie struktury
    memset(w5500, 0, sizeof(w5500_t));

    // Przypisanie SPI
    w5500->spi = spi;

    // Domyślna konfiguracja sieciowa
    // MAC Address: 00:08:DC:01:02:03 (przykładowy)
    w5500->network.mac[0] = 0x00;
    w5500->network.mac[1] = 0x08;
    w5500->network.mac[2] = 0xDC;
    w5500->network.mac[3] = 0x01;
    w5500->network.mac[4] = 0x02;
    w5500->network.mac[5] = 0x03;

    // IP Address: 192.168.0.100
    w5500->network.ip[0] = 192;
    w5500->network.ip[1] = 168;
    w5500->network.ip[2] = 0;
    w5500->network.ip[3] = 100;

    // Subnet Mask: 255.255.255.0
    w5500->network.subnet[0] = 255;
    w5500->network.subnet[1] = 255;
    w5500->network.subnet[2] = 255;
    w5500->network.subnet[3] = 0;

    // Gateway: 192.168.0.1
    w5500->network.gateway[0] = 192;
    w5500->network.gateway[1] = 168;
    w5500->network.gateway[2] = 0;
    w5500->network.gateway[3] = 1;

    // PHY Mode: Auto-negotiation enabled, All capable
    w5500->network.phy_mode = 0x07;  // OPMDC[2:0] = 111 (All capable, Auto-negotiation enabled)

    // Inicjalizacja socketów
    for (int i = 0; i < W5500_MAX_SOCKETS; i++) {
        w5500->sockets[i].socket_num = i;
        w5500->sockets[i].protocol = W5500_Sn_MR_PROTOCOL_CLOSED;
        w5500->sockets[i].source_port = 0;
        w5500->sockets[i].status = W5500_Sn_SR_SOCK_CLOSED;
        w5500->sockets[i].flags.is_open = false;
        w5500->sockets[i].flags.is_connected = false;
        w5500->sockets[i].flags.has_data = false;

        // Wyzerowanie adresów docelowych
        memset(w5500->sockets[i].dest_ip, 0, 4);
        w5500->sockets[i].dest_port = 0;
    }

    // Wyzerowanie liczników
    w5500->active_sockets = 0;
    w5500->phy_link_up = false;

    // Oznacz jako niezainicjalizowany (zostanie ustawiony w InitPeripheral)
    w5500->is_initialized = false;
}

void W5500_InitConfig(w5500_t* w5500, const spi_t* spi, const w5500_network_config_t* config){
    // Walidacja parametrów (podobnie jak w spi_init_config)
    if (!w5500 || !spi || !config) {
        return;
    }

    // Wyzerowanie struktury
    memset(w5500, 0, sizeof(w5500_t));

    // Przypisanie SPI
    w5500->spi = spi;

    // Skopiowanie konfiguracji sieciowej
    memcpy(&w5500->network, config, sizeof(w5500_network_config_t));

    // Inicjalizacja socketów
    for (int i = 0; i < W5500_MAX_SOCKETS; i++) {
        w5500->sockets[i].socket_num = i;
        w5500->sockets[i].protocol = W5500_Sn_MR_PROTOCOL_CLOSED;
        w5500->sockets[i].source_port = 0;
        w5500->sockets[i].status = W5500_Sn_SR_SOCK_CLOSED;
        w5500->sockets[i].flags.is_open = false;
        w5500->sockets[i].flags.is_connected = false;
        w5500->sockets[i].flags.has_data = false;

        // Wyzerowanie adresów docelowych
        memset(w5500->sockets[i].dest_ip, 0, 4);
        w5500->sockets[i].dest_port = 0;
    }

    // Wyzerowanie liczników
    w5500->active_sockets = 0;
    w5500->phy_link_up = false;

    // Oznacz jako niezainicjalizowany (zostanie ustawiony w InitPeripheral)
    w5500->is_initialized = false;
}

void W5500_InitPeripheral(w5500_t* w5500){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi) return;

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) return;

    // Inicjalizacja pinu RST jeśli skonfigurowany
    if (w5500->RSTPort) {
        // Włączenie zegara dla portu RST
        if (w5500->RSTPort == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        else if (w5500->RSTPort == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        else if (w5500->RSTPort == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        else if (w5500->RSTPort == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

        // Konfiguracja pinu RST jako wyjście
        w5500->RSTPort->MODER &= ~(0x3U << (w5500->RSTPin * 2));
        w5500->RSTPort->MODER |= (0x1U << (w5500->RSTPin * 2)); // Output
        w5500->RSTPort->OTYPER &= ~(1U << w5500->RSTPin); // Push-pull
        w5500->RSTPort->OSPEEDR |= (0x3U << (w5500->RSTPin * 2)); // High speed
        w5500->RSTPort->PUPDR &= ~(0x3U << (w5500->RSTPin * 2)); // No pull

        // Ustawienie domyślnego stanu HIGH (nieaktywny reset)
        w5500->RSTPort->BSRR = (1U << w5500->RSTPin);
    }

    // Inicjalizacja pinu INT jeśli skonfigurowany
    if (w5500->INTPort && w5500->enableInterrupts) {
        // Włączenie zegara dla portu INT
        if (w5500->INTPort == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        else if (w5500->INTPort == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        else if (w5500->INTPort == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        else if (w5500->INTPort == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

        // Konfiguracja pinu INT jako wejście
        w5500->INTPort->MODER &= ~(0x3U << (w5500->INTPin * 2)); // Input
        w5500->INTPort->PUPDR &= ~(0x3U << (w5500->INTPin * 2));
        w5500->INTPort->PUPDR |= (0x1U << (w5500->INTPin * 2)); // Pull-up

        // Konfiguracja EXTI bezpośrednio tutaj
        // Włączenie zegara SYSCFG
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

        uint8_t pin = w5500->INTPin;
        uint8_t port_source = 0;

        if (w5500->INTPort == GPIOA) port_source = 0;
        else if (w5500->INTPort == GPIOB) port_source = 1;
        else if (w5500->INTPort == GPIOC) port_source = 2;
        else if (w5500->INTPort == GPIOD) port_source = 3;

        // Konfiguracja EXTI dla wybranego pinu
        SYSCFG->EXTICR[pin / 4] &= ~(0xF << ((pin % 4) * 4));
        SYSCFG->EXTICR[pin / 4] |= (port_source << ((pin % 4) * 4));

        // Włączenie przerwania na zbocze opadające (INT active low)
        EXTI->FTSR |= (1U << pin);
        EXTI->RTSR &= ~(1U << pin);

        // Włączenie przerwania
        EXTI->IMR |= (1U << pin);

        // Konfiguracja NVIC
        IRQn_Type exti_irq = EXTI0_IRQn;
        if (pin >= 0 && pin <= 4) {
            exti_irq = (IRQn_Type)(EXTI0_IRQn + pin);
        } else if (pin >= 5 && pin <= 9) {
            exti_irq = EXTI9_5_IRQn;
        } else if (pin >= 10 && pin <= 15) {
            exti_irq = EXTI15_10_IRQn;
        }

        NVIC_SetPriority(exti_irq, w5500->interruptPriority);
        NVIC_EnableIRQ(exti_irq);
    }

    // Wykonanie hardware reset
    W5500_Reset(w5500);

    // 1. Software Reset W5500 (zgodnie z datasheetem)
    uint8_t reset_value = W5500_MR_RST;
    W5500_WriteReg(w5500, W5500_MR, W5500_BSB_COMMON_REG, &reset_value, 1);

    // Oczekiwanie na reset (datasheet: automatycznie się wyczyści)
    uint8_t mode_reg;
    uint32_t timeout = 1000000; // Timeout counter
    do {
        W5500_ReadReg(w5500, W5500_MR, W5500_BSB_COMMON_REG, &mode_reg, 1);
        timeout--;
    } while ((mode_reg & W5500_MR_RST) && timeout > 0);

    if (timeout == 0) {
        // Reset nie zakończył się poprawnie
        return;
    }

    // 2. Sprawdzenie wersji układu
    uint8_t version;
    W5500_ReadReg(w5500, W5500_VERSIONR, W5500_BSB_COMMON_REG, &version, 1);
    if (version != 0x04) {
        // Nieprawidłowa wersja W5500
        return;
    }

    // 3. Konfiguracja adresów sieciowych (4 oddzielne rejestry dla każdego)
    // MAC Address (6 rejestrów)
    for (int i = 0; i < 6; i++) {
        W5500_WriteReg(w5500, W5500_SHAR0 + i, W5500_BSB_COMMON_REG,
                      &w5500->network.mac[i], 1);
    }

    // IP Address (4 rejestry)
    W5500_WriteIPAddress(w5500, W5500_SIPR0, W5500_BSB_COMMON_REG, w5500->network.ip);

    // Subnet Mask (4 rejestry)
    W5500_WriteIPAddress(w5500, W5500_SUBR0, W5500_BSB_COMMON_REG, w5500->network.subnet);

    // Gateway Address (4 rejestry)
    W5500_WriteIPAddress(w5500, W5500_GAR0, W5500_BSB_COMMON_REG, w5500->network.gateway);

    // 4. Konfiguracja PHY zgodnie z datasheetem
    uint8_t phy_config = W5500_PHYCFGR_OPMD | (w5500->network.phy_mode << W5500_PHYCFGR_OPMDC_Pos);
    W5500_WriteReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_config, 1);

    // PHY Reset (RST=0)
    phy_config &= ~W5500_PHYCFGR_RST;
    W5500_WriteReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_config, 1);

    // Opóźnienie dla PHY reset - przy 180MHz
    for(volatile uint32_t i = 0; i < 180000; i++); // ~1ms

    // PHY Reset zakończenie (RST=1)
    phy_config |= W5500_PHYCFGR_RST;
    W5500_WriteReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_config, 1);

    // 5. Konfiguracja domyślnych parametrów retry
    uint8_t retry_time[2] = {0x07, 0xD0}; // 2000 * 100us = 200ms (domyślne)
    W5500_WriteReg(w5500, W5500_RTR0, W5500_BSB_COMMON_REG, retry_time, 2);

    uint8_t retry_count = 0x08; // 8 prób (domyślne)
    W5500_WriteReg(w5500, W5500_RCR, W5500_BSB_COMMON_REG, &retry_count, 1);

    // 6. Konfiguracja domyślnych rozmiarów buforów socketów (2KB każdy)
    for (int i = 0; i < W5500_MAX_SOCKETS; i++) {
        uint8_t bsb = W5500_GetSocketBSB_REG(i);
        uint8_t buffer_size = 0x02; // 2KB

        W5500_WriteReg(w5500, W5500_Sn_RXBUFSIZE, bsb, &buffer_size, 1);
        W5500_WriteReg(w5500, W5500_Sn_TXBUFSIZE, bsb, &buffer_size, 1);
    }

    // 7. Oczekiwanie na ustabilizowanie się PHY
    timeout = 1000000;
    do {
        W5500_ReadReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_config, 1);
        timeout--;
    } while (!(phy_config & W5500_PHYCFGR_LNK) && timeout > 0);

    // Aktualizacja stanu łącza
    w5500->phy_link_up = (phy_config & W5500_PHYCFGR_LNK) ? true : false;

    // 8. Oznaczenie jako zainicjalizowany
    w5500->is_initialized = true;
}

void W5500_Reset(const w5500_t* w5500) {
    // Walidacja parametrów
    if (!w5500 || !w5500->spi) return;

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) return;

    // Hardware reset przez pin RSTn jeśli dostępny
    if (w5500->RSTPort) {
        // RSTn LOW - aktywacja resetu (active low)
        w5500->RSTPort->BSRR = (1U << (w5500->RSTPin + 16));

        // Delay minimum 500µs zgodnie z datasheetem
        // Przy 180MHz: 500µs = 90000 cykli
        for(volatile uint32_t i = 0; i < 90000; i++);

        // RSTn HIGH - zakończenie resetu
        w5500->RSTPort->BSRR = (1U << w5500->RSTPin);

        // Oczekiwanie na PLL Lock (datasheet: max 1ms)
        // Przy 180MHz: 1ms = 180000 cykli
        for(volatile uint32_t i = 0; i < 180000; i++);
    } else {
        // Fallback do software reset
        W5500_SoftReset(w5500);
    }

    // Sprawdzenie czy układ odpowiada
    uint8_t version;
    W5500_ReadReg(w5500, W5500_VERSIONR, W5500_BSB_COMMON_REG, &version, 1);
    // Opcjonalnie sprawdź czy version == 0x04
}

void W5500_SoftReset(const w5500_t* w5500){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    // 1. Ustawienie bitu RST w rejestrze MR (zgodnie z datasheetem)
    uint8_t reset_value = W5500_MR_RST;
    W5500_WriteReg(w5500, W5500_MR, W5500_BSB_COMMON_REG, &reset_value, 1);

    // 2. Oczekiwanie na automatyczne wyczyszczenie bitu RST
    // Datasheet: "It will be automatically cleared as 0 after SW reset"
    uint8_t mode_reg;
    uint32_t timeout = 1000000; // Timeout counter

    do {
        W5500_ReadReg(w5500, W5500_MR, W5500_BSB_COMMON_REG, &mode_reg, 1);
        timeout--;
    } while ((mode_reg & W5500_MR_RST) && timeout > 0);

    // 3. Sprawdzenie czy reset się zakończył
    if (timeout == 0) {
        // Reset nie zakończył się poprawnie - można dodać obsługę błędu
        return;
    }

    // 4. Krótkie opóźnienie dla stabilizacji układu
    for (volatile uint32_t i = 0; i < 10000; i++);

    // 5. Opcjonalne sprawdzenie wersji układu po reset
    uint8_t version;
    W5500_ReadReg(w5500, W5500_VERSIONR, W5500_BSB_COMMON_REG, &version, 1);

    // Sprawdź czy version == 0x04 (zgodnie z datasheetem)
    if (version != 0x04) {
        // Nieprawidłowa wersja po reset - można dodać obsługę błędu
    }
}

void W5500_SocketUpdateStatus(const w5500_t* w5500, socket_t* sock){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // 1. Odczytaj aktualny status socketu
    W5500_ReadReg(w5500, W5500_Sn_SR, bsb, &sock->status, 1);

    // 2. Aktualizuj flagę is_connected na podstawie statusu
    if (sock->protocol == W5500_Sn_MR_PROTOCOL_TCP) {
        sock->flags.is_connected = (sock->status == W5500_Sn_SR_SOCK_ESTABLISHED);
    } else {
        // UDP i MACRAW nie mają połączenia
        sock->flags.is_connected = false;
    }

    // 3. Sprawdź czy socket jest otwarty
    sock->flags.is_open = (sock->status != W5500_Sn_SR_SOCK_CLOSED);

    // 4. Sprawdź czy są dane w buforze RX
    uint8_t rx_size_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_RX_RSR0, bsb, rx_size_bytes, 2);
    uint16_t rx_data_size = (rx_size_bytes[0] << 8) | rx_size_bytes[1];

    sock->flags.has_data = (rx_data_size > 0);
}

void W5500_SetNetworkConfig(w5500_t* w5500, const w5500_network_config_t* config){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !config) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    // 1. Zapisz MAC Address (6 oddzielnych rejestrów)
    for (int i = 0; i < 6; i++) {
        W5500_WriteReg(w5500, W5500_SHAR0 + i, W5500_BSB_COMMON_REG,
                      &config->mac[i], 1);
    }

    // 2. Zapisz IP Address (4 rejestry)
    W5500_WriteIPAddress(w5500, W5500_SIPR0, W5500_BSB_COMMON_REG, config->ip);

    // 3. Zapisz Subnet Mask (4 rejestry)
    W5500_WriteIPAddress(w5500, W5500_SUBR0, W5500_BSB_COMMON_REG, config->subnet);

    // 4. Zapisz Gateway Address (4 rejestry)
    W5500_WriteIPAddress(w5500, W5500_GAR0, W5500_BSB_COMMON_REG, config->gateway);

    // 5. Aktualizuj cache w strukturze
    memcpy(&w5500->network, config, sizeof(w5500_network_config_t));
}

void W5500_GetNetworkConfig(const w5500_t* w5500, w5500_network_config_t* config){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !config) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    // 1. Odczytaj MAC Address (6 oddzielnych rejestrów)
    for (int i = 0; i < 6; i++) {
        W5500_ReadReg(w5500, W5500_SHAR0 + i, W5500_BSB_COMMON_REG,
                     &config->mac[i], 1);
    }

    // 2. Odczytaj IP Address (4 rejestry)
    W5500_ReadIPAddress(w5500, W5500_SIPR0, W5500_BSB_COMMON_REG, config->ip);

    // 3. Odczytaj Subnet Mask (4 rejestry)
    W5500_ReadIPAddress(w5500, W5500_SUBR0, W5500_BSB_COMMON_REG, config->subnet);

    // 4. Odczytaj Gateway Address (4 rejestry)
    W5500_ReadIPAddress(w5500, W5500_GAR0, W5500_BSB_COMMON_REG, config->gateway);

    // 5. Odczytaj PHY mode z rejestru PHYCFGR
    uint8_t phy_reg;
    W5500_ReadReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_reg, 1);
    config->phy_mode = (phy_reg & W5500_PHYCFGR_OPMDC) >> W5500_PHYCFGR_OPMDC_Pos;
}

void W5500_SetPHYConfig(const w5500_t* w5500, uint8_t mode){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || mode > 7) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    // Konfiguracja PHY zgodnie z datasheetem (strona 42)
    // Sequence: OPMD=1, OPMDC=mode, RST=0, RST=1

    // 1. Ustaw OPMD=1 i OPMDC=mode
    uint8_t phy_config = W5500_PHYCFGR_OPMD | (mode << W5500_PHYCFGR_OPMDC_Pos);
    W5500_WriteReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_config, 1);

    // 2. PHY Reset (RST=0)
    phy_config &= ~W5500_PHYCFGR_RST;
    W5500_WriteReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_config, 1);

    // 3. Opóźnienie dla PHY reset - przy 180MHz
    for(volatile uint32_t i = 0; i < 180000; i++); // ~1ms

    // 4. PHY Reset zakończenie (RST=1)
    phy_config |= W5500_PHYCFGR_RST;
    W5500_WriteReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_config, 1);

    // 5. Oczekiwanie na ustabilizowanie się PHY
    uint32_t timeout = 1000000;
    do {
        W5500_ReadReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_config, 1);
        timeout--;
    } while (!(phy_config & W5500_PHYCFGR_LNK) && timeout > 0);
}

uint8_t W5500_GetPHYConfig(const w5500_t* w5500){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi) {
        return 0;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return 0;
    }

    // Odczytaj rejestr PHYCFGR
    uint8_t phy_reg;
    W5500_ReadReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_reg, 1);

    // Wyciągnij OPMDC[5:3]
    return (phy_reg & W5500_PHYCFGR_OPMDC) >> W5500_PHYCFGR_OPMDC_Pos;
}

bool W5500_IsLinkUp(const w5500_t* w5500){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi) {
        return false;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return false;
    }

    // Odczytaj rejestr PHYCFGR
    uint8_t phy_reg;
    W5500_ReadReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_reg, 1);

    // Sprawdź bit LNK[0]
    return (phy_reg & W5500_PHYCFGR_LNK) ? true : false;
}

void W5500_UpdateStatus(w5500_t* w5500){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    // 1. Aktualizuj status PHY i łącza
    uint8_t phy_reg;
    W5500_ReadReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_reg, 1);
    w5500->phy_link_up = (phy_reg & W5500_PHYCFGR_LNK) ? true : false;

    // 2. Aktualizuj status wszystkich socketów
    for (int i = 0; i < W5500_MAX_SOCKETS; i++) {
        W5500_SocketUpdateStatus(w5500, &w5500->sockets[i]);
    }

    // 3. Policz aktywne sockety
    w5500->active_sockets = 0;
    for (int i = 0; i < W5500_MAX_SOCKETS; i++) {
        if (w5500->sockets[i].flags.is_open) {
            w5500->active_sockets++;
        }
    }
}

uint16_t W5500_SocketGetTxFreeSize(const w5500_t* w5500, const socket_t* sock){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock) {
        return 0;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return 0;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // Odczytaj TX Free Size (16-bit register)
    uint8_t tx_free_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_TX_FSR0, bsb, tx_free_bytes, 2);

    // Konwersja z big-endian do little-endian
    return (tx_free_bytes[0] << 8) | tx_free_bytes[1];
}

uint16_t W5500_SocketGetRxDataSize(const w5500_t* w5500, const socket_t* sock){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock) {
        return 0;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return 0;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // Odczytaj RX Received Size (16-bit register)
    uint8_t rx_size_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_RX_RSR0, bsb, rx_size_bytes, 2);

    // Konwersja z big-endian do little-endian
    return (rx_size_bytes[0] << 8) | rx_size_bytes[1];
}

bool W5500_SocketHasData(const w5500_t* w5500, const socket_t* sock){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock) {
        return false;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return false;
    }

    // Sprawdź flagę cache (szybkie sprawdzenie)
    if (sock->flags.has_data) {
        return true;
    }

    // Bezpośrednie sprawdzenie rejestru (dla pewności)
    uint16_t rx_data_size = W5500_SocketGetRxDataSize(w5500, sock);
    return (rx_data_size > 0);
}

bool W5500_SocketCanSend(const w5500_t* w5500, const socket_t* sock, uint16_t size){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock || size == 0) {
        return false;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return false;
    }

    // Sprawdzenie czy socket jest otwarty
    if (!sock->flags.is_open) {
        return false;
    }

    // Dla TCP sprawdź czy jest połączony
    if (sock->protocol == W5500_Sn_MR_PROTOCOL_TCP && !sock->flags.is_connected) {
        return false;
    }

    // Sprawdź wolne miejsce w TX buffer
    uint16_t tx_free_size = W5500_SocketGetTxFreeSize(w5500, sock);

    return (size <= tx_free_size);
}

int8_t W5500_SocketInit(socket_t* sock, uint8_t socket_num, uint8_t protocol, uint16_t port){
    // Walidacja parametrów (podobnie jak w spi_init_config)
    if (!sock || socket_num >= W5500_MAX_SOCKETS) {
        return W5500_INVALID_SOCKET;
    }

    // Walidacja protokołu
    if (protocol != W5500_Sn_MR_PROTOCOL_CLOSED &&
        protocol != W5500_Sn_MR_PROTOCOL_TCP &&
        protocol != W5500_Sn_MR_PROTOCOL_UDP &&
        protocol != W5500_Sn_MR_PROTOCOL_MACRAW) {
        return W5500_ERROR;
        }

    // MACRAW tylko dla socketu 0 (zgodnie z datasheetem)
    if (protocol == W5500_Sn_MR_PROTOCOL_MACRAW && socket_num != 0) {
        return W5500_ERROR;
    }

    // Wyzerowanie struktury socketu (podobnie jak w spi_init_config)
    memset(sock, 0, sizeof(socket_t));

    // Ustawienie podstawowych parametrów
    sock->socket_num = socket_num;
    sock->protocol = protocol;
    sock->source_port = port;
    sock->status = W5500_Sn_SR_SOCK_CLOSED;

    // Inicjalizacja flag w stanie domyślnym
    sock->flags.is_open = false;
    sock->flags.is_connected = false;
    sock->flags.has_data = false;

    // Wyzerowanie adresów docelowych
    memset(sock->dest_ip, 0, 4);
    sock->dest_port = 0;

    return W5500_OK;
}

int8_t W5500_SocketOpen(const w5500_t* w5500, socket_t* sock){
    // Walidacja parametrów (podobnie jak w spi_init_peripheral)
    if (!w5500 || !w5500->spi || !sock) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return W5500_ERROR;
    }

    // Sprawdzenie numeru socketu
    if (sock->socket_num >= W5500_MAX_SOCKETS) {
        return W5500_INVALID_SOCKET;
    }

    // Sprawdzenie czy socket nie jest już otwarty
    if (sock->flags.is_open) {
        return W5500_OK; // Już otwarty
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // 1. Ustaw protokół socketu w rejestrze SnMR
    W5500_WriteReg(w5500, W5500_Sn_MR, bsb, &sock->protocol, 1);

    // 2. Ustaw port źródłowy (jeśli podany)
    if (sock->source_port != 0) {
        uint8_t port_bytes[2] = {(uint8_t)(sock->source_port >> 8), (uint8_t)(sock->source_port & 0xFF)};
        W5500_WriteReg(w5500, W5500_Sn_PORT0, bsb, port_bytes, 2);
    }

    // 3. Wyślij komendę OPEN
    uint8_t command = W5500_Sn_CR_OPEN;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    // 4. Czekaj na zakończenie komendy (podobnie jak w InitPeripheral)
    uint32_t timeout = 100000;
    do {
        W5500_ReadReg(w5500, W5500_Sn_CR, bsb, &command, 1);
        timeout--;
    } while (command != 0x00 && timeout > 0);

    if (timeout == 0) {
        return W5500_TIMEOUT;
    }

    // 5. Sprawdź status socketu i aktualizuj flagę
    W5500_ReadReg(w5500, W5500_Sn_SR, bsb, &sock->status, 1);

    // Sprawdź czy socket został poprawnie otwarty (zgodnie z datasheetem)
    if (sock->status == W5500_Sn_SR_SOCK_INIT ||      // TCP
        sock->status == W5500_Sn_SR_SOCK_UDP ||       // UDP
        sock->status == W5500_Sn_SR_SOCK_MACRAW) {    // MACRAW

        sock->flags.is_open = true;
        return W5500_OK;
        }

    return W5500_ERROR;
}

int8_t W5500_SocketClose(const w5500_t* w5500, socket_t* sock){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return W5500_ERROR;
    }

    // Sprawdzenie numeru socketu
    if (sock->socket_num >= W5500_MAX_SOCKETS) {
        return W5500_INVALID_SOCKET;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // 1. Wyślij komendę CLOSE
    uint8_t command = W5500_Sn_CR_CLOSE;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    // 2. Czekaj na zakończenie komendy
    uint32_t timeout = 100000;
    do {
        W5500_ReadReg(w5500, W5500_Sn_CR, bsb, &command, 1);
        timeout--;
    } while (command != 0x00 && timeout > 0);

    if (timeout == 0) {
        return W5500_TIMEOUT;
    }

    // 3. Sprawdź status socketu
    W5500_ReadReg(w5500, W5500_Sn_SR, bsb, &sock->status, 1);

    // 4. Reset flag socketu (niezależnie od statusu - socket jest zamykany)
    sock->flags.is_open = false;
    sock->flags.is_connected = false;
    sock->flags.has_data = false;

    // 5. Sprawdź czy socket został poprawnie zamknięty
    if (sock->status == W5500_Sn_SR_SOCK_CLOSED) {
        return W5500_OK;
    }

    // Jeśli status nie jest CLOSED, ale komendy się wykonała, to też OK
    // (czasami może być opóźnienie w aktualizacji statusu)
    return W5500_OK;
}

int8_t W5500_SocketListen(const w5500_t* w5500, socket_t* sock){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy socket jest otwarty i to TCP
    if (!sock->flags.is_open || sock->protocol != W5500_Sn_MR_PROTOCOL_TCP) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy socket jest w stanie INIT
    if (sock->status != W5500_Sn_SR_SOCK_INIT) {
        return W5500_ERROR;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // 1. Wyślij komendę LISTEN
    uint8_t command = W5500_Sn_CR_LISTEN;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    // 2. Czekaj na zakończenie komendy
    uint32_t timeout = 100000;
    do {
        W5500_ReadReg(w5500, W5500_Sn_CR, bsb, &command, 1);
        timeout--;
    } while (command != 0x00 && timeout > 0);

    if (timeout == 0) {
        return W5500_TIMEOUT;
    }

    // 3. Sprawdź status socketu
    W5500_ReadReg(w5500, W5500_Sn_SR, bsb, &sock->status, 1);

    // 4. Sprawdź czy socket przeszedł w stan LISTEN
    if (sock->status == W5500_Sn_SR_SOCK_LISTEN) {
        return W5500_OK;
    }

    return W5500_ERROR;
}

int8_t W5500_SocketConnect(const w5500_t* w5500, socket_t* sock, const uint8_t* dest_ip, uint16_t dest_port){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock || !dest_ip) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy socket jest otwarty i to TCP
    if (!sock->flags.is_open || sock->protocol != W5500_Sn_MR_PROTOCOL_TCP) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy socket jest w stanie INIT
    if (sock->status != W5500_Sn_SR_SOCK_INIT) {
        return W5500_ERROR;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // 1. Ustaw adres docelowy IP
    W5500_WriteIPAddress(w5500, W5500_Sn_DIPR0, bsb, dest_ip);

    // 2. Ustaw port docelowy
    uint8_t port_bytes[2] = {(uint8_t)(dest_port >> 8), (uint8_t)(dest_port & 0xFF)};
    W5500_WriteReg(w5500, W5500_Sn_DPORT0, bsb, port_bytes, 2);

    // 3. Wyślij komendę CONNECT
    uint8_t command = W5500_Sn_CR_CONNECT;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    // 4. Czekaj na zakończenie komendy
    uint32_t timeout = 100000;
    do {
        W5500_ReadReg(w5500, W5500_Sn_CR, bsb, &command, 1);
        timeout--;
    } while (command != 0x00 && timeout > 0);

    if (timeout == 0) {
        return W5500_TIMEOUT;
    }

    // 5. Sprawdź status socketu
    W5500_ReadReg(w5500, W5500_Sn_SR, bsb, &sock->status, 1);

    // 6. Sprawdź czy połączenie zostało nawiązane
    if (sock->status == W5500_Sn_SR_SOCK_ESTABLISHED) {
        // Zaktualizuj cache adresu docelowego w socket_t
        memcpy(sock->dest_ip, dest_ip, 4);
        sock->dest_port = dest_port;
        sock->flags.is_connected = true;
        return W5500_OK;
    }

    // Sprawdź czy socket jest w stanie przejściowym
    if (sock->status == W5500_Sn_SR_SOCK_SYNSENT) {
        return W5500_BUSY; // Połączenie w toku
    }

    return W5500_ERROR;
}

int8_t W5500_SocketDisconnect(const w5500_t* w5500, socket_t* sock){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy socket jest otwarty, to TCP i połączony
    if (!sock->flags.is_open || sock->protocol != W5500_Sn_MR_PROTOCOL_TCP || !sock->flags.is_connected) {
        return W5500_ERROR;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // 1. Wyślij komendę DISCON (graceful disconnect)
    uint8_t command = W5500_Sn_CR_DISCON;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    // 2. Czekaj na zakończenie komendy
    uint32_t timeout = 100000;
    do {
        W5500_ReadReg(w5500, W5500_Sn_CR, bsb, &command, 1);
        timeout--;
    } while (command != 0x00 && timeout > 0);

    if (timeout == 0) {
        return W5500_TIMEOUT;
    }

    // 3. Sprawdź status socketu
    W5500_ReadReg(w5500, W5500_Sn_SR, bsb, &sock->status, 1);

    // 4. Aktualizuj flagi socketu
    sock->flags.is_connected = false;
    sock->flags.has_data = false;

    // Wyzeruj cache adresu docelowego
    memset(sock->dest_ip, 0, 4);
    sock->dest_port = 0;

    // 5. Sprawdź czy socket został rozłączony
    if (sock->status == W5500_Sn_SR_SOCK_CLOSED) {
        sock->flags.is_open = false;
        return W5500_OK;
    }

    // Socket może być w stanie przejściowym (FIN_WAIT, CLOSING, etc.)
    return W5500_OK;
}

void W5500_WriteBuffer(const w5500_t* w5500, uint16_t addr, uint8_t bsb, const uint8_t* data, uint16_t len){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !data || len == 0) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    // Budowanie bajtu kontrolnego
    uint8_t control_byte = (bsb << W5500_SPI_BSB_Pos) |
                          (1U << W5500_SPI_RWB_Pos) |
                          W5500_SPI_OM_VDM;

    // Aktywacja CS (LOW)
    spi_set_cs(w5500->spi, false);

    // Wysłanie adresu i kontroli (bez DMA - krótkie)
    spi_send(w5500->spi, (uint8_t)(addr >> 8));
    spi_send(w5500->spi, (uint8_t)(addr & 0xFF));
    spi_send(w5500->spi, control_byte);

    // **KLUCZOWE: Wysłanie danych przez DMA**
    if (len > 16) {  // DMA dla większych bloków
        spi_dma_send((spi_t*)w5500->spi, data, len);

        // Czekaj na zakończenie DMA
        while (spi_is_dma_tx_busy((spi_t*)w5500->spi)) {
            // Można dodać timeout
        }
    } else {
        // Dla małych bloków - zwykłe SPI
        for (uint16_t i = 0; i < len; i++) {
            spi_send(w5500->spi, data[i]);
        }
    }

    // Deaktywacja CS (HIGH) - automatyczna w spi_dma_send()
    // spi_set_cs(w5500->spi, true); // Już zrobione przez DMA
}

void W5500_ReadBuffer(const w5500_t* w5500, uint16_t addr, uint8_t bsb, uint8_t* data, uint16_t len){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !data || len == 0) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    // Budowanie bajtu kontrolnego
    uint8_t control_byte = (bsb << W5500_SPI_BSB_Pos) |
                          (0U << W5500_SPI_RWB_Pos) |
                          W5500_SPI_OM_VDM;

    // Aktywacja CS (LOW)
    spi_set_cs(w5500->spi, false);

    // Wysłanie adresu i kontroli
    spi_send(w5500->spi, (uint8_t)(addr >> 8));
    spi_send(w5500->spi, (uint8_t)(addr & 0xFF));
    spi_send(w5500->spi, control_byte);

    // **KLUCZOWE: Odczyt danych przez DMA**
    if (len > 16) {  // DMA dla większych bloków
        spi_dma_read((spi_t*)w5500->spi, data, len);

        // Czekaj na zakończenie DMA
        while (spi_is_dma_rx_busy((spi_t*)w5500->spi)) {
            // Można dodać timeout
        }
    } else {
        // Dla małych bloków - zwykłe SPI
        for (uint16_t i = 0; i < len; i++) {
            data[i] = spi_read(w5500->spi);
        }
    }

    // CS już zdeaktywowany przez DMA
}

int16_t W5500_SocketSend(const w5500_t* w5500, socket_t* sock, const uint8_t* data, uint16_t len){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock || !data || len == 0) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy socket jest otwarty
    if (!sock->flags.is_open) {
        return W5500_ERROR;
    }

    // TCP - sprawdź połączenie
    if (sock->protocol == W5500_Sn_MR_PROTOCOL_TCP && !sock->flags.is_connected) {
        return W5500_ERROR;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // 1. Sprawdź wolne miejsce w TX buffer
    uint8_t tx_free_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_TX_FSR0, bsb, tx_free_bytes, 2);
    uint16_t tx_free_size = (tx_free_bytes[0] << 8) | tx_free_bytes[1];

    if (len > tx_free_size) {
        return W5500_BUFFER_FULL;
    }

    // 2. Pobierz TX Write Pointer
    uint8_t tx_wr_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_TX_WR0, bsb, tx_wr_bytes, 2);
    uint16_t tx_wr_ptr = (tx_wr_bytes[0] << 8) | tx_wr_bytes[1];

    // 3. Zapisz dane do TX buffer
    uint8_t tx_bsb = W5500_GetSocketBSB_TX(sock->socket_num);
    W5500_WriteBuffer(w5500, tx_wr_ptr, tx_bsb, data, len);

    // 4. Aktualizuj TX Write Pointer
    tx_wr_ptr += len;
    tx_wr_bytes[0] = (uint8_t)(tx_wr_ptr >> 8);
    tx_wr_bytes[1] = (uint8_t)(tx_wr_ptr & 0xFF);
    W5500_WriteReg(w5500, W5500_Sn_TX_WR0, bsb, tx_wr_bytes, 2);

    // 5. Wyślij komendą SEND
    uint8_t command = W5500_Sn_CR_SEND;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    // 6. Czekaj na zakończenie komendy (opcjonalnie)
    uint32_t timeout = 100000;
    do {
        W5500_ReadReg(w5500, W5500_Sn_CR, bsb, &command, 1);
        timeout--;
    } while (command != 0x00 && timeout > 0);

    if (timeout == 0) {
        return W5500_TIMEOUT;
    }

    return len;
}

int16_t W5500_SocketSendTo(const w5500_t* w5500, socket_t* sock, const uint8_t* data, uint16_t len,
                          const uint8_t* dest_ip, uint16_t dest_port){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock || !data || !dest_ip || len == 0) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy socket jest otwarty i to UDP
    if (!sock->flags.is_open || sock->protocol != W5500_Sn_MR_PROTOCOL_UDP) {
        return W5500_ERROR;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // 1. Ustaw adres docelowy IP
    W5500_WriteIPAddress(w5500, W5500_Sn_DIPR0, bsb, dest_ip);

    // 2. Ustaw port docelowy
    uint8_t port_bytes[2] = {(uint8_t)(dest_port >> 8), (uint8_t)(dest_port & 0xFF)};
    W5500_WriteReg(w5500, W5500_Sn_DPORT0, bsb, port_bytes, 2);

    // 3. Sprawdź wolne miejsce w TX buffer
    uint8_t tx_free_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_TX_FSR0, bsb, tx_free_bytes, 2);
    uint16_t tx_free_size = (tx_free_bytes[0] << 8) | tx_free_bytes[1];

    if (len > tx_free_size) {
        return W5500_BUFFER_FULL;
    }

    // 4. Pobierz TX Write Pointer
    uint8_t tx_wr_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_TX_WR0, bsb, tx_wr_bytes, 2);
    uint16_t tx_wr_ptr = (tx_wr_bytes[0] << 8) | tx_wr_bytes[1];

    // 5. Zapisz dane do TX buffer
    uint8_t tx_bsb = W5500_GetSocketBSB_TX(sock->socket_num);
    W5500_WriteBuffer(w5500, tx_wr_ptr, tx_bsb, data, len);

    // 6. Aktualizuj TX Write Pointer
    tx_wr_ptr += len;
    tx_wr_bytes[0] = (uint8_t)(tx_wr_ptr >> 8);
    tx_wr_bytes[1] = (uint8_t)(tx_wr_ptr & 0xFF);
    W5500_WriteReg(w5500, W5500_Sn_TX_WR0, bsb, tx_wr_bytes, 2);

    // 7. Wyślij komendą SEND
    uint8_t command = W5500_Sn_CR_SEND;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    // 8. Czekaj na zakończenie komendy
    uint32_t timeout = 100000;
    do {
        W5500_ReadReg(w5500, W5500_Sn_CR, bsb, &command, 1);
        timeout--;
    } while (command != 0x00 && timeout > 0);

    if (timeout == 0) {
        return W5500_TIMEOUT;
    }

    // 9. Zaktualizuj cache adresu docelowego w socket_t
    memcpy(sock->dest_ip, dest_ip, 4);
    sock->dest_port = dest_port;

    return len;
}

int16_t W5500_SocketSendRaw(const w5500_t* w5500, socket_t* sock, const uint8_t* data, uint16_t len){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !sock || !data || len == 0) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return W5500_ERROR;
    }

    // **TYLKO dla MACRAW - Socket 0**
    if (sock->protocol != W5500_Sn_MR_PROTOCOL_MACRAW) {
        return W5500_ERROR;
    }

    // Sprawdzenie czy socket jest otwarty
    if (!sock->flags.is_open) {
        return W5500_ERROR;
    }

    // **MACRAW: Walidacja rozmiaru ramki Ethernet (64-1514 bajtów)**
    if (len < 64 || len > 1514) {
        return W5500_ERROR;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(0); // Zawsze Socket 0

    // 1. Sprawdź wolne miejsce w TX buffer
    uint8_t tx_free_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_TX_FSR0, bsb, tx_free_bytes, 2);
    uint16_t tx_free_size = (tx_free_bytes[0] << 8) | tx_free_bytes[1];

    if (len > tx_free_size) {
        return W5500_BUFFER_FULL;
    }

    // 2. Pobierz TX Write Pointer
    uint8_t tx_wr_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_TX_WR0, bsb, tx_wr_bytes, 2);
    uint16_t tx_wr_ptr = (tx_wr_bytes[0] << 8) | tx_wr_bytes[1];

    // 3. Zapisz surową ramkę Ethernet do TX buffer
    uint8_t tx_bsb = W5500_GetSocketBSB_TX(0); // Zawsze Socket 0
    W5500_WriteBuffer(w5500, tx_wr_ptr, tx_bsb, data, len);

    // 4. Aktualizuj TX Write Pointer
    tx_wr_ptr += len;
    tx_wr_bytes[0] = (uint8_t)(tx_wr_ptr >> 8);
    tx_wr_bytes[1] = (uint8_t)(tx_wr_ptr & 0xFF);
    W5500_WriteReg(w5500, W5500_Sn_TX_WR0, bsb, tx_wr_bytes, 2);

    // 5. Wyślij komendą SEND
    uint8_t command = W5500_Sn_CR_SEND;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    // 6. Czekaj na zakończenie komendy
    uint32_t timeout = 100000;
    do {
        W5500_ReadReg(w5500, W5500_Sn_CR, bsb, &command, 1);
        timeout--;
    } while (command != 0x00 && timeout > 0);

    if (timeout == 0) {
        return W5500_TIMEOUT;
    }

    return len;
}

int16_t W5500_SocketReceiveTCP(const w5500_t* w5500, socket_t* sock,
                              uint8_t* buffer, uint16_t max_len){
    // Walidacja parametrów
    if (!w5500 || !sock || !buffer || max_len == 0) {
        return W5500_ERROR;
    }

    // Sprawdź czy to socket TCP
    if (sock->protocol != W5500_Sn_MR_PROTOCOL_TCP) {
        return W5500_ERROR;
    }

    // Sprawdź czy socket jest połączony
    if (!sock->flags.is_connected) {
        return W5500_ERROR;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // Sprawdź ile danych jest dostępnych
    uint8_t rx_size_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_RX_RSR0, bsb, rx_size_bytes, 2);
    uint16_t available_data = (rx_size_bytes[0] << 8) | rx_size_bytes[1];

    if (available_data == 0) {
        return W5500_NO_DATA;
    }

    // Ogranicz do rozmiaru bufora
    uint16_t to_read = (available_data > max_len) ? max_len : available_data;

    // Pobierz RX Read Pointer
    uint8_t rx_rd_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_RX_RD0, bsb, rx_rd_bytes, 2);
    uint16_t rx_rd_ptr = (rx_rd_bytes[0] << 8) | rx_rd_bytes[1];

    // Odczytaj dane przez DMA
    uint8_t rx_bsb = W5500_GetSocketBSB_RX(sock->socket_num);
    W5500_ReadBuffer(w5500, rx_rd_ptr, rx_bsb, buffer, to_read);

    // Aktualizuj RX Read Pointer
    rx_rd_ptr += to_read;
    rx_rd_bytes[0] = (uint8_t)(rx_rd_ptr >> 8);
    rx_rd_bytes[1] = (uint8_t)(rx_rd_ptr & 0xFF);
    W5500_WriteReg(w5500, W5500_Sn_RX_RD0, bsb, rx_rd_bytes, 2);

    // Potwierdź odczyt
    uint8_t command = W5500_Sn_CR_RECV;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    return to_read;
}

int16_t W5500_SocketReceiveUDP(const w5500_t* w5500, socket_t* sock, uint8_t* buffer,
                                uint16_t max_len, w5500_packet_info_t* packet_info){
    // Walidacja parametrów
    if (!w5500 || !sock || !buffer || max_len == 0) {
        return W5500_ERROR;
    }

    // Sprawdź czy to socket UDP
    if (sock->protocol != W5500_Sn_MR_PROTOCOL_UDP) {
        return W5500_ERROR;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(sock->socket_num);

    // Sprawdź ile danych jest dostępnych
    uint8_t rx_size_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_RX_RSR0, bsb, rx_size_bytes, 2);
    uint16_t available_data = (rx_size_bytes[0] << 8) | rx_size_bytes[1];

    if (available_data < 8) {
        return W5500_NO_DATA; // Minimum nagłówek UDP
    }

    // Pobierz RX Read Pointer
    uint8_t rx_rd_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_RX_RD0, bsb, rx_rd_bytes, 2);
    uint16_t rx_rd_ptr = (rx_rd_bytes[0] << 8) | rx_rd_bytes[1];

    // **ETAP 1: Odczytaj nagłówek UDP (8 bajtów)**
    uint8_t udp_header[8];
    uint8_t rx_bsb = W5500_GetSocketBSB_RX(sock->socket_num);
    W5500_ReadBuffer(w5500, rx_rd_ptr, rx_bsb, udp_header, 8);

    // **ETAP 2: Wyciągnij informacje z nagłówka**
    uint16_t packet_length = (udp_header[6] << 8) | udp_header[7];

    // Sprawdź czy cały pakiet jest dostępny
    if (available_data < (8 + packet_length)) {
        // Pakiet niekompletny - ustaw flagę
        if (packet_info) {
            packet_info->is_complete = false;
            packet_info->packet_length = packet_length;
        }
        return W5500_INCOMPLETE_PACKET;
    }

    // Sprawdź czy bufor wystarczy
    if (packet_length > max_len) {
        packet_length = max_len;
    }

    // **ETAP 3: Odczytaj dane pakietu**
    W5500_ReadBuffer(w5500, rx_rd_ptr + 8, rx_bsb, buffer, packet_length);

    // **ETAP 4: Wypełnij informacje o pakiecie**
    if (packet_info) {
        // IP nadawcy
        packet_info->src_ip[0] = udp_header[0];
        packet_info->src_ip[1] = udp_header[1];
        packet_info->src_ip[2] = udp_header[2];
        packet_info->src_ip[3] = udp_header[3];

        // Port nadawcy
        packet_info->src_port = (udp_header[4] << 8) | udp_header[5];

        // Długość i status
        packet_info->packet_length = packet_length;
        packet_info->is_complete = true;

        // Wyzeruj pola MACRAW
        memset(packet_info->src_mac, 0, 6);
        packet_info->ethertype = 0;
    }

    // **ETAP 5: Aktualizuj wskaźnik**
    rx_rd_ptr += 8 + packet_length;
    rx_rd_bytes[0] = (uint8_t)(rx_rd_ptr >> 8);
    rx_rd_bytes[1] = (uint8_t)(rx_rd_ptr & 0xFF);
    W5500_WriteReg(w5500, W5500_Sn_RX_RD0, bsb, rx_rd_bytes, 2);

    // Potwierdź odczyt
    uint8_t command = W5500_Sn_CR_RECV;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    return packet_length;
}

int16_t W5500_SocketReceiveMACRAW(const w5500_t* w5500, socket_t* sock, uint8_t* buffer, uint16_t max_len,
                                    w5500_packet_info_t* packet_info){
    // Walidacja parametrów
    if (!w5500 || !sock || !buffer || max_len == 0) {
        return W5500_ERROR;
    }

    // Sprawdź czy to socket MACRAW (tylko Socket 0)
    if (sock->protocol != W5500_Sn_MR_PROTOCOL_MACRAW || sock->socket_num != 0) {
        return W5500_ERROR;
    }

    uint8_t bsb = W5500_GetSocketBSB_REG(0);

    // Sprawdź ile danych jest dostępnych
    uint8_t rx_size_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_RX_RSR0, bsb, rx_size_bytes, 2);
    uint16_t available_data = (rx_size_bytes[0] << 8) | rx_size_bytes[1];

    if (available_data < 2) {
        return W5500_NO_DATA; // Minimum nagłówek długości
    }

    // Pobierz RX Read Pointer
    uint8_t rx_rd_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_RX_RD0, bsb, rx_rd_bytes, 2);
    uint16_t rx_rd_ptr = (rx_rd_bytes[0] << 8) | rx_rd_bytes[1];

    // **ETAP 1: Odczytaj nagłówek długości (2 bajty)**
    uint8_t length_header[2];
    uint8_t rx_bsb = W5500_GetSocketBSB_RX(0);
    W5500_ReadBuffer(w5500, rx_rd_ptr, rx_bsb, length_header, 2);

    // **ETAP 2: Wyciągnij długość ramki**
    uint16_t frame_length = (length_header[0] << 8) | length_header[1];

    // Sprawdź czy cała ramka jest dostępna
    if (available_data < (2 + frame_length)) {
        // Ramka niekompletna
        if (packet_info) {
            packet_info->is_complete = false;
            packet_info->packet_length = frame_length;
        }
        return W5500_INCOMPLETE_PACKET;
    }

    // Sprawdź rozmiar ramki (64-1514 bajtów dla Ethernet)
    if (frame_length < 64 || frame_length > 1514) {
        return W5500_ERROR;
    }

    // Sprawdź czy bufor wystarczy
    if (frame_length > max_len) {
        frame_length = max_len;
    }

    // **ETAP 3: Odczytaj ramkę Ethernet**
    W5500_ReadBuffer(w5500, rx_rd_ptr + 2, rx_bsb, buffer, frame_length);

    // **ETAP 4: Wypełnij informacje o ramce**
    if (packet_info && frame_length >= 14) {
        // MAC nadawcy (bajty 6-11 ramki Ethernet)
        memcpy(packet_info->src_mac, &buffer[6], 6);

        // EtherType (bajty 12-13)
        packet_info->ethertype = (buffer[12] << 8) | buffer[13];

        // Długość i status
        packet_info->packet_length = frame_length;
        packet_info->is_complete = true;

        // Wyzeruj pola UDP
        memset(packet_info->src_ip, 0, 4);
        packet_info->src_port = 0;
    }

    // **ETAP 5: Aktualizuj wskaźnik**
    rx_rd_ptr += 2 + frame_length;
    rx_rd_bytes[0] = (uint8_t)(rx_rd_ptr >> 8);
    rx_rd_bytes[1] = (uint8_t)(rx_rd_ptr & 0xFF);
    W5500_WriteReg(w5500, W5500_Sn_RX_RD0, bsb, rx_rd_bytes, 2);

    // Potwierdź odczyt
    uint8_t command = W5500_Sn_CR_RECV;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    return frame_length;
}

void W5500_EnableInterrupts(const w5500_t* w5500, const uint8_t socket_mask, const uint8_t common_mask) {
    if (!w5500 || !w5500->spi) return;
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) return;

    // Włączenie przerwań socketów
    W5500_WriteReg(w5500, W5500_SIMR, W5500_BSB_COMMON_REG, &socket_mask, 1);

    // Włączenie przerwań wspólnych
    W5500_WriteReg(w5500, W5500_IMR, W5500_BSB_COMMON_REG, &common_mask, 1);
}

void W5500_DisableInterrupts(const w5500_t* w5500) {
    if (!w5500 || !w5500->spi) return;
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) return;

    uint8_t zero = 0x00;
    W5500_WriteReg(w5500, W5500_SIMR, W5500_BSB_COMMON_REG, &zero, 1);
    W5500_WriteReg(w5500, W5500_IMR, W5500_BSB_COMMON_REG, &zero, 1);
}

uint8_t W5500_GetInterruptStatus(const w5500_t* w5500) {
    if (!w5500 || !w5500->spi) return 0;
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) return 0;

    uint8_t sir;
    W5500_ReadReg(w5500, W5500_SIR, W5500_BSB_COMMON_REG, &sir, 1);
    return sir;
}

void W5500_ClearInterrupt(const w5500_t* w5500, const uint8_t socket_num, const uint8_t interrupt_flags) {
    if (!w5500 || !w5500->spi || socket_num >= W5500_MAX_SOCKETS) return;
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) return;

    uint8_t bsb = W5500_GetSocketBSB_REG(socket_num);
    W5500_WriteReg(w5500, W5500_Sn_IR, bsb, &interrupt_flags, 1);
}

uint8_t W5500_GetVersion(const w5500_t* w5500){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi) {
        return 0x00;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return 0x00;
    }

    // Odczytaj wersję układu z rejestru VERSIONR
    uint8_t version;
    W5500_ReadReg(w5500, W5500_VERSIONR, W5500_BSB_COMMON_REG, &version, 1);

    return version;
}

bool W5500_SelfTest(const w5500_t* w5500){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi) {
        return false;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return false;
    }

    // 1. Test wersji układu (powinna być 0x04)
    uint8_t version = W5500_GetVersion(w5500);
    if (version != 0x04) {
        return false;
    }

    // 2. Test zapisu/odczytu rejestru (używamy RTR jako test register)
    uint8_t original_rtr[2];
    W5500_ReadReg(w5500, W5500_RTR0, W5500_BSB_COMMON_REG, original_rtr, 2);

    // Zapisz wartość testową
    uint8_t test_value[2] = {0xAA, 0x55};
    W5500_WriteReg(w5500, W5500_RTR0, W5500_BSB_COMMON_REG, test_value, 2);

    // Odczytaj i sprawdź
    uint8_t read_value[2];
    W5500_ReadReg(w5500, W5500_RTR0, W5500_BSB_COMMON_REG, read_value, 2);

    bool write_test_ok = (read_value[0] == test_value[0] && read_value[1] == test_value[1]);

    // Przywróć oryginalną wartość
    W5500_WriteReg(w5500, W5500_RTR0, W5500_BSB_COMMON_REG, original_rtr, 2);

    if (!write_test_ok) {
        return false;
    }

    // 3. Test rejestru PHY (sprawdź czy można odczytać)
    uint8_t phy_reg;
    W5500_ReadReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_reg, 1);

    // PHY register powinien mieć sensowne wartości (bity 7-0 nie mogą być wszystkie 0 lub 1)
    if (phy_reg == 0x00 || phy_reg == 0xFF) {
        return false;
    }

    return true;
}

void W5500_WriteReg(const w5500_t* w5500, uint16_t addr, uint8_t bsb, const void* data, uint8_t size){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !data || (size != 1 && size != 2)) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    const uint8_t* byte_data = (const uint8_t*)data;

    // Budowanie bajtu kontrolnego: BSB[7:3] + RWB[2] + OM[1:0]
    // RWB = 1 (Write), OM = 00 (VDM)
    uint8_t control_byte = (bsb << W5500_SPI_BSB_Pos) |
                          (1U << W5500_SPI_RWB_Pos) |
                          W5500_SPI_OM_VDM;

    // Aktywacja CS (LOW)
    spi_set_cs(w5500->spi, true);

    // Wysłanie adresu (16 bitów, MSB first)
    spi_send(w5500->spi, (uint8_t)(addr >> 8));    // Starszy bajt adresu
    spi_send(w5500->spi, (uint8_t)(addr & 0xFF));  // Młodszy bajt adresu

    // Wysłanie bajtu kontrolnego
    spi_send(w5500->spi, control_byte);

    // Wysłanie danych (big-endian dla 16-bit)
    if (size == 2) {
        // Dla 16-bit: W5500 oczekuje big-endian (MSB first)
        // Na little-endian systemie (STM32) musimy odwrócić kolejność
        spi_send(w5500->spi, byte_data[1]);  // MSB (starszy bajt)
        spi_send(w5500->spi, byte_data[0]);  // LSB (młodszy bajt)
    } else {
        // Dla 8-bit: wysyłamy bezpośrednio
        spi_send(w5500->spi, byte_data[0]);
    }

    // Deaktywacja CS (HIGH)
    spi_set_cs(w5500->spi, false);
}

void W5500_ReadReg(const w5500_t* w5500, uint16_t addr, uint8_t bsb, void* data, uint8_t size){
    // Walidacja parametrów (zgodnie z Twoim stylem SPI)
    if (!w5500 || !w5500->spi || !data || (size != 1 && size != 2)) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane (jak w spi_init_peripheral)
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    uint8_t* byte_data = (uint8_t*)data;

    // Budowanie bajtu kontrolnego: BSB[7:3] + RWB[2] + OM[1:0]
    // RWB = 0 (Read), OM = 00 (VDM)
    uint8_t control_byte = (bsb << W5500_SPI_BSB_Pos) |
                          (0U << W5500_SPI_RWB_Pos) |
                          W5500_SPI_OM_VDM;

    // Aktywacja CS (LOW)
    spi_set_cs(w5500->spi, true);

    // Wysłanie adresu (16 bitów, MSB first)
    spi_send(w5500->spi, (uint8_t)(addr >> 8));    // Starszy bajt adresu
    spi_send(w5500->spi, (uint8_t)(addr & 0xFF));  // Młodszy bajt adresu

    // Wysłanie bajtu kontrolnego
    spi_send(w5500->spi, control_byte);

    // Odczyt danych (big-endian dla 16-bit)
    if (size == 2) {
        // W5500 wysyła big-endian (MSB first)
        // Konwertujemy do little-endian (STM32)
        byte_data[1] = spi_read(w5500->spi);  // MSB (starszy bajt)
        byte_data[0] = spi_read(w5500->spi);  // LSB (młodszy bajt)
    } else {
        // Dla 8-bit: odczytujemy bezpośrednio
        byte_data[0] = spi_read(w5500->spi);
    }

    // Deaktywacja CS (HIGH)
    spi_set_cs(w5500->spi, false);
}

void W5500_WriteIPAddress(const w5500_t* w5500, uint16_t base_addr, uint8_t bsb, const uint8_t* ip){
    // Walidacja parametrów (podobnie jak w spi_init_peripheral)
    if (!w5500 || !w5500->spi || !ip) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane (podobnie jak w spi_init_peripheral)
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    // Zapisz każdy bajt IP do oddzielnego rejestru
    for (int i = 0; i < 4; i++) {
        W5500_WriteReg(w5500, base_addr + i, bsb, &ip[i], 1);
    }
}

void W5500_ReadIPAddress(const w5500_t* w5500, uint16_t base_addr, uint8_t bsb, uint8_t* ip){
    // Walidacja parametrów
    if (!w5500 || !w5500->spi || !ip) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

    // Odczytaj każdy bajt IP z oddzielnego rejestru
    for (int i = 0; i < 4; i++) {
        W5500_ReadReg(w5500, base_addr + i, bsb, &ip[i], 1);
    }
}

