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
    // Walidacja parametrów (podobnie jak w spi_init_peripheral)
    if (!w5500 || !w5500->spi) {
        return;
    }

    // Sprawdzenie czy SPI jest skonfigurowane
    if (!(w5500->spi->config->spi->CR1 & SPI_CR1_SPE)) {
        return;
    }

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

    // 4. Konfiguracja PHY (zgodnie z datasheetem i forum WIZnet)
    // Sequence: OPMD=1, OPMDC=config, RST=0, RST=1
    uint8_t phy_config = W5500_PHYCFGR_OPMD | (w5500->network.phy_mode << W5500_PHYCFGR_OPMDC_Pos);
    W5500_WriteReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_config, 1);

    // PHY Reset (RST=0)
    phy_config &= ~W5500_PHYCFGR_RST;
    W5500_WriteReg(w5500, W5500_PHYCFGR, W5500_BSB_COMMON_REG, &phy_config, 1);

    // Krótkie opóźnienie dla PHY reset
    for (volatile uint32_t i = 0; i < 100000; i++){};

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
    spi_set_cs(w5500->spi, false);

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
    spi_set_cs(w5500->spi, true);
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
    spi_set_cs(w5500->spi, false);

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
    spi_set_cs(w5500->spi, true);
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

