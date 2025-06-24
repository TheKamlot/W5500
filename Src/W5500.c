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
    W5500_ReadReg(w5500, W5500_Sn_RXRSR0, bsb, rx_size_bytes, 2);
    uint16_t available_data = (rx_size_bytes[0] << 8) | rx_size_bytes[1];

    if (available_data == 0) {
        return W5500_NO_DATA;
    }

    // Ogranicz do rozmiaru bufora
    uint16_t to_read = (available_data > max_len) ? max_len : available_data;

    // Pobierz RX Read Pointer
    uint8_t rx_rd_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_RXRD0, bsb, rx_rd_bytes, 2);
    uint16_t rx_rd_ptr = (rx_rd_bytes[0] << 8) | rx_rd_bytes[1];

    // Odczytaj dane przez DMA
    uint8_t rx_bsb = W5500_GetSocketBSB_RX(sock->socket_num);
    W5500_ReadBuffer(w5500, rx_rd_ptr, rx_bsb, buffer, to_read);

    // Aktualizuj RX Read Pointer
    rx_rd_ptr += to_read;
    rx_rd_bytes[0] = (uint8_t)(rx_rd_ptr >> 8);
    rx_rd_bytes[1] = (uint8_t)(rx_rd_ptr & 0xFF);
    W5500_WriteReg(w5500, W5500_Sn_RXRD0, bsb, rx_rd_bytes, 2);

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
    W5500_ReadReg(w5500, W5500_Sn_RXRSR0, bsb, rx_size_bytes, 2);
    uint16_t available_data = (rx_size_bytes[0] << 8) | rx_size_bytes[1];

    if (available_data < 8) {
        return W5500_NO_DATA; // Minimum nagłówek UDP
    }

    // Pobierz RX Read Pointer
    uint8_t rx_rd_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_RXRD0, bsb, rx_rd_bytes, 2);
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
    W5500_WriteReg(w5500, W5500_Sn_RXRD0, bsb, rx_rd_bytes, 2);

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
    W5500_ReadReg(w5500, W5500_Sn_RXRSR0, bsb, rx_size_bytes, 2);
    uint16_t available_data = (rx_size_bytes[0] << 8) | rx_size_bytes[1];

    if (available_data < 2) {
        return W5500_NO_DATA; // Minimum nagłówek długości
    }

    // Pobierz RX Read Pointer
    uint8_t rx_rd_bytes[2];
    W5500_ReadReg(w5500, W5500_Sn_RXRD0, bsb, rx_rd_bytes, 2);
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
    W5500_WriteReg(w5500, W5500_Sn_RXRD0, bsb, rx_rd_bytes, 2);

    // Potwierdź odczyt
    uint8_t command = W5500_Sn_CR_RECV;
    W5500_WriteReg(w5500, W5500_Sn_CR, bsb, &command, 1);

    return frame_length;
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

