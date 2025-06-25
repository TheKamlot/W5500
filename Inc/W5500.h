#ifndef W5500_H
#define W5500_H

#include "W5500_REG.h"
#include "spi.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Kody błędów
#define W5500_OK                 0
#define W5500_ERROR             -1
#define W5500_BUSY              -2
#define W5500_TIMEOUT           -3
#define W5500_INVALID_SOCKET    -4
#define W5500_BUFFER_FULL       -5
#define W5500_INCOMPLETE_PACKET -6  // Pakiet niekompletny
#define W5500_NO_DATA           -7   // Brak danych

// Maksymalna liczba socketów
#define W5500_MAX_SOCKETS       8

// Rozmiary buforów (w KB)
#define W5500_BUFFER_SIZE_1KB   1
#define W5500_BUFFER_SIZE_2KB   2
#define W5500_BUFFER_SIZE_4KB   4
#define W5500_BUFFER_SIZE_8KB   8
#define W5500_BUFFER_SIZE_16KB  16

// Struktura konfiguracji sieciowej W5500
typedef struct {
    uint8_t mac[6];              // Adres MAC
    uint8_t ip[4];               // Adres IP
    uint8_t subnet[4];           // Maska podsieci
    uint8_t gateway[4];          // Brama domyślna
    uint8_t phy_mode;            // Tryb PHY
} w5500_network_config_t;

// Struktura socketu (prosta, podobna do Twojego spi_t)
typedef struct {
    uint8_t socket_num;          // Numer socketu (0-7)
    uint8_t protocol;            // TCP/UDP/MACRAW
    uint16_t source_port;        // Port źródłowy
    uint8_t status;              // Cache statusu

    // Informacje o połączeniu
    uint8_t dest_ip[4];          // IP docelowe
    uint16_t dest_port;          // Port docelowy

    // Flagi stanu (kompaktowe)
    struct {
        volatile bool is_open : 1;
        volatile bool is_connected : 1;
        volatile bool has_data : 1;
        uint8_t reserved : 5;
    } flags;
} socket_t;

// Struktura główna W5500 (podobna do Twojego spi_t)
typedef struct {
    volatile bool is_initialized;
    volatile bool phy_link_up;
    const spi_t* spi;                    // Wskaźnik do SPI
    w5500_network_config_t network;      // Konfiguracja sieciowa
    socket_t sockets[W5500_MAX_SOCKETS]; // Tablica socketów
    uint8_t active_sockets;              // Liczba aktywnych socketów
} w5500_t;

// Struktura dla informacji o pakiecie UDP/MACRAW
typedef struct {
    uint8_t src_ip[4];           // IP nadawcy (tylko UDP)
    uint16_t src_port;           // Port nadawcy (tylko UDP)
    uint8_t src_mac[6];          // MAC nadawcy (tylko MACRAW)
    uint16_t ethertype;          // EtherType (tylko MACRAW)
    uint16_t packet_length;      // Długość danych pakietu
    bool is_complete;            // Czy pakiet jest kompletny
} w5500_packet_info_t;


// ============================================================================
// FUNKCJE INICJALIZACJI I KONFIGURACJI
// ============================================================================

// Inicjalizacja z domyślną konfiguracją
void W5500_InitDefault(w5500_t* w5500, const spi_t* spi);                                                       //S

// Inicjalizacja z konfiguracją sieciową
void W5500_InitConfig(w5500_t* w5500, const spi_t* spi, const w5500_network_config_t* config);                  //S

// Inicjalizacja peryferiów (podobnie jak w SPI)
void W5500_InitPeripheral(w5500_t* w5500);                                                                      //S

// Reset układu
void W5500_Reset(const w5500_t* w5500);
void W5500_SoftReset(const w5500_t* w5500);

// ============================================================================
// FUNKCJE NISKOPOZIOMOWEJ KOMUNIKACJI SPI
// ============================================================================

// Podstawowe operacje SPI (podobne do Twojego API SPI)
void W5500_WriteReg(const w5500_t* w5500, uint16_t addr, uint8_t bsb, const void* data, uint8_t size);          //S
void W5500_ReadReg(const w5500_t* w5500, uint16_t addr, uint8_t bsb, void* data, uint8_t size);                 //S
void W5500_WriteIPAddress(const w5500_t* w5500, uint16_t base_addr, uint8_t bsb, const uint8_t* ip);            //S
void W5500_ReadIPAddress(const w5500_t* w5500, uint16_t base_addr, uint8_t bsb, uint8_t* ip);                   //S
void W5500_WriteBuffer(const w5500_t* w5500, uint16_t addr, uint8_t bsb, const uint8_t* data, uint16_t len);
void W5500_ReadBuffer(const w5500_t* w5500, uint16_t addr, uint8_t bsb, uint8_t* data, uint16_t len);

// ============================================================================
// FUNKCJE KONFIGURACJI SIECIOWEJ
// ============================================================================

// Konfiguracja sieciowa
void W5500_SetNetworkConfig(w5500_t* w5500, const w5500_network_config_t* config);
void W5500_GetNetworkConfig(const w5500_t* w5500, w5500_network_config_t* config);

// Konfiguracja PHY
void W5500_SetPHYConfig(const w5500_t* w5500, uint8_t mode);
uint8_t W5500_GetPHYConfig(const w5500_t* w5500);

// Status łącza
bool W5500_IsLinkUp(const w5500_t* w5500);
void W5500_UpdateStatus(w5500_t* w5500);

// ============================================================================
// FUNKCJE ZARZĄDZANIA SOCKETAMI
// ============================================================================

// Inicjalizacja socketu (podobnie jak w SPI)
int8_t W5500_SocketInit(socket_t* sock, uint8_t socket_num, uint8_t protocol, uint16_t port);

// Podstawowe operacje na socketach
int8_t W5500_SocketOpen(const w5500_t* w5500, socket_t* sock);
int8_t W5500_SocketClose(const w5500_t* w5500, socket_t* sock);

// Operacje TCP
int8_t W5500_SocketListen(const w5500_t* w5500, socket_t* sock);
int8_t W5500_SocketConnect(const w5500_t* w5500, socket_t* sock, const uint8_t* dest_ip, uint16_t dest_port);
int8_t W5500_SocketDisconnect(const w5500_t* w5500, socket_t* sock);

// ============================================================================
// FUNKCJE TRANSMISJI DANYCH
// ============================================================================

// Transmisja danych (podobnie jak spi_dma_send/read)
int16_t W5500_SocketSend(const w5500_t* w5500, socket_t* sock, const uint8_t* data, uint16_t len);
int16_t W5500_SocketSendTo(const w5500_t* w5500, socket_t* sock, const uint8_t* data, uint16_t len,
                            const uint8_t* dest_ip, uint16_t dest_port);
int16_t W5500_SocketSendRaw(const w5500_t* w5500, socket_t* sock, const uint8_t* data, uint16_t len);

//  Odbiór danych
int16_t W5500_SocketReceiveTCP(const w5500_t* w5500, socket_t* sock,uint8_t* buffer, uint16_t max_len);
int16_t W5500_SocketReceiveUDP(const w5500_t* w5500, socket_t* sock, uint8_t* buffer, uint16_t max_len,
                                w5500_packet_info_t* packet_info);
int16_t W5500_SocketReceiveMACRAW(const w5500_t* w5500, socket_t* sock, uint8_t* buffer,
                                uint16_t max_len, w5500_packet_info_t* packet_info);

// ============================================================================
// FUNKCJE MONITOROWANIA STANU (podobnie jak spi_is_dma_busy itp.)
// ============================================================================

// Status socketów
void W5500_SocketUpdateStatus(const w5500_t* w5500, socket_t* sock);
uint16_t W5500_SocketGetTxFreeSize(const w5500_t* w5500, const socket_t* sock);
uint16_t W5500_SocketGetRxDataSize(const w5500_t* w5500, const socket_t* sock);
bool W5500_SocketHasData(const w5500_t* w5500, const socket_t* sock);
bool W5500_SocketCanSend(const w5500_t* w5500, const socket_t* sock, uint16_t size);

// ============================================================================
// FUNKCJE POMOCNICZE (podobnie jak w SPI)
// ============================================================================

// Informacje o układzie
uint8_t W5500_GetVersion(const w5500_t* w5500);
bool W5500_SelfTest(const w5500_t* w5500);

// Zarządzanie buforami
void W5500_SetBufferSizes(const w5500_t* w5500, const uint8_t* tx_sizes, const uint8_t* rx_sizes);

// Poprawione funkcje BSB (zgodnie z datasheetem)
static inline uint8_t W5500_GetSocketBSB_REG(uint8_t socket_num) {
    return W5500_BSB_SOCKET0_REG + (socket_num * 4);
}

static inline uint8_t W5500_GetSocketBSB_TX(uint8_t socket_num) {
    return W5500_BSB_SOCKET0_TX + (socket_num * 4);
}

static inline uint8_t W5500_GetSocketBSB_RX(uint8_t socket_num) {
    return W5500_BSB_SOCKET0_RX + (socket_num * 4);
}

#endif // W5500_H
