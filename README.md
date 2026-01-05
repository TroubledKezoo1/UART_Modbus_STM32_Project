# UART Modbus STM32 Project

STM32 mikrodenetleyicisi ile Modbus RTU protokolü implementasyonu - Endüstriyel seri haberleşme projesi.

## 📋 Proje Hakkında

Bu proje, STM32F103 mikrodenetleyicisi kullanarak Modbus RTU (Remote Terminal Unit) protokolünün UART üzerinden implementasyonunu içerir. Endüstriyel otomasyon sistemlerinde yaygın olarak kullanılan bu protokol ile cihazlar arası güvenilir veri iletişimi sağlanır.

## 🎯 Özellikler

- **Modbus RTU Master/Slave**: Hem master hem slave mod desteği
- **UART İletişim**: RS485/RS232 üzerinden haberleşme
- **CRC16 Doğrulama**: Veri bütünlüğü kontrolü
- **Function Code Desteği**: 0x03, 0x04, 0x05, 0x06, 0x0F, 0x10
- **Register Management**: Holding ve Input register yönetimi
- **Exception Handling**: Modbus exception response
- **Timeout Kontrolü**: Frame timeout yönetimi
- **DMA Support**: Yüksek performanslı veri transferi

## 🛠️ Teknolojiler

- **C Dili** (100%)
- **STM32 HAL Library**: UART ve Timer periferalleri
- **Modbus RTU Protocol**: Seri iletişim protokolü
- **RS485 Interface**: Diferansiyel sinyal iletimi
- **CRC Algorithm**: Cyclic Redundancy Check

## 📦 Donanım Gereksinimleri

### Gerekli Bileşenler
- STM32F103C8T6 (Blue Pill) veya benzeri
- MAX485 / SN75176 RS485 transceiver modülü
- USB-Serial dönüştürücü (debug için)
- Breadboard ve jumper kablolar
- 120Ω termination direnci (hat sonları için)

### Pin Bağlantıları

```
STM32 ↔ MAX485
━━━━━━━━━━━━━━━━━━━━━━━
PA9  (TX) → DI (Data In)
PA10 (RX) → RO (Receiver Out)
PA8  (GPIO)→ DE/RE (Direction Enable)
3.3V      → VCC
GND       → GND

RS485 Bus Topology
━━━━━━━━━━━━━━━━━━━━━━━
Master → [120Ω] ← Slave1 ← Slave2 ← [120Ω]
   A ←→ A         A        A
   B ←→ B         B        B
```

## 📥 Kurulum

### 1. Projeyi Klonlayın

```bash
git clone https://github.com/TroubledKezoo1/UART_Modbus_STM32_Project.git
cd UART_Modbus_STM32_Project
```

### 2. STM32CubeIDE'de Açın

```bash
File → Open Projects from File System
```

### 3. UART Konfigürasyonu

```c
// UART1 Configuration
Baud Rate: 9600 / 19200 / 38400
Word Length: 8 bits
Stop Bits: 1
Parity: None / Even
Mode: Asynchronous
```

### 4. Derleyin ve Yükleyin

```bash
# Derle
Project → Build Project

# Yükle
Run → Debug
```

## 💻 Kullanım

### Modbus Master Örneği

```c
#include "modbus.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    // UART ve Modbus başlat
    MX_USART1_UART_Init();
    Modbus_Init(MODBUS_MODE_MASTER, 1);  // Master mode, slave ID: 1
    
    uint16_t read_data[10];
    uint16_t write_data[5] = {100, 200, 300, 400, 500};
    
    while(1)
    {
        // Holding Register Oku (Function Code: 0x03)
        if(Modbus_ReadHoldingRegisters(1, 0x0000, 5, read_data) == MODBUS_OK)
        {
            printf("Read successful: %d, %d, %d\n", 
                   read_data[0], read_data[1], read_data[2]);
        }
        
        HAL_Delay(1000);
        
        // Single Register Yaz (Function Code: 0x06)
        Modbus_WriteSingleRegister(1, 0x0000, 1234);
        
        HAL_Delay(1000);
        
        // Multiple Registers Yaz (Function Code: 0x10)
        Modbus_WriteMultipleRegisters(1, 0x0000, 5, write_data);
        
        HAL_Delay(1000);
    }
}
```

### Modbus Slave Örneği

```c
#include "modbus.h"

// Register tanımlamaları
uint16_t holding_registers[100];
uint16_t input_registers[100];

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    // UART ve Modbus başlat
    MX_USART1_UART_Init();
    Modbus_Init(MODBUS_MODE_SLAVE, 1);  // Slave mode, address: 1
    
    // Register callback'lerini ayarla
    Modbus_RegisterCallback(MODBUS_FC_READ_HOLDING_REGS, read_holding_callback);
    Modbus_RegisterCallback(MODBUS_FC_WRITE_SINGLE_REG, write_single_callback);
    
    // İlk değerleri ata
    holding_registers[0] = 1000;
    holding_registers[1] = 2000;
    input_registers[0] = 100;
    
    while(1)
    {
        // Modbus frame'lerini işle
        Modbus_Process();
        
        // Sensör verilerini güncelle
        input_registers[0]++;  // Simüle edilmiş sensör verisi
        
        HAL_Delay(10);
    }
}

// Holding Register okuma callback
void read_holding_callback(uint16_t address, uint16_t quantity, uint16_t *data)
{
    for(uint16_t i = 0; i < quantity; i++)
    {
        data[i] = holding_registers[address + i];
    }
}

// Single Register yazma callback
void write_single_callback(uint16_t address, uint16_t value)
{
    holding_registers[address] = value;
}
```

## 📋 Modbus Function Codes

### Desteklenen Fonksiyonlar

| Code | İsim | Açıklama | Master | Slave |
|------|------|----------|---------|-------|
| 0x01 | Read Coils | Coil (bit) oku | ✅ | ✅ |
| 0x03 | Read Holding Registers | Holding register oku | ✅ | ✅ |
| 0x04 | Read Input Registers | Input register oku | ✅ | ✅ |
| 0x05 | Write Single Coil | Tek coil yaz | ✅ | ✅ |
| 0x06 | Write Single Register | Tek register yaz | ✅ | ✅ |
| 0x0F | Write Multiple Coils | Çoklu coil yaz | ✅ | ✅ |
| 0x10 | Write Multiple Registers | Çoklu register yaz | ✅ | ✅ |

### Function Code Örnekleri

#### 0x03 - Read Holding Registers

```c
// Master gönderir:
// [Slave ID][0x03][Start Addr Hi][Start Addr Lo][Quantity Hi][Quantity Lo][CRC Lo][CRC Hi]
// Örnek: 01 03 00 00 00 0A C5 CD

Modbus_ReadHoldingRegisters(
    1,        // Slave address
    0x0000,   // Starting address
    10,       // Quantity of registers
    buffer    // Receive buffer
);

// Slave cevap verir:
// [Slave ID][0x03][Byte Count][Data...][CRC Lo][CRC Hi]
// Örnek: 01 03 14 00 64 00 C8 ... 00 FA XX XX
```

#### 0x06 - Write Single Register

```c
// Master gönderir:
// [Slave ID][0x06][Reg Addr Hi][Reg Addr Lo][Value Hi][Value Lo][CRC Lo][CRC Hi]
// Örnek: 01 06 00 01 00 03 9A 0B

Modbus_WriteSingleRegister(
    1,        // Slave address
    0x0001,   // Register address
    0x0003    // Value to write
);

// Slave aynı frame'i echo eder (başarılı ise)
```

#### 0x10 - Write Multiple Registers

```c
uint16_t data[] = {100, 200, 300};

// Master gönderir:
// [Slave ID][0x10][Start Hi][Start Lo][Qty Hi][Qty Lo][Byte Count][Data...][CRC]
// Örnek: 01 10 00 00 00 03 06 00 64 00 C8 01 2C XX XX

Modbus_WriteMultipleRegisters(
    1,        // Slave address
    0x0000,   // Starting address
    3,        // Quantity
    data      // Data array
);
```

## 🔒 CRC16 Hesaplama

### Modbus CRC-16 Algoritması

```c
uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for(uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)buffer[i];
        
        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;  // Polynomial
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

// Kullanım
uint8_t frame[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
uint16_t crc = Modbus_CRC16(frame, 6);
frame[6] = crc & 0xFF;         // CRC Low
frame[7] = (crc >> 8) & 0xFF;  // CRC High
```

## 📊 Frame Yapısı

### Request Frame (Master → Slave)

```
┌─────────────┬──────────────┬─────────┬──────────┬─────────┐
│  Slave ID   │ Function Code│ Address │   Data   │   CRC   │
│   (1 byte)  │   (1 byte)   │(2 bytes)│ (N bytes)│(2 bytes)│
└─────────────┴──────────────┴─────────┴──────────┴─────────┘

Örnek (Read 10 registers from address 0):
┌────┬────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ 01 │ 03 │ 00  │ 00  │ 00  │ 0A  │ C5  │ CD  │
└────┴────┴─────┴─────┴─────┴─────┴─────┴─────┘
  ID   FC   Addr   Addr  Qty   Qty   CRC   CRC
           (Hi)   (Lo)  (Hi)  (Lo)  (Lo)  (Hi)
```

### Response Frame (Slave → Master)

```
┌─────────────┬──────────────┬────────────┬──────────┬─────────┐
│  Slave ID   │ Function Code│ Byte Count │   Data   │   CRC   │
│   (1 byte)  │   (1 byte)   │  (1 byte)  │ (N bytes)│(2 bytes)│
└─────────────┴──────────────┴────────────┴──────────┴─────────┘

Örnek (10 registers, values: 100, 200, ..., 1000):
┌────┬────┬────┬─────┬─────┬─────┬─────┬───┬─────┬─────┐
│ 01 │ 03 │ 14 │ 00  │ 64  │ 00  │ C8  │...│ XX  │ XX  │
└────┴────┴────┴─────┴─────┴─────┴─────┴───┴─────┴─────┘
  ID   FC  Cnt  Data  Data  Data  Data     CRC   CRC
                (Hi)  (Lo)  (Hi)  (Lo)     (Lo)  (Hi)
```

## ⚠️ Exception Responses

### Exception Codes

| Code | İsim | Açıklama |
|------|------|----------|
| 0x01 | Illegal Function | Desteklenmeyen function code |
| 0x02 | Illegal Data Address | Geçersiz register adresi |
| 0x03 | Illegal Data Value | Geçersiz veri değeri |
| 0x04 | Slave Device Failure | Slave cihaz hatası |

### Exception Frame

```c
// Normal response: 01 03 14 ...
// Exception response: 01 83 02 XX XX
//                        ^^ Exception code (0x80 + FC)
//                           ^^ Exception reason

void Modbus_SendException(uint8_t function_code, uint8_t exception_code)
{
    uint8_t frame[5];
    frame[0] = slave_address;
    frame[1] = function_code | 0x80;  // Set MSB
    frame[2] = exception_code;
    
    uint16_t crc = Modbus_CRC16(frame, 3);
    frame[3] = crc & 0xFF;
    frame[4] = (crc >> 8) & 0xFF;
    
    HAL_UART_Transmit(&huart1, frame, 5, 100);
}
```

## 🎓 Timing ve Protokol Detayları

### Character Timeout (T1.5)

```c
// T1.5 = 1.5 * (11 bits / baud rate) * 1000 ms
// @9600 baud: T1.5 = 1.5 * (11 / 9600) * 1000 = 1.72 ms

#define T1_5_TIMEOUT  2  // ms
```

### Frame Timeout (T3.5)

```c
// T3.5 = 3.5 * (11 bits / baud rate) * 1000 ms
// @9600 baud: T3.5 = 3.5 * (11 / 9600) * 1000 = 4.01 ms

#define T3_5_TIMEOUT  5  // ms

// Frame bitişini algıla
if(HAL_GetTick() - last_char_time > T3_5_TIMEOUT)
{
    // Frame tamamlandı
    Modbus_ProcessFrame();
}
```

## 🧪 Test ve Debug

### Modbus RTU Tester

```c
void Modbus_Test(void)
{
    printf("=== Modbus RTU Test ===\n");
    
    // Test 1: Read Holding Registers
    printf("Test 1: Read Holding Registers\n");
    uint16_t data[10];
    if(Modbus_ReadHoldingRegisters(1, 0, 10, data) == MODBUS_OK)
    {
        printf("Success! Data: ");
        for(int i = 0; i < 10; i++)
            printf("%d ", data[i]);
        printf("\n");
    }
    
    // Test 2: Write Single Register
    printf("Test 2: Write Single Register\n");
    if(Modbus_WriteSingleRegister(1, 0, 1234) == MODBUS_OK)
    {
        printf("Success!\n");
    }
    
    // Test 3: CRC Verification
    printf("Test 3: CRC Verification\n");
    uint8_t test_frame[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
    uint16_t crc = Modbus_CRC16(test_frame, 6);
    printf("Calculated CRC: 0x%04X (Expected: 0xC5CD)\n", crc);
}
```

### Serial Monitor

```bash
# Linux/Mac
screen /dev/ttyUSB0 9600

# Windows (PuTTY)
# COM port, 9600 baud, 8N1

# Modbus Poll/Slave yazılımları
# QModMaster (Linux)
# Modbus Poll (Windows)
```

## 🔧 Yapılandırma

### modbus_config.h

```c
#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

// Modbus ayarları
#define MODBUS_SLAVE_ADDRESS    1
#define MODBUS_BAUD_RATE        9600
#define MODBUS_TIMEOUT          1000  // ms

// Buffer boyutları
#define MODBUS_MAX_BUFFER       256
#define MODBUS_MAX_REGISTERS    100

// Register adresleri
#define REG_ADDR_STATUS         0x0000
#define REG_ADDR_CONTROL        0x0010
#define REG_ADDR_SENSOR_1       0x0020
#define REG_ADDR_SENSOR_2       0x0021

// Function code enable/disable
#define MODBUS_FC01_ENABLED     1
#define MODBUS_FC03_ENABLED     1
#define MODBUS_FC04_ENABLED     1
#define MODBUS_FC05_ENABLED     1
#define MODBUS_FC06_ENABLED     1
#define MODBUS_FC15_ENABLED     1
#define MODBUS_FC16_ENABLED     1

#endif
```

## 📱 Örnek Uygulamalar

### 1. Sensör Okuma Sistemi

```c
// Slave cihaz (sensör modülü)
void sensor_module_task(void)
{
    uint16_t temperature = read_temperature_sensor();
    uint16_t humidity = read_humidity_sensor();
    
    holding_registers[REG_ADDR_SENSOR_1] = temperature * 10;
    holding_registers[REG_ADDR_SENSOR_2] = humidity * 10;
    
    Modbus_Process();
}

// Master cihaz (veri toplayıcı)
void data_collector_task(void)
{
    uint16_t sensor_data[2];
    
    if(Modbus_ReadHoldingRegisters(1, REG_ADDR_SENSOR_1, 2, sensor_data) == MODBUS_OK)
    {
        float temp = sensor_data[0] / 10.0;
        float hum = sensor_data[1] / 10.0;
        printf("Temp: %.1f°C, Humidity: %.1f%%\n", temp, hum);
    }
}
```

### 2. Kontrol Sistemi

```c
// Master kontrolcü
void control_system(void)
{
    uint16_t motor_speed = 1500;  // RPM
    uint16_t valve_position = 75; // %
    
    // Motor hızını ayarla
    Modbus_WriteSingleRegister(MOTOR_SLAVE_ID, REG_MOTOR_SPEED, motor_speed);
    
    // Valf pozisyonunu ayarla
    Modbus_WriteSingleRegister(VALVE_SLAVE_ID, REG_VALVE_POS, valve_position);
    
    // Durum bilgisini oku
    uint16_t status[5];
    Modbus_ReadInputRegisters(MOTOR_SLAVE_ID, REG_STATUS, 5, status);
}
```

### 3. Multi-Slave Network

```c
#define NUM_SLAVES  5

void poll_all_slaves(void)
{
    for(uint8_t slave = 1; slave <= NUM_SLAVES; slave++)
    {
        uint16_t data[10];
        
        if(Modbus_ReadHoldingRegisters(slave, 0, 10, data) == MODBUS_OK)
        {
            printf("Slave %d: OK\n", slave);
            process_slave_data(slave, data);
        }
        else
        {
            printf("Slave %d: Communication Error\n", slave);
        }
        
        HAL_Delay(50);  // Slave'ler arası gecikme
    }
}
```

## 🐛 Sorun Giderme

### CRC Hatası
```c
// CRC hatasını kontrol et
if(received_crc != calculated_crc)
{
    printf("CRC Error! Received: 0x%04X, Calculated: 0x%04X\n", 
           received_crc, calculated_crc);
    // Frame'i yoksay
}
```

### Timeout Problemleri
```c
// Timeout sürelerini artır
#define MODBUS_TIMEOUT  2000  // 1000'den 2000'e

// Baud rate'i düşür
// 38400 → 19200 → 9600
```

### RS485 Direction Control
```c
void Modbus_SetTxMode(void)
{
    HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_SET);
    HAL_Delay(1);  // Transceiver switch time
}

void Modbus_SetRxMode(void)
{
    HAL_Delay(1);  // Son bitin iletimini bekle
    HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_RESET);
}
```

## 📚 Faydalı Kaynaklar

- [Modbus Organization](https://modbus.org/)
- [Modbus Protocol Specification](https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf)
- [RS485 Standard (TIA-485-A)](https://www.ti.com/lit/an/slla070d/slla070d.pdf)
- [STM32 UART Guide](https://www.st.com/resource/en/application_note/dm00160528-getting-started-with-stm32f4xxxx-mcu-hardware-development-stmicroelectronics.pdf)

## 📝 Lisans

Bu proje açık kaynak kodludur.

## 📧 İletişim

Proje Sahibi: TroubledKezoo1

Proje Linki: [https://github.com/TroubledKezoo1/UART_Modbus_STM32_Project](https://github.com/TroubledKezoo1/UART_Modbus_STM32_Project)

---

⭐ Bu projeyi beğendiyseniz yıldız vermeyi unutmayın!

**Not**: Endüstriyel otomasyon sistemlerinde kullanılabilir.