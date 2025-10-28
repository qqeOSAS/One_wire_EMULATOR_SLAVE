#pragma once
#include <Arduino.h>
#include <OneWireHub.h>
#include <OneWireItem.h>
#include <functional>

// Packet command definitions
#define OW_LOW_CMD_SEND_VARIABLE_ 0x01  // відправка змінної зі слейва
#define OW_CMD_INT8        0x0F  // payload: 1 byte (int8_t)
#define OW_CMD_INT16       0x0E  // payload: 2 bytes (int16_t, LSB first)
#define OW_CMD_UINT16      0x0D  // payload: 2 bytes (uint16_t, LSB first)
#define OW_CMD_UINT32      0x12  // payload: 4 bytes (uint32_t, LSB first)
#define OW_CMD_INT32       0x10  // payload: 4 bytes (int32_t, LSB first)
#define OW_CMD_FLOAT32     0x11  // payload: 4 bytes (IEEE754 float, LSB first)
#define OW_CMD_CHAR8       0x13  // payload: 1 byte (char)
#define OW_CMD_STRUCT      0x14  // payload: N bytes (структура, LEN в заголовку)

#define OW_CMD_REQUEST     0x20  // майстер просить значення від слейва
#define OW_CMD_ACK         0x30  // підтвердження прийому
#define OW_CMD_NACK        0x31  // помилка

// Формат пакету для передачі даних між майстром та слейвом
// [CMD SEND_VARIABLE | CMD_variable | LEN | PAYLOAD... | CRC8 ]
//        1               1       1       N         1

class Emulator : public OneWireItem
{
private:
    uint8_t scratchpad[9];
    uint8_t rawBuffer[32];
    uint8_t rawBufferLen;
    uint8_t lastCommand;

    // значення різних типів для отриманих пакетів
    int8_t int8_value;
    int16_t int16_value;
    uint16_t uint16_value;
    int32_t int32_value;
    uint32_t uint32_value;
    float float_value;

    std::function<bool(uint8_t)> customHandler; // callback для кастомних команд

public:
    Emulator(uint8_t ID1, uint8_t ID2, uint8_t ID3, uint8_t ID4,
             uint8_t ID5, uint8_t ID6, uint8_t ID7);

    void duty(OneWireHub *hub);
    void setValue(int value);
    int getValue() const;

    // --- нові функції ---
    void captureRaw(const uint8_t *data, uint8_t len);
    uint8_t getLastCommand() const;
    void printRaw() const;
    void setCustomHandler(std::function<bool(uint8_t)> handler);

    // --- геттери для різних типів даних ---
    int8_t getInt8() const;
    int16_t getInt16() const;
    uint16_t getUInt16() const;
    int32_t getInt32() const;
    uint32_t getUInt32() const;
    float getFloat() const;
    const uint8_t* getRawBuffer() const;
    uint8_t getRawBufferLen() const;
};
