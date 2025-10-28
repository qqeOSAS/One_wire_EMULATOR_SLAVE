#include "Emulator.h"
#include <Arduino.h>

Emulator::Emulator(uint8_t ID1, uint8_t ID2, uint8_t ID3, uint8_t ID4,
                   uint8_t ID5, uint8_t ID6, uint8_t ID7)
    : OneWireItem(ID1, ID2, ID3, ID4, ID5, ID6, ID7)
{
    memset(scratchpad, 0, sizeof(scratchpad));
    lastCommand = 0x00;
    rawBufferLen = 0;
    int32_value = 0;
    float_value = 0.0f;
    customHandler = nullptr;
}

uint8_t crc8_local(const uint8_t *data, size_t len, uint8_t crc_init = 0) {
    uint8_t crc = crc_init;
    while (len--) {
        uint8_t in = *data++;
        for (uint8_t i = 0; i < 8; ++i) {
            uint8_t mix = (crc ^ in) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            in >>= 1;
        }
    }
    return crc;
}

// головна функція для обробки подій на шині
void Emulator::duty(OneWireHub *hub)
{   
    uint8_t low_cmd;
    if(hub->recv(&low_cmd, 1)) return; // якщо нічого не прийшло
    else{
        Serial.print(F("Emulator received low-level CMD: 0x")); Serial.println(low_cmd, HEX);
    }

    switch(low_cmd){
        case OW_LOW_CMD_SEND_VARIABLE_:
        {
            uint8_t packet_header[2];
            if (hub->recv(packet_header, 2)) return; // CMD + LEN

            uint8_t cmd = packet_header[0];
            uint8_t len = packet_header[1];
            uint8_t payload[256];

            if (len > sizeof(payload)) { hub->raiseDeviceError(cmd); return; }
            if (len) {
                if (hub->recv(payload, len)) { hub->raiseDeviceError(cmd); return; }
            }

            uint8_t recv_crc;
            if (hub->recv(&recv_crc, 1)) { hub->raiseDeviceError(cmd); return; }

            // перевірка CRC
            uint8_t calc_crc = crc8_local(packet_header, 2);       // CMD + LEN
            calc_crc = crc8_local(payload, len, calc_crc);         // payload
            if (calc_crc != recv_crc) {
                hub->raiseDeviceError(cmd);
                return;
            }

            // зберігаємо останню команду
            lastCommand = cmd;

            bool handled = false;

            // обробка типів даних
            switch(cmd){
                case OW_CMD_INT8:
                    if(len != 1) { hub->raiseDeviceError(cmd); break; }
                    int8_value = (int8_t)payload[0];
                    Serial.print(F("Received INT8: ")); Serial.println(int8_value);
                    handled = true;
                    break;

                case OW_CMD_INT16:
                    if(len != 2) { hub->raiseDeviceError(cmd); break; }
                    int16_value = (int16_t)(payload[0] | (payload[1]<<8));
                    Serial.print(F("Received INT16: ")); Serial.println(int16_value);
                    handled = true;
                    break;

                case OW_CMD_UINT16:
                    if(len != 2) { hub->raiseDeviceError(cmd); break; }
                    uint16_value = (uint16_t)(payload[0] | (payload[1]<<8));
                    Serial.print(F("Received UINT16: ")); Serial.println(uint16_value);
                    handled = true;
                    break;

                case OW_CMD_INT32:
                    if(len != 4) { hub->raiseDeviceError(cmd); break; }
                    int32_value = (int32_t)(payload[0] | (payload[1]<<8) | (payload[2]<<16) | (payload[3]<<24));
                    Serial.print(F("Received INT32: ")); Serial.println(int32_value);
                    handled = true;
                    break;

                case OW_CMD_UINT32:
                    if(len != 4) { hub->raiseDeviceError(cmd); break; }
                    uint32_value = (uint32_t)(payload[0] | (payload[1]<<8) | (payload[2]<<16) | (payload[3]<<24));
                    Serial.print(F("Received UINT32: ")); Serial.println(uint32_value);
                    handled = true;
                    break;

                case OW_CMD_FLOAT32:
                    if(len != 4) { hub->raiseDeviceError(cmd); break; }
                    memcpy(&float_value, payload, 4);
                    Serial.print(F("Received FLOAT32: ")); Serial.println(float_value, 4);
                    handled = true;
                    break;

                case OW_CMD_STRUCT:
                    rawBufferLen = (len > sizeof(rawBuffer)) ? sizeof(rawBuffer) : len;
                    memcpy(rawBuffer, payload, rawBufferLen);
                    Serial.print(F("Received STRUCT, length: ")); Serial.println(rawBufferLen);
                    handled = true;
                    break;

                default:
                    // якщо кастомний хендлер встановлений — передати йому команду
                    if(customHandler) {
                        handled = customHandler(cmd);
                        if(handled){
                            Serial.print(F("Custom handler processed CMD: 0x")); Serial.println(cmd, HEX);
                        }
                    }

                    if(!handled){
                        Serial.print(F("Unknown CMD: 0x")); Serial.println(cmd, HEX);
                        hub->raiseDeviceError(cmd);
                    }
                    break;
            }

            // відправка ACK якщо команда оброблена
            if(handled){
                uint8_t ack = OW_CMD_ACK;
                hub->send(&ack, 1);
            }
        }
        break;
    }
}


// --- Геттери ---
int8_t Emulator::getInt8() const { return int8_value; }
int16_t Emulator::getInt16() const { return int16_value; }
uint16_t Emulator::getUInt16() const { return uint16_value; }
int32_t Emulator::getInt32() const { return int32_value; }
uint32_t Emulator::getUInt32() const { return uint32_value; }
float Emulator::getFloat() const { return float_value; }
const uint8_t* Emulator::getRawBuffer() const { return rawBuffer; }
uint8_t Emulator::getRawBufferLen() const { return rawBufferLen; }

// встановлюємо значення у scratchpad (як приклад)
void Emulator::setValue(int value) { scratchpad[0] = value & 0xFF; }
int Emulator::getValue() const { return scratchpad[0]; }

// збереження “сирих” даних
void Emulator::captureRaw(const uint8_t *data, uint8_t len) {
    rawBufferLen = min(len, (uint8_t)sizeof(rawBuffer));
    memcpy(rawBuffer, data, rawBufferLen);
}

// доступ до останнього отриманого байту / команди
uint8_t Emulator::getLastCommand() const { return lastCommand; }

// виведення всіх отриманих байтів
void Emulator::printRaw() const {
    Serial.print(F("Raw buffer ["));
    Serial.print(rawBufferLen);
    Serial.println(F(" bytes]:"));
    for (uint8_t i = 0; i < rawBufferLen; i++) {
        Serial.print(F("0x"));
        Serial.print(rawBuffer[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}

// встановлення callback
void Emulator::setCustomHandler(std::function<bool(uint8_t)> handler) { customHandler = handler; }
