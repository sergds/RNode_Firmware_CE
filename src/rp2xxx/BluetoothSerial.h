/* A btstack serial implementation for RNode, with SSP and callbacks. (BR/EDR, SPP)
** Copyright (c) 2026, Sergey Morozyuk <me@sergds.xyz>
** Loosely based on btstack examples and SerialBT from arduino-pico
** arduino-pico is Copyright (c) 2023 Earle F. Philhower, III <earlephilhower@yahoo.com>
*/
#pragma once

#include <Arduino.h>
#include "../../Boards.h"
#include "bluetooth.h"
#if HAS_BLUETOOTH == true
#include <Arduino.h>
#include <api/HardwareSerial.h>
#include <cstdint>
#include <functional>
#include <pico/mutex.h>
#include <CoreMutex.h>
#include <LocklessQueue.h>
#include <btstack_defines.h>
//#include "btstack_undefs.h" // needed because of hid conflicts with tinyusb

// Reusing arduino-pico technique
#define CCALLBACKNAME _CBRNODEBTUART
#include <ctocppcallback.h>

#define PACKETHANDLERCB(class, cbFcn) \
  (CCALLBACKNAME<void(uint8_t, uint16_t, uint8_t*, uint16_t), __COUNTER__>::func = std::bind(&class::cbFcn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), \
   static_cast<btstack_packet_handler_t>(CCALLBACKNAME<void(uint8_t, uint16_t, uint8_t*, uint16_t), __COUNTER__ - 1>::callback))

class BluetoothSerial: HardwareSerial {
    public:
    BluetoothSerial();
    bool setFIFOSize(size_t size);
    bool setName(const char* name);
    bool setBondable(bool isBondable);
    void disconnect();
    void setPairingCallback(void (*bt_confirm_pairing)(uint32_t)) {
        this->_bt_confirm_pairing = bt_confirm_pairing;
    }
    void setPairingCompleteCallback(void (*bt_pairing_complete)(bool)) {
        this->_bt_pairing_complete = bt_pairing_complete;
    }
    void setConnectionCallback(void (*bt_connection_callback)(bool)) {
        this->_bt_connection_callback = bt_connection_callback;
    }

    void begin(unsigned long baudrate = 115200) override {
        begin(baudrate, SERIAL_8N1);
    };
    void begin(unsigned long baudrate, uint16_t config) override;
    void end() override;
    int available(void) override;
    int availableForWrite(void) override;
    bool overflow(void);
    int peek(void) override;
    int read(void) override;
    void flush(void) override;
    size_t write(uint8_t) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    operator bool() override;
    using Print::write;
    uint16_t getTxbuflenght() {
        return _txLen;
    }
    unsigned long lastFlushTime = 0;

    private:
    void packetHandler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size);
    bool applyBondable();

    LocklessQueue<uint8_t> *_queue = nullptr;
    size_t   _fifoSize = 1024;
    btstack_packet_callback_registration_t _hci_event_callback_registration;
    mutex_t _mtx;
    bool _running = false;
    bool _overflow = false;
    volatile bool _connected = false;
    uint8_t _spp_service[255];
    const int RFCOMM_SERVER_CHANNEL = 1;
    char* _name = NULL;
    bool _isBondable = false;
    uint16_t _rfcommChannelID = 0;
    hci_con_handle_t _connHandle = HCI_CON_HANDLE_INVALID;
    //const void* _txBuf = nullptr;
    const void* _txBuf[1024];
    volatile uint16_t _txLen = 0;

    void (*_bt_confirm_pairing)(uint32_t) = nullptr;
    void (*_bt_pairing_complete)(bool) = nullptr;
    void (*_bt_connection_callback)(bool) = nullptr;
    
};
#endif