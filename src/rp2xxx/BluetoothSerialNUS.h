/* A btstack serial implementation for RNode, with SSP and callbacks. (BLE, Nordic UART Service)
** Copyright (c) 2026, Sergey Morozyuk <me@sergds.xyz>
** Loosely based on btstack examples and SerialBT from arduino-pico
** arduino-pico is Copyright (c) 2023 Earle F. Philhower, III <earlephilhower@yahoo.com>
*/
#pragma once

#include <Arduino.h>
#include "../../Boards.h"
#if HAS_BLE == true

#include "bluetooth.h"
#include <Arduino.h>
#include <api/HardwareSerial.h>
#include <cstdint>
#include <functional>
#include <pico/mutex.h>
#include <CoreMutex.h>
#include <LocklessQueue.h>
#include <btstack_defines.h>

// Reusing arduino-pico technique
#define CCALLBACKNAME _CBRNODEBTUART
#include <ctocppcallback.h>

#define PACKETHANDLERCB(class, cbFcn) \
  (CCALLBACKNAME<void(uint8_t, uint16_t, uint8_t*, uint16_t), __COUNTER__>::func = std::bind(&class::cbFcn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), \
   static_cast<btstack_packet_handler_t>(CCALLBACKNAME<void(uint8_t, uint16_t, uint8_t*, uint16_t), __COUNTER__ - 1>::callback))

#define ATTREADHANDLERCB(class, cbFcn) \
  (CCALLBACKNAME<uint16_t(hci_con_handle_t, uint16_t, uint16_t, uint8_t*, uint16_t), __COUNTER__>::func = std::bind(&class::cbFcn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5), \
   static_cast<att_read_callback_t>(CCALLBACKNAME<uint16_t(hci_con_handle_t, uint16_t, uint16_t, uint8_t*, uint16_t), __COUNTER__ - 1>::callback))

typedef void (btstack_context_callback_t)(void* context);

#define CONTEXTCB(class, cbFcn) \
  (CCALLBACKNAME<void(void*), __COUNTER__>::func = std::bind(&class::cbFcn, this, std::placeholders::_1), \
   static_cast<void(*)(void*)>(CCALLBACKNAME<void(void*), __COUNTER__ - 1>::callback))

// How many advertising data slots do we want
#define BLE_ADV_SLOTS 2

class BluetoothSerialNUS: HardwareSerial {
    public:
    BluetoothSerialNUS();
    bool setFIFOSize(size_t size);
    bool setName(const char* name);
    bool setBondable(bool isBondable);
    void disconnect();
    bd_addr_t* getLocalAddr() {
        return &_local_addr;
    }
    void setPairingCallback(void (*bt_confirm_pairing)(uint32_t)) {
        this->_bt_confirm_pairing = bt_confirm_pairing;
    }
    void setPairingCompleteCallback(void (*bt_pairing_complete)(bool)) {
        this->_bt_pairing_complete = bt_pairing_complete;
    }
    void setConnectionCallback(void (*bt_connection_callback)(bool)) {
        this->_bt_connection_callback = bt_connection_callback;
    }
    uint16_t getTxbuflenght() {
        return _txLen;
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
    unsigned long lastFlushTime = 0;

    private:
    void packetHandler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size);
    void smPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    void nordicCanSend(void* context);
    uint16_t attReadHandler(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
    bool applyBondable();
    bool appendAdvertisingDataElement(uint8_t slot, uint8_t len, uint8_t type, uint8_t* data);
    void clearAdvertisingData(uint8_t slot);
    void setupAdvertising();
    void rebuildAdvertisingData();

    uint8_t _adv_data[31 * BLE_ADV_SLOTS] = {0};
    size_t _adv_data_len[BLE_ADV_SLOTS] = {0};
    LocklessQueue<uint8_t> *_queue = nullptr;
    size_t   _fifoSize = 1024;
    btstack_packet_callback_registration_t _hci_event_callback_registration;
    btstack_packet_callback_registration_t _sm_event_callback_registration;
    mutex_t _mtx;
    bool _running = false;
    bool _overflow = false;
    volatile bool _connected = false;
    uint8_t _spp_service[255];
    const int RFCOMM_SERVER_CHANNEL = 1;
    char* _name = NULL;
    bool _isBondable = false;
    hci_con_handle_t _conHandle = HCI_CON_HANDLE_INVALID;
    btstack_context_callback_registration_t _sendRequest;
    // const uint8_t _txBuf[2048] = {0};
    uint8_t* _txBuf = nullptr;
    volatile uint16_t _txLen = 0;
    uint16_t _flushingTime = 0;
    bd_addr_t _local_addr = {0};

    void (*_bt_confirm_pairing)(uint32_t) = nullptr;
    void (*_bt_pairing_complete)(bool) = nullptr;
    void (*_bt_connection_callback)(bool) = nullptr;
    
};
#endif