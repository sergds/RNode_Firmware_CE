/* A btstack serial implementation for RNode, with SSP and callbacks. (BR/EDR, SPP)
** Copyright (c) 2026, Sergey Morozyuk <me@sergds.xyz>
** Loosely based on btstack examples and SerialBT from arduino-pico
** arduino-pico is Copyright (c) 2023 Earle F. Philhower, III <earlephilhower@yahoo.com>
*/
#include <Arduino.h>
#include "../../Boards.h"
#if HAS_BLUETOOTH == true && PLATFORM == PLATFORM_RP2XXX
#include "BluetoothSerial.h"
#include "CoreMutex.h"
#include "LocklessQueue.h"
#include "bluetooth.h"
#include <btstack.h>
#include "btstack_defines.h"
#include "btstack_event.h"
#include "classic/rfcomm.h"
#include "gap.h"
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <BluetoothLock.h>
#include <SerialUSB.h>

#ifdef DEBUG
#define TRACELOG(...) Serial.printf(__VA_ARGS__)
#else
#define TRACELOG(...) 
#endif

BluetoothSerial::BluetoothSerial() {
    mutex_init(&_mtx);
}

bool BluetoothSerial::setFIFOSize(size_t size) {
    if (!size || _running) {
        return false;
    }
    _fifoSize = size + 1;
    return true;
}

bool BluetoothSerial::setName(const char* name) {
    if(_running)
        return false;

    free(_name);
    _name = strdup(name);
    return true;
}

bool BluetoothSerial::applyBondable() {
    BluetoothLock l;
    if (!_running)
        return false;
    // if (_isBondable) {
    //     gap_set_bondable_mode(1);
    // } else {
    //     gap_set_bondable_mode(0);
    // }
    return true;
}

bool BluetoothSerial::setBondable(bool isBondable) {
    _isBondable = isBondable;
    return applyBondable();
}

BluetoothSerial::operator bool() {
    return _running;
}

void BluetoothSerial::disconnect() {
    if (_running && _connHandle != HCI_CON_HANDLE_INVALID) {
        gap_disconnect(_connHandle);
    }
}

void BluetoothSerial::end() {
    BluetoothLock l;
    if (!_running) {
        return;
    }
    setBondable(false);
    _running = false;
    hci_power_control(HCI_POWER_OFF);
    delete _queue;
}

int BluetoothSerial::availableForWrite(void) {
    CoreMutex cmtx(&_mtx);
    if (!_running || !cmtx)
        return 0;
    return _connected ? 1 : 0;
}

int BluetoothSerial::available(void) {
    CoreMutex cmtx(&_mtx);
    if (!_running || !cmtx)
        return -1;
    return _queue->available();
}

int BluetoothSerial::peek(void) {
    CoreMutex cmtx(&_mtx);
    uint8_t res;
    if (!_running || !cmtx)
        return -1;
    if (_queue->peek(&res))
        return res;
    return -1;
}

int BluetoothSerial::read(void) {
    CoreMutex cmtx(&_mtx);
    uint8_t res;
    if (!_running || !cmtx)
        return -1;
    if (_queue->read(&res))
        return res;
    return -1;
}

void BluetoothSerial::flush(void) {
    BluetoothLock l;
    TRACELOG("Requesting SEND NOW");
    rfcomm_request_can_send_now_event(_rfcommChannelID);
    while (_connected && _txLen) {
        delay(10);
    }
}

bool BluetoothSerial::overflow(void) {
    BluetoothLock l;
    if (!_running)
        return false;

    bool overflow = _overflow;
    _overflow = false;

    return overflow;
}

size_t BluetoothSerial::write(uint8_t chr) {
    return write(&chr, 1);
}

size_t BluetoothSerial::write(const uint8_t *buffer, size_t size) {
    CoreMutex cmtx(&_mtx);
    if (!_running || !cmtx || !size || !_connected)
        return 0;
    if (_txLen + size > 1024) {
        flush();
    }
    TRACELOG("TX Written: ");
    for (int i = 0; i < size; i++) {TRACELOG("%02X", buffer[i]);}
    TRACELOG("\r\n");
    memcpy((uint8_t*)_txBuf + _txLen, buffer, size);
    _txLen += size;

    return size;
};

void BluetoothSerial::begin(unsigned long baudrate, uint16_t config) {
    if (_running)
        end();

    _queue = new LocklessQueue<uint8_t>(_fifoSize);

    _overflow = false;
    _connected = false;

    BluetoothLock l;
    _hci_event_callback_registration.callback = PACKETHANDLERCB(BluetoothSerial, packetHandler);
    hci_add_event_handler(&_hci_event_callback_registration);

    l2cap_init();

#ifdef ENABLE_BLE
    sm_init();
#endif

    gap_ssp_set_enable(1);
    gap_discoverable_control(1);
    gap_ssp_set_auto_accept(0);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_set_security_level(LEVEL_2);
    gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_GENERAL_BONDING);

    rfcomm_init();
    rfcomm_register_service(PACKETHANDLERCB(BluetoothSerial, packetHandler), RFCOMM_SERVER_CHANNEL, 0xffff);

    sdp_init();
    memset(_spp_service, 0, sizeof(_spp_service));
    spp_create_sdp_record(_spp_service, sdp_create_service_record_handle(), RFCOMM_SERVER_CHANNEL, "RNode SPP");
    sdp_register_service(_spp_service);

    if (!_name) {
        setName("RNode FIXME"); // Unnamed rnode? that's a bug
    }

    gap_set_local_name(_name);

    hci_power_control(HCI_POWER_ON);
    // applyBondable();
    // Don't touch bondable, instead we deny pairing in packet handler.
    gap_set_bondable_mode(1);

    _running = true;
}

void BluetoothSerial::packetHandler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size) {
    bd_addr_t event_addr;
    
    TRACELOG("PACKET: type=%0X\r\n", type);

    switch (type) {
        case HCI_EVENT_PACKET: {
            TRACELOG("HCI EVENT: %0X\r\n", hci_event_packet_get_type(packet));
            switch (hci_event_packet_get_type(packet)) {
                case BTSTACK_EVENT_STATE: {
                    switch (btstack_event_state_get_state(packet)) {
                        case HCI_STATE_WORKING: {
                            gap_local_bd_addr(_local_addr);
                            TRACELOG("Got addr: %x:%x:%x:%x:%x:%x\r\n", _local_addr[0], _local_addr[1], _local_addr[2], _local_addr[3], _local_addr[4], _local_addr[5]);
                        }
                    }
                }
                case HCI_EVENT_PIN_CODE_REQUEST: {
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    // TRACELOG("Attempted a legacy pair!\r\n");
                    gap_pin_code_negative(event_addr);
                    break;
                }
                case HCI_EVENT_USER_CONFIRMATION_REQUEST: {
                    if (!_isBondable) {
                        gap_ssp_confirmation_negative(event_addr);
                        break;
                    }
                    hci_event_user_confirmation_request_get_bd_addr(packet, event_addr);
                    uint32_t num = hci_event_user_confirmation_request_get_numeric_value(packet);
                    if (_bt_confirm_pairing != nullptr) {
                        _bt_confirm_pairing(num);
                    }
                    TRACELOG("SSP REQ with num: %d\r\n", num);
                    gap_ssp_confirmation_response(event_addr);
                    break;
                }
                case HCI_EVENT_SIMPLE_PAIRING_COMPLETE: {
                    hci_event_simple_pairing_complete_get_bd_addr(packet, event_addr);
                    uint8_t status_code = hci_event_simple_pairing_complete_get_status(packet);
                    bool success = false;
                    if (status_code == 0) {
                        success = true;
                    }
                    if (_bt_pairing_complete != nullptr) {
                        _bt_pairing_complete(success);
                    }
                    break;
                }
                case RFCOMM_EVENT_INCOMING_CONNECTION: {
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    _rfcommChannelID = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    // TRACELOG("New RFCOMM Channel REQ: %i\r\n", _rfcommChannelID);
                    rfcomm_accept_connection(_rfcommChannelID);
                    break;
                }
                case RFCOMM_EVENT_CHANNEL_OPENED: {
                    if(rfcomm_event_channel_opened_get_status(packet)) {
                        TRACELOG("RFCOMM Channel failed to open: %02X\r\n", rfcomm_event_channel_opened_get_status(packet));
                    } else {
                        _rfcommChannelID = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        _connHandle = rfcomm_event_channel_opened_get_con_handle(packet);
                        TRACELOG("RFCOMM Channel %i opened!\r\n", _rfcommChannelID);
                        _connected = true;
                        if (_bt_connection_callback != nullptr)
                            _bt_connection_callback(_connected);
                    }
                    break;
                }
                case RFCOMM_EVENT_CAN_SEND_NOW: {
                    TRACELOG("CAN SEND NOW");
                    rfcomm_send(_rfcommChannelID, (uint8_t*)_txBuf, _txLen);
                    _txLen = 0;
                    lastFlushTime = millis();
                    break;
                }
                case RFCOMM_EVENT_CHANNEL_CLOSED: {
                    TRACELOG("RFCOMM Channel %i has closed!\r\n", _rfcommChannelID);
                    _connected = false;
                    if (_bt_connection_callback != nullptr)
                        _bt_connection_callback(_connected);
                    _rfcommChannelID = 0;
                    _connHandle = HCI_CON_HANDLE_INVALID;
                    break;
                }
            }
            break;
        }
        case RFCOMM_DATA_PACKET: {
            for (int i = 0; i < size; i++) {
                if (!_queue->write(packet[i]))
                    _overflow = true;
            }
        }
    }
}
#endif