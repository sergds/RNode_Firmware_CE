/* A btstack serial implementation for RNode, with SSP and callbacks. (BLE, Nordic UART Service)
** Copyright (c) 2026, Sergey Morozyuk <me@sergds.xyz>
** Loosely based on btstack examples and SerialBT from arduino-pico
** arduino-pico is Copyright (c) 2023 Earle F. Philhower, III <earlephilhower@yahoo.com>
*/
#include <Arduino.h>
#include "../../Boards.h"
#if HAS_BLE == true
#include "btstack_util.h"
#include "ble/sm.h"
#include "BluetoothSerialNUS.h"
#include "CoreMutex.h"
#include "LocklessQueue.h"
#include "ble/att_db.h"
#include "ble/att_server.h"
#include "ble/gatt_client.h"
#include "bluetooth.h"
#include <btstack.h>
#include "bluetooth_data_types.h"
#include "btstack_defines.h"
#include "btstack_event.h"
#include "btstack_undefs.h"
#include "ble/gatt-service/nordic_spp_service_server.h"
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <BluetoothLock.h>
#include <SerialUSB.h>
#include "RNodeBluetoothSerial.h"
#include "gap.h"

#define ENSURE_AD_ELEMENT(appendFcn, type) if (!appendFcn) {Serial.printf("Failed to append AD element type=%02x", type);}

#ifdef DEBUG
#define TRACELOG(...) Serial.printf(__VA_ARGS__)
#else
#define TRACELOG(...) 
#endif

BluetoothSerialNUS::BluetoothSerialNUS() {
    mutex_init(&_mtx);
}

bool BluetoothSerialNUS::setFIFOSize(size_t size) {
    if (!size || _running) {
        return false;
    }
    _fifoSize = size + 1;
    return true;
}

void BluetoothSerialNUS::disconnect() {
    if(_running && _conHandle != HCI_CON_HANDLE_INVALID) {
        gap_disconnect(_conHandle);
    }
}

void BluetoothSerialNUS::clearAdvertisingData(uint8_t slot) {
    if (slot + 1 > BLE_ADV_SLOTS) {
        return;
    }
    memset(_adv_data + slot * 31, 0, 31);
    _adv_data_len[slot] = 0;
}

// len includes type byte as well! (len = 1 + sizeof(data))
bool BluetoothSerialNUS::appendAdvertisingDataElement(uint8_t slot, uint8_t len, uint8_t type, uint8_t* data) {
    if (slot + 1 > BLE_ADV_SLOTS) {
        return false;
    }
    if (_adv_data_len[slot] + 3 >= sizeof(_adv_data)) {
        return false;
    }
    uint8_t entry[len + 1];
    entry[0] = len;
    entry[1] = type;
    memcpy(entry + 2, data, len - 1); // Now copy payload of the entry.
    memcpy(_adv_data + slot * 31 + _adv_data_len[slot], entry, sizeof(entry));
    _adv_data_len[slot] += sizeof(entry);
    return true;
}

void BluetoothSerialNUS::setupAdvertising() {
    gap_advertisements_set_data(_adv_data_len[0], _adv_data);
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(0x0030, 0x0100, 0, 0, null_addr, 0x07, 0x00);
    if (_adv_data_len[1] > 0)
        gap_scan_response_set_data(_adv_data_len[1], _adv_data + 31);
    gap_advertisements_enable(1);
}

void BluetoothSerialNUS::rebuildAdvertisingData() {
    uint8_t flags = 0x06;
    uint8_t nus_uuid[] = {0x9e, 0xca, 0xdc, 0x24, 0xe, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x1, 0x0, 0x40, 0x6e};
    clearAdvertisingData(0); // Slot 0: Primary Legacy Advertising Data
    clearAdvertisingData(1); // Slot 1: Scan response data
    // Advertising data
    ENSURE_AD_ELEMENT(appendAdvertisingDataElement(0, 2, BLUETOOTH_DATA_TYPE_FLAGS, &flags), BLUETOOTH_DATA_TYPE_FLAGS); // General discoverable, BR/EDR not supported
    ENSURE_AD_ELEMENT(appendAdvertisingDataElement(0, 17, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, nus_uuid), BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS); // Service UUID
    // Scan response
    ENSURE_AD_ELEMENT(appendAdvertisingDataElement(1, 1 + strlen(_name), BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, (uint8_t*)_name), BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME); // Name
}

bool BluetoothSerialNUS::setName(const char* name) {
    if(_running)
        return false;
    if(strlen(name) > 29) {
        return false; // 29 Bytes max in scan response
    }
    BluetoothLock l;
    if(_running)
        gap_advertisements_enable(0);
    free(_name);
    _name = strdup(name);
    rebuildAdvertisingData();
    if (_running) {
        setupAdvertising();
    }
    return true;
}

bool BluetoothSerialNUS::applyBondable() {
    BluetoothLock l;
    if (!_running)
        return false;
    if (_isBondable) {
        gap_set_bondable_mode(1);
    } else {
        gap_set_bondable_mode(0);
    }
    return true;
}

bool BluetoothSerialNUS::setBondable(bool isBondable) {
    // if (!_running) {
    //     return false;
    // }
    _isBondable = isBondable;
    return applyBondable();
}

BluetoothSerialNUS::operator bool() {
    return _running;
}

void BluetoothSerialNUS::end() {
    BluetoothLock l;
    if (!_running) {
        return;
    }
    setBondable(false);
    _running = false;
    hci_power_control(HCI_POWER_OFF);
    delete _queue;
}

int BluetoothSerialNUS::availableForWrite(void) {
    CoreMutex cmtx(&_mtx);
    if (!_running || !cmtx)
        return 0;
    return _connected ? 1 : 0;
}

int BluetoothSerialNUS::available(void) {
    CoreMutex cmtx(&_mtx);
    if (!_running || !cmtx)
        return -1;
    return _queue->available();
}

int BluetoothSerialNUS::peek(void) {
    CoreMutex cmtx(&_mtx);
    uint8_t res;
    if (!_running || !cmtx)
        return -1;
    if (_queue->peek(&res))
        return res;
    return -1;
}

int BluetoothSerialNUS::read(void) {
    CoreMutex cmtx(&_mtx);
    uint8_t res;
    if (!_running || !cmtx)
        return -1;
    if (_queue->read(&res))
        return res;
    return -1;
}

void BluetoothSerialNUS::flush(void) {
    BluetoothLock l;
    TRACELOG("Requesting SEND NOW\r\n");
    //delay(15);
    nordic_spp_service_server_request_can_send_now(&_sendRequest, _conHandle);
    while (_connected && _txLen) {
        delay(10);
    }
}

bool BluetoothSerialNUS::overflow(void) {
    BluetoothLock l;
    if (!_running)
        return false;

    bool overflow = _overflow;
    _overflow = false;

    return overflow;
}

size_t BluetoothSerialNUS::write(uint8_t chr) {
    return write(&chr, 1);
}

void BluetoothSerialNUS::nordicCanSend(void* context) {
    TRACELOG("CAN SEND NOW\r\n");
    for (int i = 0; i < _txLen; i++) {TRACELOG("TX: %c\t%02x\r\n", ((const uint8_t*)_txBuf)[i], ((const uint8_t*)_txBuf)[i]);}
    nordic_spp_service_server_send(_conHandle, (const uint8_t*)_txBuf, _txLen);
    _txLen = 0;
    lastFlushTime = millis();
}

size_t BluetoothSerialNUS::write(const uint8_t *buffer, size_t size) {
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
    // _txBuf = buffer;
    _txLen += size;
    // flush();
    return size;
};

void BluetoothSerialNUS::begin(unsigned long baudrate, uint16_t config) {
    if (_running)
        end();

    _queue = new LocklessQueue<uint8_t>(_fifoSize);

    _overflow = false;
    _connected = false;

    BluetoothLock l;
    _hci_event_callback_registration.callback = PACKETHANDLERCB(BluetoothSerialNUS, packetHandler);
    hci_add_event_handler(&_hci_event_callback_registration);

    l2cap_init();

    _sm_event_callback_registration.callback = PACKETHANDLERCB(BluetoothSerialNUS, smPacketHandler);
    sm_init();
    sm_add_event_handler(&_sm_event_callback_registration);
    gatt_client_set_required_security_level(LEVEL_3);
    sm_set_secure_connections_only_mode(true);
    sm_set_io_capabilities(IO_CAPABILITY_DISPLAY_YES_NO);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION|SM_AUTHREQ_MITM_PROTECTION|SM_AUTHREQ_BONDING);

    att_server_init(profile_data, ATTREADHANDLERCB(BluetoothSerialNUS, attReadHandler), NULL);
    gatt_client_init();

    _sendRequest.callback = CONTEXTCB(BluetoothSerialNUS, nordicCanSend);

    nordic_spp_service_server_init(PACKETHANDLERCB(BluetoothSerialNUS, packetHandler));

    if (!_name) {
        setName("RNode FIXME"); // Unnamed rnode? that's a bug
    }

    setupAdvertising();

    hci_power_control(HCI_POWER_ON);
    applyBondable();

    _running = true;
}

void BluetoothSerialNUS::packetHandler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size) {
    bd_addr_t event_addr;
    
    TRACELOG("PACKET: type=%02X\r\n", type);

    switch (type) {
        case HCI_EVENT_PACKET: {
            TRACELOG("HCI EVENT: %0X\r\n", hci_event_packet_get_type(packet));
            switch (hci_event_packet_get_type(packet)) {
                case HCI_EVENT_GATTSERVICE_META: {
                    switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
                        case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED: {
                            _conHandle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
                            _connected = true;
                            TRACELOG("New NUS Connection: handle=%04x\r\n", _conHandle);
                            if (_bt_connection_callback != nullptr) {
                                _bt_connection_callback(true);
                            }
                            break;
                        }
                        case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED: {
                            TRACELOG("Lost NUS Connection: handle=%04x\r\n", _conHandle);
                            _conHandle = HCI_CON_HANDLE_INVALID;
                            _connected = false;
                            if (_bt_connection_callback != nullptr) {
                                _bt_connection_callback(false);
                            }
                            break;
                        }
                    }
                    break;
                }
                default:
                    break;
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

void BluetoothSerialNUS::smPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) return;
    TRACELOG("SM PACKET: type=%02X; hci_type=%02X\r\n", packet_type, hci_event_packet_get_type(packet));
    hci_con_handle_t con_handle;
    bd_addr_t address;
    bd_addr_type_t addr_type;
    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_META_GAP: {
            switch (hci_event_gap_meta_get_subevent_code(packet)) {
                case GAP_SUBEVENT_LE_CONNECTION_COMPLETE: {
                    TRACELOG("\r\n\r\n\r\nConnection complete\r\n\r\n\r\n");
                    break;
                }
                default:
                    break;
            }
        break;
        }
        case SM_EVENT_JUST_WORKS_REQUEST: {
            sm_bonding_decline(sm_event_just_works_request_get_handle(packet));
            break;
        }
        case SM_EVENT_NUMERIC_COMPARISON_REQUEST: {
            if (!_isBondable) {
                sm_bonding_decline(sm_event_numeric_comparison_request_get_handle(packet));
                gap_disconnect(sm_event_numeric_comparison_request_get_handle(packet));
                break;
            }
            uint32_t passkey = sm_event_numeric_comparison_request_get_passkey(packet);
            TRACELOG("SM Numeric comparison: num=%06i\r\n", passkey);
            sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet));
            if (_bt_confirm_pairing != nullptr) {
                _bt_confirm_pairing(passkey);
            }
            break;
        }
        case SM_EVENT_PASSKEY_DISPLAY_NUMBER: {
            if (!_isBondable) {
                con_handle = sm_event_passkey_display_number_get_handle(packet);
                sm_bonding_decline(con_handle);
                gap_disconnect(con_handle);
                break;
            }
            uint32_t passkey = sm_event_passkey_display_number_get_passkey(packet);
            TRACELOG("Display Passkey: %06u\r\n", passkey);
            if (_bt_confirm_pairing != nullptr) {
                _bt_confirm_pairing(passkey);
            }
            break;
        }
        case SM_EVENT_IDENTITY_CREATED: {
            sm_event_identity_created_get_identity_address(packet, address);
            TRACELOG("Identity created: type %u address %s\r\n", sm_event_identity_created_get_identity_addr_type(packet), bd_addr_to_str(address));
            break;
        }
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED: {
            sm_event_identity_resolving_succeeded_get_identity_address(packet, address);
            TRACELOG("Identity resolved: type %u address %s\r\n", sm_event_identity_resolving_succeeded_get_identity_addr_type(packet), bd_addr_to_str(address));
            break;
        }
        case SM_EVENT_IDENTITY_RESOLVING_FAILED: {
            sm_event_identity_created_get_address(packet, address);
            TRACELOG("Identity resolving failed: addr=%s\r\n", bd_addr_to_str(address));
            break;
        }
        case SM_EVENT_PAIRING_STARTED: {
            if (!_isBondable) {
                gap_disconnect(sm_event_pairing_started_get_handle(packet));
                break;
            }
            TRACELOG("Pairing started\r\n");
            break;
        }
        case SM_EVENT_PAIRING_COMPLETE: {
            switch (sm_event_pairing_complete_get_status(packet)) {
                case ERROR_CODE_SUCCESS: {
                    TRACELOG("Pairing complete, success.\r\n");
                    if (_bt_pairing_complete != nullptr) {
                        _bt_pairing_complete(true);
                    }
                    break;
                }
                case ERROR_CODE_CONNECTION_TIMEOUT: {
                    TRACELOG("Pairing failed, timeout\r\n");
                    if (_bt_pairing_complete != nullptr) {
                        _bt_pairing_complete(false);
                    }
                    break;
                }
                case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION: {
                    TRACELOG("Pairing failed, disconnected\r\n");
                    if (_bt_pairing_complete != nullptr) {
                        _bt_pairing_complete(false);
                    }
                    break;
                }
                case ERROR_CODE_AUTHENTICATION_FAILURE: {
                    TRACELOG("Pairing failed, authentication failure with reason = %u\r\n", sm_event_pairing_complete_get_reason(packet));
                    if (_bt_pairing_complete != nullptr) {
                        _bt_pairing_complete(false);
                    }
                    break;
                }
                default:
                    break;
            }
        }
        case SM_EVENT_REENCRYPTION_STARTED: {
            sm_event_reencryption_started_get_address(packet, address);
            TRACELOG("Bonding information exists for addr type %u, identity addr %s -> re-encryption started\r\n",
            sm_event_reencryption_started_get_addr_type(packet), bd_addr_to_str(address));
            break;
        }
        case SM_EVENT_REENCRYPTION_COMPLETE: {
            switch (sm_event_reencryption_complete_get_status(packet)){
                case ERROR_CODE_SUCCESS: {
                    TRACELOG("Re-encryption complete, success\r\n");
                    break;
                }
                case ERROR_CODE_CONNECTION_TIMEOUT: {
                    TRACELOG("Re-encryption failed, timeout\r\n");
                    break;
                }
                case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION: {
                    TRACELOG("Re-encryption failed, disconnected\r\n");
                    break;
                }
                case ERROR_CODE_PIN_OR_KEY_MISSING: {
                    TRACELOG("Re-encryption failed, bonding information missing\r\n");
                    TRACELOG("Assuming remote lost bonding information\r\n");
                    TRACELOG("Deleting local bonding information to allow for new pairing..\r\n");
                    sm_event_reencryption_complete_get_address(packet, address);
                    addr_type = (bd_addr_type_t)sm_event_reencryption_started_get_addr_type(packet);
                    gap_delete_bonding(addr_type, address);
                    break;
                }
                default:
                    break;
            }
        }
        case GATT_EVENT_QUERY_COMPLETE: {
            uint8_t status = gatt_event_query_complete_get_att_status(packet);
            switch (status) {
                case ATT_ERROR_INSUFFICIENT_ENCRYPTION: {
                    TRACELOG("GATT Query failed, Insufficient Encryption\n");
                    break;
                }
                case ATT_ERROR_INSUFFICIENT_AUTHENTICATION: {
                    TRACELOG("GATT Query failed, Insufficient Authentication\n");
                    break;
                }
                case ATT_ERROR_BONDING_INFORMATION_MISSING: {
                    TRACELOG("GATT Query failed, Bonding Information Missing\n");
                    break;
                }
                case ATT_ERROR_SUCCESS: {
                    TRACELOG("GATT Query successful\n");
                    break;
                }
                default: {
                    TRACELOG("GATT Query failed, status 0x%02x\n", gatt_event_query_complete_get_att_status(packet));
                    break;
                }
            }
        }
    }
}

uint16_t BluetoothSerialNUS::attReadHandler(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    if (att_handle == ATT_CHARACTERISTIC_GAP_DEVICE_NAME_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob((const uint8_t *)_name, strlen(_name), offset, buffer, buffer_size);
    }
    return 0;
}
#endif