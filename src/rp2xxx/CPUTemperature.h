/*
 ** RP2XXX chip-agnostic temperature measuring routines
 ** Copyright (c) 2026, Sergey Morozyuk <me@sergds.xyz>
*/

#pragma once
#include "hardware/address_mapped.h"
#include <hardware/adc.h>
#include <hardware/regs/sysinfo.h>
#include <stdint.h>
#include <Arduino.h>

#define TEMP_MIN -30.f
#define TEMP_MAX 90.f
#define TEMP_OFFSET 120.f

// TODO: Move somehere else?
uint32_t last_temp_indication_ms = 0;
uint32_t temp_indication_interval_ms = 2000;

uint8_t adc_temp_input = 0;
float last_temp = TEMP_MIN-1;
bool temp_measurement_ready = false;
uint32_t last_temperature_measurement_update = 0;
uint32_t temp_measurement_update_interval = 1000;

const float conversion_factor = 3.3f / (1 << 12);

void temp_measurement_init() {
    adc_init();
    adc_set_temp_sensor_enabled(true);
    uint32_t chipid = *((io_ro_32*)(SYSINFO_BASE + SYSINFO_CHIP_ID_OFFSET));
    uint32_t part = (chipid & SYSINFO_CHIP_ID_PART_BITS) >> SYSINFO_CHIP_ID_PART_LSB;
    switch (part) {
        case 0x0002: // RP2040
        {
            adc_temp_input = 4;
            break;
        }
        case 0x0004: // RP235XA/B
        {
            uint32_t package = *((io_ro_32*)(SYSINFO_BASE + 0x04));
            if (package) { adc_temp_input = 4; } else { adc_temp_input = 8; }
            break;
        }
        default:
        {
            return;
        }
    }
    temp_measurement_ready = true;
    return;
}

void temp_measurement_update() {
    if (!temp_measurement_ready) return;
    if (millis()-last_temperature_measurement_update >= temp_measurement_update_interval) {
        adc_select_input(adc_temp_input);
        float voltage = (float)adc_read() * conversion_factor;
        last_temp = 27.0f - (voltage - 0.706f) / 0.001721f;
        last_temperature_measurement_update = millis();
    }
}
