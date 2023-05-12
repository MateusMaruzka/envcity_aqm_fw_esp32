// Copyright (C) 2022 Lucas Mulling

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

static const char *TAG = "am2302";

#include "am2302.hpp"

#include <driver/gpio.h>
#include <driver/rmt.h>
#include <esp_err.h>
#include <esp_log.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static RingbufHandle_t ringbuf_handle = NULL;

void am2302_init(void){

    rmt_config_t rmt_rx_config = {
        .rmt_mode      = RMT_MODE_RX,
        .channel       = CONFIG_AM2302_RMT_CHANNEL,
        .gpio_num      = AM2302_GPIO_PIN,
        .clk_div       = 80,
        .mem_block_num = 4,  // mem_block is 512 * uint32_t
        .rx_config = {
            .idle_threshold      = 100, // NOTE: if the signal does not change for `idle_threshold` amount of time, ISR is triggered
                                        //       (not stated in the official documentation)
            .filter_ticks_thresh = 50, // NOTE: "Counted in source clock, not divided counter clock." ¯\_(ツ)_/¯
            .filter_en           = 1
        }
    };

    ESP_ERROR_CHECK(rmt_config(&rmt_rx_config));
    ESP_ERROR_CHECK(rmt_driver_install(CONFIG_AM2302_RMT_CHANNEL, 2048, 0));
    ESP_ERROR_CHECK(rmt_set_memory_owner(CONFIG_AM2302_RMT_CHANNEL, RMT_MEM_OWNER_RX));
    gpio_config_t gpio = {
        .pin_bit_mask = (uint64_t)AM2302_GPIO_PIN_SEL,
        .mode         = (gpio_mode_t)GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&gpio));
    ESP_ERROR_CHECK(rmt_get_ringbuf_handle(CONFIG_AM2302_RMT_CHANNEL, &ringbuf_handle));
}

static inline esp_err_t am2302_checksum(const int8_t *data, size_t len){
    int8_t sum = 0;
    for(int8_t i = 0; i < len - 1; i++){
        sum += data[i];
    }

    sum %= 256;
    return sum == data[len-1] ? ESP_OK : ESP_ERR_INVALID_CRC;
}

static inline int16_t convertTemperature(int8_t *data){
    int16_t res = (((uint16_t)(data[2] & 0b01111111)) << 8) | (data[3] & 0xFF);
    if(res & 0xb10000000){
        res *= -1;
    }
    return res;
}

static inline int16_t convertHumidity(int8_t *data){
    int16_t res = (((uint16_t)(data[0] & 0b01111111)) << 8) | (data[1] & 0xFF);
    return res;
}

esp_err_t am2302_check_checksum(const uint64_t bits){ 
    uint8_t sum = 0x00;

    for (uint8_t i = 1; i < 5; i++)
        sum += 0xFF & (bits >> (i << 3));

    return sum == (uint8_t)(bits & 0xFF) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

static inline esp_err_t am2302_parse(const rmt_item32_t *items, int8_t *data, size_t len){

    if(items == NULL || data == NULL){
        return ESP_ERR_INVALID_ARG;
    }
    if(len != 5){
        return ESP_ERR_INVALID_SIZE;
    }

    for(int  i = 0; i < 40; i++){

        if(items[i+2].duration0 > items[i + 2].duration1){
            data[i/8] |= (1 << (7 - (i % 8)));
        }else{
            data[i/8] &= ~(1 << (7 - (i % 8)));
        }
    }
    return ESP_OK;
}

/*
* Example:
* DHT22 sends out the higher data bit first!
* 
* 
*         MSB                                                                               LSB
*          | data_bytes[0]    data_bytes[1]   ...                                 checksum   |  
* stream:  0 0 0 0 0 0 1 0  1 0 0 0 1 1 0 0  0 0 0 0 0 0 0 1  0 1 0 1 1 1 1 1  1 1 1 0 1 1 1 0
*          ^                                                                                 ^                                                            
*          |                                                                                 |
*      First bit                                                                           Last bit
*/
esp_err_t am2302_parse2(const rmt_item32_t *items, int16_t *t, int16_t *h){

    uint64_t bits = 0x0;

    // NOTE: not checking for durations higher than the ones in the spec ¯\_(ツ)_/¯
    // also not checking for events where two signals are high/low in sequence, i.e:
    //     __ __    __
    // ___|     |__|
    //       ^
    //       |
    //       the RMT driver should take care of these cases, resulting in a checksum fail,
    //       also the length of the response *should* not match 42, resulting in an error
    //       prior to parsing

    for (ssize_t i = 39; i >= 0; i--){
        bits |= ((abs((uint16_t)items->duration0 - 70) <= 5) ? 1ULL : 0ULL) << i;
        items++;
    }

    ESP_LOGE(TAG, "bits: %lld", bits);
    esp_err_t checksum_result = am2302_check_checksum(bits);

    if (checksum_result != ESP_OK){
        ESP_LOGE(TAG, "checksum fail");
        //return checksum_result;
    }

    *h = (0xFFFF) & bits >> 24;
    *t = (0xFFFF) & bits >> 8;

    checksum_result = ESP_OK;
    return checksum_result;
}

esp_err_t am2302_read(int16_t *t, int16_t *h){

    esp_err_t err;

    size_t len_items = 0;

    gpio_set_level(AM2302_GPIO_PIN, 0);
    rmt_rx_start(CONFIG_AM2302_RMT_CHANNEL, true);
    ets_delay_us(1100);
    gpio_set_level(AM2302_GPIO_PIN, 1);
    // NOTE: wait for 4 ticks (4ms)
    rmt_item32_t *items = (rmt_item32_t *)xRingbufferReceive(ringbuf_handle, &len_items, portMAX_DELAY);
    rmt_rx_stop(CONFIG_AM2302_RMT_CHANNEL);

    //ESP_LOGE(TAG, "len_items = %d", len_items);

    len_items /= 4;

    if (len_items != 42){
        err = ESP_ERR_INVALID_SIZE;
        ESP_LOGE(TAG, "could not read sensor data (len_items = %d)", len_items);
        goto end;
    }

    if (items == NULL){
        err = ESP_ERR_INVALID_RESPONSE;
        ESP_LOGE(TAG, "could not read sensor");
        goto end;

    }else{

        int8_t data_bytes[5] = {0,0,0,0,0};

        err = am2302_parse(items, data_bytes, 5);
       
        /*for (int i = 0; i < 5; i++) {
            ESP_LOGE(TAG, "data_bytes[%d] = %ud ", i, data_bytes[i] & 0xFF);
        }*/

        // Verify checksum
        if(am2302_checksum(data_bytes, 5) == ESP_OK){
            // ESP_LOGE(TAG, "Checksum OK\n");
        } else {
            ESP_LOGE(TAG, "Checksum error");
            // return
        }

        *t = convertTemperature(data_bytes);
        *h = convertHumidity(data_bytes);

    }

end:

    if(items != NULL){
        vRingbufferReturnItem(ringbuf_handle, (void *)items);
    }

    //xRingbufferPrintInfo(ringbuf_handle);
    //rmt_rx_memory_reset(CONFIG_AM2302_RMT_CHANNEL);    
    
    return err;
}
