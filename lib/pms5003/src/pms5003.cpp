#include "pms5003.h"

#define TAG "PMS5003"

static inline uint16_t convertBigToLittle(uint16_t value){

    uint16_t h = 0, l = 0;
    h = (value & 0xFF00) >> 8;
    l = value & 0x00FF;
    return h | (l << 8);

}


static inline void convert_pms_sensor_data(pms_sensor_data_t *data) {
    data->packet_len = convertBigToLittle(data->packet_len);
    data->pm1_0_std = convertBigToLittle(data->pm1_0_std);
    data->pm2_5_std = convertBigToLittle(data->pm2_5_std);
    data->pm10_std = convertBigToLittle(data->pm10_std);
    data->pm1_0_atm = convertBigToLittle(data->pm1_0_atm);
    data->pm2_5_atm = convertBigToLittle(data->pm2_5_atm);
    data->pm10_atm = convertBigToLittle(data->pm10_atm);
    data->particles_0_3um = convertBigToLittle(data->particles_0_3um);
    data->particles_0_5um = convertBigToLittle(data->particles_0_5um);
    data->particles_1_0um = convertBigToLittle(data->particles_1_0um);
    data->particles_2_5um = convertBigToLittle(data->particles_2_5um);
    data->particles_5_0um = convertBigToLittle(data->particles_5_0um);
    data->particles_10um = convertBigToLittle(data->particles_10um);
}


void print_pms_sensor_data(const pms_sensor_data_t *data) {

    ESP_LOGI(TAG, "packet_len: %u", data->packet_len);
    ESP_LOGI(TAG, "pm1_0_std: %u", data->pm1_0_std);
    ESP_LOGI(TAG, "pm2_5_std: %u", data->pm2_5_std);
    ESP_LOGI(TAG, "pm10_std: %u", data->pm10_std);
    ESP_LOGI(TAG, "pm1_0_atm: %u", data->pm1_0_atm);
    ESP_LOGI(TAG, "pm2_5_atm: %u", data->pm2_5_atm);
    ESP_LOGI(TAG, "pm10_atm: %u", data->pm10_atm);
    ESP_LOGI(TAG, "particles_0_3um: %u", data->particles_0_3um);
    ESP_LOGI(TAG, "particles_0_5um: %u", data->particles_0_5um);
    ESP_LOGI(TAG, "particles_1_0um: %u", data->particles_1_0um);
    ESP_LOGI(TAG, "particles_2_5um: %u", data->particles_2_5um);
    ESP_LOGI(TAG, "particles_5_0um: %u", data->particles_5_0um);
    ESP_LOGI(TAG, "particles_10um: %u", data->particles_10um);
}

void pms5003_init(){


    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, GPIO_NUM_4, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));



    gpio_config_t gpio = {
        .pin_bit_mask = (uint64_t)GPIO_NUM_25,
        .mode         = (gpio_mode_t)GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&gpio));



}

esp_err_t pms5003_read(pms_sensor_data_t *data_sensor){

    uint8_t data[BUF_SIZE];

    if(data_sensor == NULL){
        return ESP_ERR_INVALID_ARG;
    }

    int len = uart_read_bytes(UART_NUM, data, 256, portTICK_RATE_MS);
    ESP_LOGE("PMS5003", "Read %d bytes", len);

    if(len <= 0){
        return ESP_FAIL;
    }

    int idx = 0;
    for(idx = 0; idx < len; idx++){ // Find the initial bytes 0x42 and 0x4D
        if(data[idx] == 0x42 && data[idx+1] == 0x4d){
            break;
        }
    }

    if(idx + 32 > BUF_SIZE || idx >= len){
        return ESP_FAIL;
    }

    uint16_t checksum = 0;
    for(int i = idx; i < idx + 30; i++){
        checksum += data[i];
    }
    if(checksum != (data[idx + 30] << 8 | data[idx + 31])){
        return ESP_FAIL;
    }

    memcpy((void *)data_sensor, (void *)&data[idx + 2], sizeof(pms_sensor_data_t));

    convert_pms_sensor_data(data_sensor);

    return ESP_OK;

}


