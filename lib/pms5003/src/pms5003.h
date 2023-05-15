#ifndef PMS5003_LIB
#define PMS5003_LIB

#include <driver/gpio.h>
#include <driver/rmt.h>
#include <esp_err.h>
#include <esp_log.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "driver/uart.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE 512
#define START_CHAR1 0x42    
#define START_CHAR2 0x4D

typedef struct __attribute__((packed)) data {

    uint16_t packet_len;
    uint16_t pm1_0_std;
    uint16_t pm2_5_std;
    uint16_t pm10_std;
    uint16_t pm1_0_atm;
    uint16_t pm2_5_atm;
    uint16_t pm10_atm; 
    uint16_t particles_0_3um;
    uint16_t particles_0_5um;
    uint16_t particles_1_0um;
    uint16_t particles_2_5um;
    uint16_t particles_5_0um;
    uint16_t particles_10um;


}pms_sensor_data_t;

void print_pms_sensor_data(const pms_sensor_data_t *data);

void pms5003_init();

esp_err_t pms5003_read(pms_sensor_data_t *data_sensor);

#endif