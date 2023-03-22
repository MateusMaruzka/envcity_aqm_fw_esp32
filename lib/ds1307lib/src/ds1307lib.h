#ifndef DS1307LIB_H
#define DS1307LIB_H

#include "esp_err.h"
#include "esp32-hal-i2c.h"

#define TIMEOUT 50

typedef struct {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t weekDay;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} ds1307_date_t;

typedef struct {
    uint8_t i2c_num;
    uint8_t sda;
    uint8_t scl;
    uint32_t freq;
    uint8_t addr;
} ds1307_config_t;


esp_err_t ds1307_init(ds1307_config_t config);

esp_err_t ds1307_init_default();

esp_err_t ds1307_write_date(ds1307_date_t date);

esp_err_t ds1307_read_date(ds1307_date_t *date);


#endif // DS1307LIB_H