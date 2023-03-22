#include "ds1307lib.h"

ds1307_config_t _config = {
    .i2c_num = 0,
    .sda = 21,
    .scl = 22,
    .freq = 100000,
    .addr = 0x68
};

inline static uint8_t bcdToInt(uint8_t bcd){
    return (bcd && 0b11110000) * 10 + (bcd && 0b00001111);
}

esp_err_t ds1307_init(ds1307_config_t config){
    _config = config;
    esp_err_t err = i2cInit(config.i2c_num, config.sda, config.scl, config.freq);
    return err;
}

esp_err_t ds1307_init_default(){
    esp_err_t err = i2cInit(_config.i2c_num, _config.sda, _config.scl, _config.freq);
    return err;
}

//esp_err_t i2cWrite(uint8_t i2c_num, uint16_t address, const uint8_t* buff, size_t size, uint32_t timeOutMillis){
esp_err_t ds1307_write_date(ds1307_date_t date){

    esp_err_t err = ESP_FAIL;

    if(i2cIsInit(_config.i2c_num)){
        
        const uint8_t buff[8] = {0x00, date.second, date.minute, date.hour, 
                          date.day, date.date, date.month, date.year};

        err = i2cWrite(_config.i2c_num, _config.addr, buff, sizeof(buff), 50);

    }

    return err;
}

esp_err_t ds1307_read_date(ds1307_date_t *date){

    esp_err_t err = ESP_FAIL;
    size_t readCount;
    
    uint8_t buff[7];

    if(i2cIsInit(_config.i2c_num)){
        
        err = i2cRead(_config.i2c_num, _config.addr, buff, sizeof(buff), TIMEOUT, &readCount);
    
    }

    date->second = bcdToInt(buff[0]);
    date->minute = bcdToInt(buff[1]);
    date->hour = bcdToInt(buff[2]);
    date->weekDay = buff[3];
    date->day = bcdToInt(buff[3]);
    date->month = bcdToInt(buff[4]);
    date->year  = bcdToInt(buff[5]);

    return err;
}
