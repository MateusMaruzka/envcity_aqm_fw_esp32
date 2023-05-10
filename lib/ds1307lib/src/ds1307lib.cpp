#include "ds1307lib.h"

ds1307_config_t _config = {
    .i2c_num = 0,
    .sda = 21,
    .scl = 22,
    .freq = 100000,
    .addr = 0x68
};

inline static uint8_t bcdToInt(uint8_t bcd){
    return ((bcd & 0b11110000) >> 4) * 10 + (bcd & 0b00001111);
}

inline static uint8_t intToBcd(uint8_t i){
    return ((i / 10) << 4) + (i % 10);
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
        
        uint8_t buff[8] = {0x00, date.second, date.minute, date.hour, 
                                date.weekDay,date.day, date.month, date.year};

        for(int i = 0; i < 8; i++){
            buff[i] = intToBcd(buff[i]);
        }

        err = i2cWrite(_config.i2c_num, _config.addr, buff, sizeof(buff), 50);

    }

    return err;
}

esp_err_t ds1307_read_date(ds1307_date_t *date){

    esp_err_t err = ESP_FAIL;
    size_t readCount;
    
    uint8_t buff[7] = {0};

    if(i2cIsInit(_config.i2c_num)){
        
        i2cWrite(0, 0x68, buff, 1, 50);

        err = i2cRead(_config.i2c_num, _config.addr, buff, sizeof(buff), TIMEOUT, &readCount);

        for(int i = 0; i < 7; i++){
            printf("%d ", buff[i]);
        }
        printf("\n");
    
    }

    date->second = bcdToInt(buff[0]);
    printf("sec: %d\n", date->second);
    date->minute = bcdToInt(buff[1]);
    date->hour = bcdToInt(buff[2]);
    date->weekDay = buff[3];
    date->day = bcdToInt(buff[4]);
    date->month = bcdToInt(buff[5]);
    date->year  = bcdToInt(buff[6]);

    return err;
}
