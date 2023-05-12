
#include "hm3301_lib.hpp"

HM330X sensor;

uint8_t buf[30];

const char *str[] = {"sensor num: ", 
                     "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
};

HM330XErrorCode print_result(const char *str, uint16_t value) {
    if (NULL == str) {
        return ERROR_PARAM;
    }
    Serial.print(str);
    Serial.println(value);
    return NO_ERROR;
}

/*parse buf with 29 uint8_t-data*/
HM330XErrorCode parse_result(uint8_t *data, HM330X_RES *hm3301) {

    uint16_t value = 0;

    if (NULL == data || hm3301 == NULL)
        return ERROR_PARAM;

    for (int i = 1; i < 8; i++) {
        value = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
        print_result(str[i - 1], value);
        
        uint16_t *ptr = (uint16_t *)hm3301;
        *(ptr + i) = value;

    }

    return NO_ERROR;
}

HM330XErrorCode parse_result_value(uint8_t *data) {

    if (NULL == data) {
        return ERROR_PARAM;
    }

    for (int i = 0; i < 28; i++) {
        Serial.print(data[i], HEX);
        Serial.print("  ");
        if ((0 == (i) % 5) || (0 == i)) {
            Serial.println("");
        }
    }
    uint8_t sum = 0;
    for (int i = 0; i < 28; i++) {
        sum += data[i];
    }
    if (sum != data[28]) {
        Serial.println("wrong checkSum!!!!");
    }
    Serial.println("");
    return NO_ERROR;
}
