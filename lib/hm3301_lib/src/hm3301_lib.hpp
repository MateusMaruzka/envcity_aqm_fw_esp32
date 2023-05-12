#ifndef HM3301_LIB
#define HM3301_LIB

#include <Seeed_HM330X.h>

struct __attribute__((__packed__))  HM330X_RES {
    uint16_t sensor_num;
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10;
    uint16_t pm1_0_atmos;
    uint16_t pm2_5_atmos;
    uint16_t pm10_atmos;
};

HM330XErrorCode print_result(const char *str, uint16_t value);

HM330XErrorCode parse_result(uint8_t *data, HM330X_RES *hm3301);

HM330XErrorCode parse_result_value(uint8_t *data);


#endif /* HM3301_LIB */
