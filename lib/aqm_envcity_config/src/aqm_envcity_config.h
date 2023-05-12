#ifndef AQM_ENVCITY_CONFIG_H
#define AQM_ENVCITY_CONFIG_H

#include <stdint.h>

#define AQM_ENVCITY_CONFIG_VERSION_MAJOR 0
#define AQM_ENVCITY_CONFIG_VERSION_MINOR 1
// kkkk copilot

#define HM3305
// Como fazer pelo buid flags do platformio?

// Moving Average Filter

#define FILTER_COEF 8

typedef enum {
  CO_WE_PIN = 0, CO_AE_PIN, 
  OX_WE_PIN, OX_AE_PIN, 
  NO2_WE_PIN, NO2_AE_PIN,
  SO2_WE_PIN, SO2_AE_PIN, 
  ANEM_PIN = 12,
  TOTAL_ANALOG_PINS
}SensorAnalogPins;

typedef struct __attribute__((packed)) _sensors_readings{
  float co_ppb[4]; //0,1,2,3
  float no2_ppb[4];//8,9,10,11
  float so2_ppb[4];//12,13,14,15
  float ox_ppb[4]; //16
  float anem; // 24
  int16_t temp; // 1
  int16_t humidity; // 
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
} SensorsReadings;

typedef struct __attribute__((packed)) _sensor_voltage{
  
  float co_we;
  float co_ae;
  float no2_we;
  float no2_ae;
  float so2_we;
  float so2_ae;
  float ox_we;
  float ox_ae;
  float anem;

  int16_t temp; // 1
  int16_t humidity; // 

  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;

} SensorVoltage;

#endif // AQM_ENVCITY_CONFIG_H