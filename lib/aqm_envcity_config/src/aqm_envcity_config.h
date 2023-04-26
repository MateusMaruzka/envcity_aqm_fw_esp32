#ifndef AQM_ENVCITY_CONFIG_H
#define AQM_ENVCITY_CONFIG_H

#include <stdint.h>

#define AQM_ENVCITY_CONFIG_VERSION_MAJOR 0
#define AQM_ENVCITY_CONFIG_VERSION_MINOR 1
// kkkk copilot

#define HM3305
// Como fazer pelo buid flags do platformio?

typedef enum {
  H2S_WE_PIN, H2S_AE_PIN, 
  NH3_WE_PIN, NH3_AE_PIN, 
  TOTAL_ANALOG_PINS
}SensorAnalogPins;

typedef struct __attribute__((packed)) _sensor_voltage{

  float h2s_we;
  float h2s_ae;
  float nh3_we;
  float nh3_ae;
  int16_t temp;
  int16_t umid;

} SensorVoltage;


typedef struct __attribute__((packed)) _sensors_readings{
  float h2s_ppb[4]; //0,1,2,3
  float nh3_ppb[4];//4, 5, 6, 7
  int16_t temp; // 1
  int16_t humidity; // 
  unsigned long long utc_time;

} SensorsReadings;

#endif // AQM_ENVCITY_CONFIG_H