#include <string>
#include <math.h>

#include <Adafruit_ADS1X15.h>
#include <Wire.h>

#include <Arduino.h>
#include <ArduinoJson.h>

#include <lmic.h>
#include "arduino_lmic_hal_configuration.h"
#include <hal/hal.h>
#include <time.h>
#include <sys/time.h>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <iomanip>

#include "Alphasense_GasSensors.hpp"
#include "anemometro_analog.hpp"
#include "envcity_lora_config.hpp"

//#include "DHT.h"

#define PRINT_ANALOG_READS 0
#define PRINT_MESSAGE 0

// COb4 -> 354
AlphasenseSensorParam param1 = {"CO-B4", COB4_n, 0.8, 330, 316, 510, 0.408, 336, 321, 0};
Alphasense_COB4 cob4_s1(param1);

AlphasenseSensorParam param2 = {"NH3-B1", COB4_n, 0.8, 775, 277, 59, 0.047, 277, 278, 0};
Alphasense_NH3 nh3(param2);

AlphasenseSensorParam param3 = {"H2S-B4", H2SB4_n, 0.8, 353,342,2020, 1.616, 345,344, 0};
Alphasense_H2S h2s(param3);

AlphasenseSensorParam param4 = {"0X", OXB431_n, -0.73, 229, 234, -506, 0.369, 237, 242, -587};
Alphasense_OX ox(param4);

AlphasenseSensorParam param5 = {"NO2", NO2B43F_n, -0.73, 222, 212, -424, 0.31, 230, 220, 0};
Alphasense_NO2 no2(param5);

AlphasenseSensorParam param6 = {"SO2", SO2B4_n, 0.8, 361, 350, 363, 0.29, 335, 343, 0};
Alphasense_SO2 so2(param6);

#define DHTPIN 13     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//DHT dht(DHTPIN, DHTTYPE);

#define ADS_SDA 21
#define ADS_SCL 22
Adafruit_ADS1115 ads;
enum {S0 = 17, S1 = 2, S2 = 4, S3 = 16}; // Pinos Multiplexador

//#include <oled/SSD1306Wire.h>
//SSD1306Wire *display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_128_64);


/**
 * @brief You must put them in order that the sensors are connected to the board(REV2)
 * Connector(analogInput1, analogInput2)
 * J6(0,1) J9(2,3) J7(4, 5) J10(6,7) J11(8,9) J8(10, 11) J12(12)
 * 
 */

/*

#define _stringfy(s) #s
#define stringfy_we(x) _stringfy(x##_WE_PIN)
#define stringfy_ae(x) _stringfy(x##_AE_PIN)

typedef enum {
  #define EXPAND_PINS(SENSOR)  SENSOR##_WE_PIN, SENSOR##_AE_PIN,
  #include "sensors_config_pins.h"
  #undef EXPAND_PINS
  ANEM_PIN,
  TOTAL_PINS
}SensorAnalogPins;

char *enum_str_full[] = {
    #define EXPAND_PINS(x) [x##_WE_PIN] = stringfy_we(x), stringfy_ae(x),
    #include "sensors_config_pins.h"
    #undef EXPAND_PINS  
};*/


typedef enum {
  CO_WE_PIN = 0, CO_AE_PIN, 
  NH3_WE_PIN, NH3_AE_PIN, 
  H2S_WE_PIN, H2S_AE_PIN, 
  NO2_WE_PIN, NO2_AE_PIN, 
  SO2_WE_PIN, SO2_AE_PIN, 
  OX_WE_PIN, OX_AE_PIN, 
  ANEM_PIN,
  TOTAL_ANALOG_PINS
}SensorAnalogPins;

typedef struct __attribute__((packed)) _sensors_readings{
  float co_ppb;
  float nh3_ppb;
  float no2_ppb;
  float so2_ppb;
  float ox_ppb;
  float h2s_ppb;
  float anem;
  float temp;
  float humidity;

} SensorsReadings;

SensorsReadings readings = {6.144}; 
uint16_t data_payload[9];

osjob_t sendjob;
uint32_t userUTCTime; // Seconds since the UTC epoch

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
u1_t NWKSKEY[16] = { 0x25, 0xBF, 0x21, 0xDC, 0x06, 0x33, 0x50, 0x70, 0x5F, 0xF1, 0x04, 0xDA, 0x37, 0x22, 0xAA, 0x51 };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
u1_t APPSKEY[16] = {0x5C, 0xB0, 0x03, 0x13, 0xAD, 0x1B, 0x19, 0x8E, 0xB3, 0x6B, 0x5A, 0x91, 0x25, 0x2F, 0x39, 0x93};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
u4_t DEVADDR = 0x260D27C1 ; // <-- Change this address for every node!

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst =14,
    .dio = { /*dio0*/ 26, /*dio1*/ 35, /*dio2*/ 34 },
    //.rxtx_rx_active = 0,
    //.rssi_cal = 10,
    //.spi_freq = 10000     /* 8 MHz */
};

void do_send(osjob_t* job){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        // data = [co_ppb, no2_ppb, ox_ppb, h2s_ppb, nh3_ppb, so2_ppb, temp, umid]
        // static uint8_t mydata[] = "Hello, world!";
      
#if (PRINT_MESSAGE == 1)
        for(int i = 1; i < 18; i+=2){
          //uint16_t aux = (data[i] << 8) | data[i-1];
          uint16_t aux = LMIC_f2uflt16(1.6);
          uint8_t b = (aux & 0xF000) >> 12;
          uint16_t f = aux & 0x0FFF; 
          Serial.print("Teste: "); Serial.print(aux);Serial.print(" : ");Serial.println(f / 4096.0 * pow(2, b-15));
          // f/4096 * 2^(b-15)
        }
#endif
        LMIC_requestNetworkTime(requestNetworkTimeCallback, &userUTCTime);
        LMIC_setTxData2(1, (unsigned char *) data_payload, 2*9, 0);
        //LMIC_setTxData2(1, (unsigned char *) s, sizeof(s), 0);
        Serial.println(F("Packet queued"));

     
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

volatile uint32_t timeReq = 0;
volatile bool flagTimeReq = true;
volatile bool flagADC = false;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t timer_aux = 0;

void IRAM_ATTR timerADC(){
  portENTER_CRITICAL_ISR(&timerMux);
  flagADC = true;
  ++timeReq > 86400 ? (flagTimeReq = true) : (flagTimeReq = false);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void init_timer(){
  timer = timerBegin(1, 80, true); // Timer0, clk divided by 80 and count down (abp clk)
  if(!timer){
    Serial.println("Erro timer");
  }
  timerAttachInterrupt(timer, &timerADC, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
}

void setup() {
  
    while (!Serial); // wait for Serial to be initialized
    Serial.begin(115200);
    delay(100);     // per sample code on RF_95 test
    Serial.println(F("Starting"));
    Serial.print(F("Frequency: ")); Serial.println(getCpuFrequencyMhz());
    Serial.print(F("Frequency ABP: ")); Serial.println(getApbFrequency());
    
    setenv("TZ","<-03>3",1);
    tzset();

    time_t t;
    time (&t);
    struct tm *timeinfo;
    timeinfo = localtime(&t);
    std::cout << timeinfo->tm_hour << "h" << timeinfo->tm_min << ":" << timeinfo->tm_sec << std::endl;

    init_lora(&sendjob, DEVADDR, NWKSKEY, APPSKEY);

    init_timer();

    Serial.println("Getting single-ended readings from AIN0..3");
    Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

    Wire.begin(ADS_SDA, ADS_SCL);
    Wire.setClock(10000);

    ads.setGain(GAIN_TWOTHIRDS);  // ADS1115: 2/3x gain +/- 6.144V  1 bit = 0.1875mV (default)
    if (!ads.begin(0x48, &Wire)) {
      Serial.println("Failed to initialize ADS.");
      //while (1);
    }else {
       Serial.println("ADS_ok");
    }
  
  // Configurando o mux 16x1
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // LED
  pinMode(25, OUTPUT);

  timer_aux = millis();

}

void loop() {

  uint16_t adc = 0;
  float v[13]; // L
  float anemometro=0, temperature=0, humidity=0;

  os_runloop_once();
  
  if(flagTimeReq){
    // Fazer o timeReq
    
    
    portENTER_CRITICAL(&timerMux);
    timeReq = 0;
    LMIC_requestNetworkTime(requestNetworkTimeCallback, &userUTCTime);
    portEXIT_CRITICAL(&timerMux);
        
  }
  
  if(flagADC){

    portENTER_CRITICAL(&timerMux);
    flagADC = false;
    portEXIT_CRITICAL(&timerMux);

    for(uint8_t i = 0; i < TOTAL_ANALOG_PINS; i++){

      digitalWrite(S0, bitRead(i, 0));digitalWrite(S1, bitRead(i, 1));
      digitalWrite(S2, bitRead(i, 2));digitalWrite(S3, bitRead(i, 3));
      delayMicroseconds(5); 

      //adc = ads.readADC_SingleEnded(0);
      //v[i] = ads.computeVolts(adc);
    }

    readings.co_ppb = (float)cob4_s1.ppb(1000*v[CO_WE_PIN], 1000*v[CO_AE_PIN], 20.0);
    readings.nh3_ppb = (float)cob4_s2.ppb(1000*v[NH3_WE_PIN], 1000*v[NH3_AE_PIN], 20.0);
   
    float *kkkkk;
    kkkkk = (float*)&readings; // Isso é uma gambi das boas

    //uint8_t *data = new uint8_t[18]; //  12B sensors + 4B temp and humidity + 2B anemom
    for(int i = 0; i < 9; i++)
        *(data_payload+i) = LMIC_f2uflt16(*(kkkkk + i)); // 6.144 is the adc max value

#if (PRINT_ANALOG_READS == 1)
    Serial.println("Imprimindo leituras adc");

    for(auto& i : v){
      Serial.print(i); Serial.print(" ");
    }
    Serial.println(" ");
    Serial.print("Temp: ");Serial.println(temperature);
    Serial.print("Umid: ");Serial.println(humidity);
#endif


    digitalWrite(25, !digitalRead(25));
  }

}
