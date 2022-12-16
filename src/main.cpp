#include <iostream>
#include <chrono>
#include <cmath>
#include <time.h>

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>

#include "Alphasense_GasSensors.hpp"
#include "anemometro_analog.hpp"
#include "envcity_lora_config.hpp"
#include "hm3301_lib.hpp"
#include "DHT.h"

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "SPI.h"

#define PRINT_ANALOG_READS 1
#define PRINT_MESSAGE 0

// COb4 -> 354
AlphasenseSensorParam param1 = {"CO-B4", COB4_n, 0.8, 330, 316, 510, 0.408, 336, 321, 0};
Alphasense_COB4 cob4_s1(param1);

// AlphasenseSensorParam param2 = {"NH3-B1", COB4_n, 0.8, 775, 277, 59, 0.047, 277, 278, 0};
// Alphasense_NH3 nh3(param2);

//AlphasenseSensorParam param3 = {"H2S-B4", H2SB4_n, 0.8, 353,342,2020, 1.616, 345,344, 0};
//Alphasense_H2S h2s(param3);

AlphasenseSensorParam param4 = {"0X", OXB431_n, -0.73, 229, 234, -506, 0.369, 237, 242, -587};
Alphasense_OX ox(param4);

AlphasenseSensorParam param5 = {"NO2", NO2B43F_n, -0.73, 222, 212, -424, 0.31, 230, 220, 0};
Alphasense_NO2 no2(param5);

AlphasenseSensorParam param6 = {"SO2", SO2B4_n, 0.8, 361, 350, 363, 0.29, 335, 343, 0};
Alphasense_SO2 so2(param6);

Anemometro anem;

HM330X hm3301;

#define DHTPIN 23    // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

#define ADS_SDA 21
#define ADS_SCL 22
Adafruit_ADS1115 ads;
enum {S0 = 17, S1 = 2, S2 = 4, S3 = 16}; // Pinos Multiplexador

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
  SO2_WE_PIN, SO2_AE_PIN, 
  H2S_WE_PIN, H2S_AE_PIN, 
  NH3_WE_PIN, NH3_AE_PIN, 
  CO_WE_PIN, CO_AE_PIN, 
  NO2_WE_PIN = 8, NO2_AE_PIN = 9, 
  OX_WE_PIN = 10, OX_AE_PIN = 11, 
  ANEM_PIN,
  TOTAL_ANALOG_PINS
}SensorAnalogPins;


typedef struct __attribute__((packed)) _sensors_readings{
  float co_ppb[4]; //0,1,2,3
  //float nh3_ppb;//4,5,6,7
  float no2_ppb[4];//8,9,10,11
  float so2_ppb[4];//12,13,14,15
  float ox_ppb[4]; //16
  //float h2s_ppb; //20
  float anem; // 24
  float temp; // 28
  float humidity; //32
  //HM330X_RES hm3301;
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
  unsigned long long utc_time;

} SensorsReadings;

SensorsReadings readings = {6.144}; 

osjob_t sendjob;
uint32_t userUTCTime; // Seconds since the UTC epoch

#define TC_TELECOM
#ifdef TC_TELECOM
/* TC TELECOM - Marcio Oyamada */

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
//cbb26990ffbccf7313672a5fa3ec20e3
u1_t NWKSKEY[16] = {0xcb,0xb2,0x69,0x90,0xff,0xbc,0xcf,0x73,0x13,0x67,0x2a,0x5f,0xa3,0xec,0x20,0xe3};
// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
u1_t APPSKEY[16] = {0x82,0xca,0xc3,0x33,0xb6,0x36,0xae,0x11,0x1c,0x71,0xff,0x7a,0x1b,0x18,0x8f,0x38};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
u4_t DEVADDR = 0x260D7446 ; // <-- Change this address for every node!
#else // TTN - Mateus Maruzka Roncaglio

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

#endif

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst =14,
    .dio = { /*dio0*/ 26, /*dio1*/ 35, /*dio2*/ 34 },
    //.rxtx_rx_active = 0,
    //.rssi_cal = 10,
    //.spi_freq = 10000     /* 8 MHz */
};

#define CHUNK_SIZE 31
int sendChunkedI2CMessage(uint8_t *msg, int _msgLength, int addr){
  
  int msgLength = _msgLength;
  
  if(msgLength < CHUNK_SIZE)
    return -1;


  Serial.print("Sending: "); Serial.print(_msgLength);Serial.println(" bytes");

  int chunkLen = CHUNK_SIZE;

  for(int i = 0; i < (_msgLength / CHUNK_SIZE + 1); i++){
    
    msgLength < 32 ? chunkLen = _msgLength % CHUNK_SIZE : chunkLen = CHUNK_SIZE;
    
    Serial.print("chunk: "); Serial.println(chunkLen);
    Serial.print("i: "); Serial.println(i);

    Wire.beginTransmission(addr);
    Wire.write(&msg[i*CHUNK_SIZE], chunkLen);
    byte error = Wire.endTransmission();    // stop transmitting
    Serial.print("Error: "); Serial.println(error);
    msgLength = msgLength - CHUNK_SIZE;
  }
  return 0;
}

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
        if(flagTimeReq)
          LMIC_requestNetworkTime(requestNetworkTimeCallback, &userUTCTime);

        //Serial.print("COlmic: "); Serial.println(readings.co_ppb);
        //Serial.print("Size: "); Serial.println(sizeof(SensorsReadings));
        LMIC_setTxData2(1, (unsigned char *) &readings, sizeof(SensorsReadings) - sizeof(unsigned long long), 0);
        Serial.println(F("Packet queued"));

    }
    // Next TX is scheduled after TX_COMPLETE event.
}

volatile uint32_t timeReq = 0;
volatile bool flagTimeReq = true;
volatile bool flagADC = false;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

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
  timerAlarmWrite(timer, 3000000, true);
  timerAlarmEnable(timer);
}


void setup() {

    while (!Serial); // wait for Serial to be initialized
    Serial.begin(115200);
    delay(100);     // per sample code on RF_95 test
    Serial.println(F("Starting"));
    Serial.print(F("Frequency: ")); Serial.println(getCpuFrequencyMhz());
    Serial.print(F("Frequency ABP: ")); Serial.println(getApbFrequency());
    Serial.println(sizeof(readings));
    dht.begin();

    time_t t;
    time (&t);
    struct tm *timeinfo;
    timeinfo = localtime(&t);
    std::cout << timeinfo->tm_hour << "h" << timeinfo->tm_min << ":" << timeinfo->tm_sec << std::endl;
    
    init_timer();

#ifdef TC_TELECOM
    init_lora(&sendjob, DEVADDR, NWKSKEY, APPSKEY, (u1_t) 0);
#else
    init_lora(&sendjob, DEVADDR, NWKSKEY, APPSKEY, (u1_t) 1);
#endif

    Serial.println("Getting single-ended readings from AIN0..3");
    Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

    Wire.begin(ADS_SDA, ADS_SCL);
    Wire.setClock(100000);

    ads.setGain(GAIN_TWOTHIRDS);  // ADS1115: 2/3x gain +/- 6.144V  1 bit = 0.1875mV (default)
    if (!ads.begin(0x48, &Wire)) {
      Serial.println("Failed to initialize ADS.");
      //while (1);
    }else {
       Serial.println("ADS_ok");
    }
  
    if (hm3301.init()) {
        Serial.println("HM330X init failed!!!");
        //while (1);
    }

  // Configurando o mux 16x1
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);



}

void loop() {

  uint16_t adc = 0;
  float v[13]; // L
  uint8_t buffer_hm3301[30];

  HM330X_RES hm3301_res;

  os_runloop_once();

  if(flagADC){

    portENTER_CRITICAL(&timerMux);
    flagADC = false;
    portEXIT_CRITICAL(&timerMux);

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    
    /*if (hm3301.read_sensor_value(buffer_hm3301, 29)) {
        Serial.println("HM330X read result failed!!!");
    }
    parse_result_value(buffer_hm3301);
    parse_result(buffer_hm3301, &hm3301_res);*/

    // readings.pm1_0 = hm3301_res.pm1_0_atmos; 
    // readings.pm2_5 = hm3301_res.pm2_5_atmos;
    // readings.pm10 = hm3301_res.pm10_atmos;
    readings.pm1_0 = 0; 
    readings.pm2_5 = 0;
    readings.pm10 = 0;

    for(uint8_t i = 0; i < TOTAL_ANALOG_PINS; i++){

      digitalWrite(S0, bitRead(i, 0));digitalWrite(S1, bitRead(i, 1));
      digitalWrite(S2, bitRead(i, 2));digitalWrite(S3, bitRead(i, 3));
      delayMicroseconds(5); 

      adc = ads.readADC_SingleEnded(0);
      v[i] = ads.computeVolts(adc);
    }

    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    if(!isnan(temp) && !isnan(humidity)){
      readings.temp = temp;
      readings.humidity = humidity;
    }else{
      Serial.println("Error reading temp and humid");
    }
    
    cob4_s1.fourAlgorithms(1000*v[CO_WE_PIN], 1000*v[CO_AE_PIN], readings.co_ppb, readings.temp);
    // readings.h2s_ppb = (float)h2s.ppb(1000*v[H2S_WE_PIN], 1000*v[H2S_AE_PIN], readings.temp);
    no2.fourAlgorithms(1000*v[NO2_WE_PIN], 1000*v[NO2_AE_PIN], readings.no2_ppb, readings.temp);
    so2.fourAlgorithms(1000*v[SO2_WE_PIN], 1000*v[SO2_AE_PIN], readings.so2_ppb, readings.temp);
    // readings.nh3_ppb = (float)nh3.ppb(1000*v[NH3_WE_PIN], 1000*v[NH3_AE_PIN], readings.temp);
    ox.fourAlgorithms(1000*v[OX_WE_PIN], 1000*v[OX_AE_PIN], readings.ox_ppb, readings.no2_ppb[0],readings.temp);
    
    //Old_Average + ((New_Sample – Old_Average) / (Sample_Size + 1))
    readings.anem = readings.anem + (anem.windSpeed(v[ANEM_PIN]) - readings.anem) / 30;

    // u = [1, 2, 3, 4, 5, 6, 7] / 7 = 4 
    // u = [2, 3, 4, 5, 6, 7, 20] / 7 = 43 / 7 = 6,14
    // u = 4 * 6 + 20 / 7 = 44 / 7 = 6,28

    time_t t;
    time(&t);
    readings.utc_time = t;

    //sendChunkedI2CMessage((uint8_t *) &readings, sizeof(readings), 0x30);

#if (PRINT_ANALOG_READS == 1)
    
    Serial.println("Imprimindo leituras adc");
    Serial.printf("%ld UTC ", t);
    for(auto& i : v){
      Serial.print(i); Serial.print(" ");
    }

    Serial.println(" ");
    Serial.print("Temp: ");Serial.println(readings.temp);
    Serial.print("Umid: ");Serial.println(readings.humidity);
    Serial.print("CO: ");Serial.println(readings.co_ppb[0]);
    Serial.print("NO2: ");Serial.println(readings.no2_ppb[0]);
    Serial.print("SO2: ");Serial.println(readings.so2_ppb[0]);
    Serial.print("OX: ");Serial.println(readings.ox_ppb[0]);
    Serial.print("Anem: ");Serial.println(readings.so2_ppb[0]);
    Serial.print("pm1_0: ");Serial.println(readings.pm1_0);
    Serial.print("pm2_5: ");Serial.println(readings.pm2_5);
    Serial.print("pm10: ");Serial.println(readings.pm10);


#endif

  }

}