#include <string>
//#include <heltec.h>

#include <Adafruit_ADS1X15.h>
#include <Wire.h>

#include <ArduinoJson.h>
#include <arduino_lmic.h>
#include <Arduino.h>
#include <lmic.h>
#include "arduino_lmic_hal_configuration.h"
#include <hal/hal.h>

#include "Alphasense_GasSensors.hpp"
#include "anemometro_analog.hpp"
#include "envcity_lora_config.hpp"

//#include <driver/timer.h>
//#include "DHT.h"

#define PRINT_ANALOG_READS 0
//#include "DHT.h"

AlphasenseSensorParam param1 = {"CO-B4", COB4_n,0.8,340,342, 475,0.38,0.28,310, 341, 0};
Alphasense_COB4 cob4(param1);

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
//enum {S0 = 16, S1 = 17, S2 = 4, S3 = 2}; // funcionou 
enum {S0 = 17, S1 = 2, S2 = 4, S3 = 16};

//#include <oled/SSD1306Wire.h>
//SSD1306Wire *display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_128_64);

static uint8_t mydata[] = "Hello, world!";

osjob_t sendjob;

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
u1_t NWKSKEY[16] = { 0x25, 0xBF, 0x21, 0xDC, 0x06, 0x33, 0x50, 0x70, 0x5F, 0xF1, 0x04, 0xDA, 0x37, 0x22, 0xAA, 0x51 };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
u1_t APPSKEY[16] = { 0x2B, 0x3C, 0x26, 0xBC, 0x96, 0x0C, 0xAA, 0x3D, 0xE1, 0x2A, 0x70, 0xEB, 0x8D, 0xED, 0x2F, 0xF7 };

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

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        // ler aqui
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

volatile bool flagADC = false;
uint32_t timer_aux = 0;
volatile uint32_t lastIntrAt = 0;
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR timerADC(){
  portENTER_CRITICAL_ISR(&timerMux);
  flagADC = true;
  lastIntrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void init_timer(){
  timer = timerBegin(0, 240, false); // Timer0, clk divided by 240 and count down
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
   
    
    //dht.begin();

    init_lora(&sendjob, DEVADDR, NWKSKEY, APPSKEY);

    timer = timerBegin(1, 80, true); // Timer0, clk divided by 80 and count down (abp clk)
    if(!timer){
      Serial.println("Erro timer");
    }
    timerAttachInterrupt(timer, &timerADC, true);
    timerAlarmWrite(timer, 1000000, true);
    timerAlarmEnable(timer);

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

  // Lê sensores Alphasense
  //digitalWrite(25, HIGH);

  uint16_t adc = 0;
  float v[12];
  float anemometro=0, temperature=0, humidity=0;

  /*for(uint8_t i = 0; i < (6*2); i++){

      digitalWrite(S0, bitRead(i, 0));
      digitalWrite(S1, bitRead(i, 1));
      digitalWrite(S2, bitRead(i, 2));
      digitalWrite(S3, bitRead(i, 3));
      
      delayMicroseconds(10); 

      //Serial.print("S0 "); Serial.println(digitalRead(S0));
      //Serial.print("S1 "); Serial.println(digitalRead(S1));
      //Serial.print("S2 "); Serial.println(digitalRead(S2));
      //Serial.print("S3 "); Serial.println(digitalRead(S3));
      //Serial.println("");

      delayMicroseconds(1000); 
      adc = ads.readADC_SingleEnded(0);
      v[i] = ads.computeVolts(adc);
  }*/

  // Seleciona pino 12 do mux p/ ler o anemômetro
  /*digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  delayMicroseconds(10); 
  adc = ads.readADC_SingleEnded(0);
  anemometro = ads.computeVolts(adc);*/


#if (PRINT_ANALOG_READS == 1)
  Serial.println("Imprimindo leituras adc");

  for(auto& i : v){
    Serial.print(i); Serial.print(" ");
  }
  Serial.println(" ");
  Serial.print("Temp: ");Serial.println(temperature);
  Serial.print("Umid: ");Serial.println(humidity);
#endif

  os_runloop_once();

  if(flagADC){

    portENTER_CRITICAL(&timerMux);
    flagADC = false;
    portEXIT_CRITICAL(&timerMux);

    //Serial.print("Tempo: ");Serial.println(lastIntrAt - timer_aux);
    timer_aux = lastIntrAt;

    digitalWrite(25, !digitalRead(25));
  }

}
