#include <Arduino.h>
#include <iostream>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "esp32-hal-i2c.h"
#include "Adafruit_ADS1X15.h"

#include "ds1307lib.h"
#include "envcity_lora_config.hpp"
#include "Alphasense_GasSensors.hpp"
#include "DHT.h"
#include "am2302.hpp"
#include "mux.hpp"

#include "aqm_envcity_config.h"

Adafruit_ADS1X15 ads; 

SensorsReadings readings;
SensorVoltage voltages;

AlphasenseSensorParam param_h2s = {"H2S", H2SB4_n, 0.8, 353, 342, 2020, 1.616, 345, 344, 0};
Alphasense_H2S h2s(param_h2s);

AlphasenseSensorParam param_nh3 = {"asdas", COB4_n, 0.8, 775, 277, 59, 0.047, 277, 278, 0};
Alphasense_NH3 nh3(param_nh3);

ds1307_date_t date;

#define TTGO

#ifdef TC_TELECOM
/* TC TELECOM - Marcio Oyamada */

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
u1_t NWKSKEY[16] = {0xcb,0xb2,0x69,0x90,0xff,0xbc,0xcf,0x73,0x13,0x67,0x2a,0x5f,0xa3,0xec,0x20,0xe3};
// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
u1_t APPSKEY[16] = {0x82,0xca,0xc3,0x33,0xb6,0x36,0xae,0x11,0x1c,0x71,0xff,0x7a,0x1b,0x18,0x8f,0x38};
// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
u4_t DEVADDR = 0x260D7446 ; // <-- Change this address for every node!

#elif defined(TTGO)

u1_t NWKSKEY[16] = {0xCC, 0x4D, 0x60, 0xEC, 0xAC, 0xEC, 0xBA, 0xF7, 0x35, 0xFA, 0xAE, 0xE7, 0x3E, 0x07, 0xC0, 0x3D};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
u1_t APPSKEY[16] = {0x0C, 0x7B, 0x26, 0x49, 0x3D, 0x59, 0x72, 0x12, 0x4D, 0xFF, 0x70, 0x99, 0xB5, 0x33, 0xAA, 0x23};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
u4_t DEVADDR = 0x260D8129 ; // <-- Change this address for every node!

#endif

osjob_t sendjob;

void do_send(osjob_t* job){
    // Check if there is not a current TX/RX job running
    static int count = 5;
    static int voltageOrPpb = true;

    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        
        if(++count > 0){
                      
            if(voltageOrPpb){
                LMIC_setTxData2(1, (unsigned char *) &readings, sizeof(SensorsReadings) - sizeof(unsigned long long), 0);          
            } else {
                //Serial.print("temp: "); Serial.println(voltages.temp);
                //uint8_t *ptr = (uint8_t*)&voltages;
                //ESP_LOGE("lmic", "temp: %d", voltages.temp);
                //ESP_LOGE("lmic", "TEmp2 : %x %x", ptr[16] & 0xff, ptr[17] & 0xFf);
                LMIC_setTxData2(2, (unsigned char *) &voltages, sizeof(SensorVoltage), 0);
            }

            voltageOrPpb = !voltageOrPpb;
            
            count = 0;

        } else {

            Serial.print(F("Skipping. Count: ")); Serial.println(count);
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

        }
        
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    //.dio = { /*dio0*/ 26, /*dio1*/ 32, /*dio2*/ 33 },
    .dio = {26, 33, 32},
    //.rxtx_rx_active = 0,
    //.rssi_cal = 10,
    .spi_freq = 8000000     /* 8 MHz */
};


static esp_err_t s_example_write_file(const char *path, char *data)
{
    //Serial.printf("Opening file %s\n", path);
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        Serial.printf("Failed to open file for writing\n");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    //Serial.printf("File written\n");

    return ESP_OK;
}


void onEvent (ev_t ev) {

    std::cout << os_getTime() << std::endl;

    switch(ev) {
        case EV_SCAN_TIMEOUT:
            std::cout << "EV_SCAN_TIMEOUT" << std::endl;
            break;
        case EV_BEACON_FOUND:
            std::cout <<  "EV_BEACON_FOUND" << std::endl;
            break;
        case EV_BEACON_MISSED:
            std::cout << "EV_BEACON_MISSED" << std::endl;
            break;
        case EV_BEACON_TRACKED:
            std::cout << "EV_BEACON_TRACKED" << std::endl;
            break;
        case EV_JOINING:
            std::cout << "EV_JOINING"<< std::endl;
            break;
        case EV_JOINED:
            std::cout << "EV_JOINED" << std::endl;
           break;
        case EV_JOIN_FAILED:
           std::cout << "EV_JOIN_FAILED"<< std::endl;
            break;
        case EV_REJOIN_FAILED:
            std::cout << "EV_REJOIN_FAILED"<< std::endl;
            break;
        case EV_TXCOMPLETE:
            std::cout <<  "EV_TXCOMPLETE (includes waiting for RX windows)"<< std::endl;
            if (LMIC.txrxFlags & TXRX_ACK)
              std::cout << "Received ack"<< std::endl;
            if (LMIC.dataLen) {
              std::cout << "Received "<< std::endl;
              std::cout << LMIC.dataLen << std::endl;
              std::cout << " bytes of payload"<< std::endl;
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            std::cout << "EV_LOST_TSYNC"<< std::endl;
            break;
        case EV_RESET:
            std::cout << "EV_RESET"<< std::endl;
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            std::cout << "EV_RXCOMPLETE"<< std::endl;
            break;
        case EV_LINK_DEAD:
            std::cout << "EV_LINK_DEAD"<< std::endl;
            break;
        case EV_LINK_ALIVE:
            std::cout << "EV_LINK_ALIVE"<< std::endl;
            break;
        case EV_TXSTART:
            std::cout << "EV_TXSTART"<< std::endl;
            break;
        case EV_TXCANCELED:
            std::cout << "EV_TXCANCELED"<< std::endl;
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            std::cout << "EV_JOIN_TXCOMPLETE: no JoinAccept" << std::endl;
            break;
        default:
            std::cout << "Unknown event: "<< std::endl;
            std::cout << (unsigned) ev << std::endl;
            break;
    }
}

#define TAG "TESTESD"

extern SPIClass SPI;

extern "C" void app_main() {

    initArduino();
    
    Serial.begin(9600);
    while(!Serial);

    am2302_init();

    SPI.begin(5, 19, 27, 18);
    init_lora(&sendjob, DEVADDR, NWKSKEY, APPSKEY, 1);
    Serial.print("Pins SPI ");
    Serial.println(SPI.pinSS());

    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = "/sdcard";
    Serial.println("Initializing SD card");

       // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    Serial.println("Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = (gpio_num_t)15,
        .miso_io_num = (gpio_num_t)2,
        .sclk_io_num = (gpio_num_t)14,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        Serial.println("Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)13;
    slot_config.host_id = (spi_host_device_t)host.slot;

    Serial.println("Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            Serial.println("Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            Serial.printf("Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
    }
    Serial.println("Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    
    Wire.begin(21, 22);
    if(!ads.begin(0x48, &Wire)){
        Serial.println("ADS1115 fail");
    }else {
        Serial.println("ADS1115 OK");
    }

    gpio_num_t pins[] = {GPIO_NUM_12, GPIO_NUM_0, GPIO_NUM_25};
    mux Mux(pins, 3);

    while(true){

        float v[TOTAL_ANALOG_PINS];
       os_runloop_once();

       int16_t temp = 0, umid = 0; 
       am2302_read(&temp, &umid);
       
        readings.temp = temp;
        readings.humidity = umid;

        voltages.temp = temp;
        voltages.umid = umid;

        Serial.print("Temperatura: "); Serial.print(temp/10.0); Serial.println(" C");
        Serial.print("Umidade: "); Serial.print(umid/10.0); Serial.println(" %");

       Serial.println("Reading ADC: ");
       for(int i = 0; i <= TOTAL_ANALOG_PINS; i++){

            Mux.selectOutput(i);
            uint16_t adc = ads.readADC_SingleEnded(0);
            v[i] = ads.computeVolts(adc);
            Serial.print(v[i]); Serial.print(", ");
            ets_delay_us(10);
       }
       Serial.println("");

        Serial.println("Concentration:");
        Serial.print("H2S: "); Serial.println(h2s.ppb(v[H2S_WE_PIN], v[H2S_AE_PIN], temp/10.0));
        Serial.print("NH3: "); Serial.println(nh3.simpleRead(v[NH3_WE_PIN], v[NH3_AE_PIN]));

        h2s.fourAlgorithms(v[H2S_WE_PIN], v[H2S_AE_PIN], readings.h2s_ppb, temp/10.0);
        readings.nh3_ppb[0] = nh3.simpleRead(v[NH3_WE_PIN], v[NH3_AE_PIN]);

        voltages.h2s_we = v[H2S_WE_PIN];
        voltages.h2s_ae = v[H2S_AE_PIN];
        voltages.nh3_we = v[NH3_WE_PIN];
        voltages.nh3_ae = v[NH3_AE_PIN];

       vTaskDelay(pdMS_TO_TICKS(1000)); 
       // s_example_write_file("/sdcard/hello.txt", "Hello world!\n");

    }
}