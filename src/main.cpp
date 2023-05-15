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
#include "pms5003.h"
#include "mux.hpp"

#include "Adafruit_PM25AQI.h"
#include "SoftwareSerial.h"

#include "aqm_envcity_config.h"

Adafruit_ADS1X15 ads;

SoftwareSerial pmSerial(4);
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

int16_t temp = 0, umid = 0;

SensorsReadings readings;
SensorVoltage voltages;

AlphasenseSensorParam param1 = {"CO-B4", COB4_n, 0.8, 353, 328, 454, 0.363, 343, 328, 0};
Alphasense_COB4 cob4_s1(param1);

AlphasenseSensorParam param4 = {"0X", OXB431_n, -0.73, 229, 234, -506, 0.369, 237, 242, -587};
Alphasense_OX ox(param4);

AlphasenseSensorParam param5 = {"NO2", NO2B43F_n, -0.73, 222, 212, -424, 0.31, 230, 220, 0};
Alphasense_NO2 no2(param5);

AlphasenseSensorParam param6 = {"SO2", SO2B4_n, 0.8, 361, 350, 363, 0.29, 335, 343, 0};
Alphasense_SO2 so2(param6);

ds1307_date_t date;

#define TTGO

#ifdef TC_TELECOM
/* TC TELECOM - Marcio Oyamada */

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
u1_t NWKSKEY[16] = {0xcb, 0xb2, 0x69, 0x90, 0xff, 0xbc, 0xcf, 0x73, 0x13, 0x67, 0x2a, 0x5f, 0xa3, 0xec, 0x20, 0xe3};
// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
u1_t APPSKEY[16] = {0x82, 0xca, 0xc3, 0x33, 0xb6, 0x36, 0xae, 0x11, 0x1c, 0x71, 0xff, 0x7a, 0x1b, 0x18, 0x8f, 0x38};
// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
u4_t DEVADDR = 0x260D7446; // <-- Change this address for every node!

#elif defined(TTGO)

u1_t NWKSKEY[16] = {0xCC, 0x4D, 0x60, 0xEC, 0xAC, 0xEC, 0xBA, 0xF7, 0x35, 0xFA, 0xAE, 0xE7, 0x3E, 0x07, 0xC0, 0x3D};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
u1_t APPSKEY[16] = {0x0C, 0x7B, 0x26, 0x49, 0x3D, 0x59, 0x72, 0x12, 0x4D, 0xFF, 0x70, 0x99, 0xB5, 0x33, 0xAA, 0x23};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
u4_t DEVADDR = 0x260D8129; // <-- Change this address for every node!


//u1_t NWKSKEY[16] = {0x9c,0xa2,0x12,0xe7,0x3a,0x65,0xcd,0x4d,0x2d,0x68,0xe2,0xc7,0x78,0x08,0x99,0xaa};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
//u1_t APPSKEY[16] = {0xfb,0x91,0x5c,0x4f,0xb4,0x9c,0xc5,0x41,0xf3,0xd4,0xe2,0xf6,0x47,0x4c,0xc3,0x59};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
//u4_t DEVADDR = 0x0f66b66a; // <-- Change this address for every node!

#endif

osjob_t sendjob;

void do_send(osjob_t *job)
{
    // Check if there is not a current TX/RX job running
    static int count = 5;
    static int voltageOrPpb = true;

    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {

        if (++count > 1)
        {

            if (voltageOrPpb)
            {
                LMIC_setTxData2(1, (unsigned char *)&readings, sizeof(SensorsReadings), 0);
            }
            else
            {
                LMIC_setTxData2(2, (unsigned char *)&voltages, sizeof(SensorVoltage), 0);
            }

            voltageOrPpb = !voltageOrPpb;

            count = 0;
        }
        else
        {

            Serial.print(F("Skipping. Count: "));
            Serial.println(count);
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
    .spi_freq = 8000000 /* 8 MHz */
};

static esp_err_t s_file_exits(const char *path)
{

    struct stat st;
    if (stat(path, &st) == 0)
    {
        Serial.println("File exists");
        // Delete it if it exists
        return ESP_OK;
    }
    return ESP_FAIL;
}
static esp_err_t s_write_voltage_to_file(const char *path, char *mode, SensorVoltage my_sensor_voltage, char *time)
{

    FILE *f = fopen(path, mode);
    if (f == NULL)
    {
        ESP_LOGE("SDCARD", "Failed to open file for writing\n");
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGI("SDCARD", "Writing to file");
        // open a file for writing

        // write each member of the struct to the file, separated by commas
        fprintf(f, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%u,%u,%u,%s\n",
                my_sensor_voltage.co_we,
                my_sensor_voltage.co_ae,
                my_sensor_voltage.no2_we,
                my_sensor_voltage.no2_ae,
                my_sensor_voltage.so2_we,
                my_sensor_voltage.so2_ae,
                my_sensor_voltage.ox_we,
                my_sensor_voltage.ox_ae,
                my_sensor_voltage.anem,
                my_sensor_voltage.temp,
                my_sensor_voltage.humidity,
                my_sensor_voltage.pm1_0,
                my_sensor_voltage.pm2_5,
                my_sensor_voltage.pm10,
                time);

        // close the file
        fclose(f);
        return ESP_OK;
    }
}
static esp_err_t s_example_write_file(const char *path, char *data, char *mode)
{
    // Serial.printf("Opening file %s\n", path);
    FILE *f = fopen(path, mode);
    if (f == NULL)
    {
        Serial.printf("Failed to open file for writing\n");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    // Serial.printf("File written\n");

    return ESP_OK;
}
static void getSensorVoltage(float v[], SensorVoltage *sensorVoltage)
{
    sensorVoltage->co_we = v[CO_WE_PIN];
    sensorVoltage->co_ae = v[CO_AE_PIN];
    sensorVoltage->no2_we = v[NO2_WE_PIN];
    sensorVoltage->no2_ae = v[NO2_AE_PIN];
    sensorVoltage->so2_we = v[SO2_WE_PIN];
    sensorVoltage->so2_ae = v[SO2_AE_PIN];
    sensorVoltage->ox_we = v[OX_WE_PIN];
    sensorVoltage->ox_ae = v[OX_AE_PIN];
    sensorVoltage->anem = v[ANEM_PIN];
}

void onEvent(ev_t ev)
{

    std::cout << os_getTime() << std::endl;

    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        std::cout << "EV_SCAN_TIMEOUT" << std::endl;
        break;
    case EV_BEACON_FOUND:
        std::cout << "EV_BEACON_FOUND" << std::endl;
        break;
    case EV_BEACON_MISSED:
        std::cout << "EV_BEACON_MISSED" << std::endl;
        break;
    case EV_BEACON_TRACKED:
        std::cout << "EV_BEACON_TRACKED" << std::endl;
        break;
    case EV_JOINING:
        std::cout << "EV_JOINING" << std::endl;
        break;
    case EV_JOINED:
        std::cout << "EV_JOINED" << std::endl;
        break;
    case EV_JOIN_FAILED:
        std::cout << "EV_JOIN_FAILED" << std::endl;
        break;
    case EV_REJOIN_FAILED:
        std::cout << "EV_REJOIN_FAILED" << std::endl;
        break;
    case EV_TXCOMPLETE:
        std::cout << "EV_TXCOMPLETE (includes waiting for RX windows)" << std::endl;
        if (LMIC.txrxFlags & TXRX_ACK)
            std::cout << "Received ack" << std::endl;
        if (LMIC.dataLen)
        {
            std::cout << "Received " << std::endl;
            std::cout << LMIC.dataLen << std::endl;
            std::cout << " bytes of payload" << std::endl;
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        std::cout << "EV_LOST_TSYNC" << std::endl;
        break;
    case EV_RESET:
        std::cout << "EV_RESET" << std::endl;
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        std::cout << "EV_RXCOMPLETE" << std::endl;
        break;
    case EV_LINK_DEAD:
        std::cout << "EV_LINK_DEAD" << std::endl;
        break;
    case EV_LINK_ALIVE:
        std::cout << "EV_LINK_ALIVE" << std::endl;
        break;
    case EV_TXSTART:
        std::cout << "EV_TXSTART" << std::endl;
        break;
    case EV_TXCANCELED:
        std::cout << "EV_TXCANCELED" << std::endl;
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        break;
    case EV_JOIN_TXCOMPLETE:
        std::cout << "EV_JOIN_TXCOMPLETE: no JoinAccept" << std::endl;
        break;
    default:
        std::cout << "Unknown event: " << std::endl;
        std::cout << (unsigned)ev << std::endl;
        break;
    }
}

#define TAG "TESTESD"

extern SPIClass SPI;

extern "C" void app_main()
{

    initArduino();

    Serial.begin(9600);
    while (!Serial);

    //Serial2.begin(9600, SERIAL_8N1, 4, 25);
    //pinMode(4, INPUT_PULLUP);
    //pinMode(25, OUTPUT);

    //if(!aqi.begin_UART(&Serial2)){
    //    ESP_LOGE(TAG, "PM sensor error");
    //}
    pms5003_init();
    Wire.begin(21, 22, 100000);

    esp_err_t ret;

    am2302_init();

    ret = ds1307_init_default();
    ESP_LOGE("INIT", "ds1307_init_default: %s", esp_err_to_name(ret));

    SPI.begin(5, 19, 27, 18);
    init_lora(&sendjob, DEVADDR, NWKSKEY, APPSKEY, 1);

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
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
    if (ret != ESP_OK)
    {
        Serial.println("Failed to initialize bus.");
        //return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)13;
    slot_config.host_id = (spi_host_device_t)host.slot;

    Serial.println("Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            Serial.println("Failed to mount filesystem. "
                           "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            Serial.printf("Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                          esp_err_to_name(ret));
        }

    } else {
        
        sdmmc_card_print_info(stdout, card);

        if (s_file_exits("/sdcard/log.csv") == ESP_OK)
        {
            Serial.println("File exists");
            ESP_LOGE("SETUP", "Datalogger file exists");
        }
        else
        {
            ESP_LOGE("SETUP", "Datalogger file not exists. Creating new file");
            s_example_write_file("/sdcard/log.csv", "co_we,co_ae,no2_we,no2_ae,so2_we,so2_ae,ox_we,ox_ae,anem,temp,umid,pm1.0,pm2.5,pm10.0, utc_time\n", "w");
        }

        Serial.println("Filesystem mounted");

    }

    // Card has been initialized, print its properties

    if (!ads.begin(0x48, &Wire))
    {
        Serial.println("ADS1115 fail");
    }
    else
    {
        Serial.println("ADS1115 OK");
    }

    gpio_num_t pins[] = {GPIO_NUM_12, GPIO_NUM_0, GPIO_NUM_25};
    mux Mux(pins, 3);

    while (true)
    {

        float v[TOTAL_ANALOG_PINS];
        os_runloop_once();

        am2302_read(&temp, &umid);

        readings.temp = temp;
        readings.humidity = umid;

        voltages.temp = temp;
        voltages.humidity = umid;

        Serial.print("Temperatura: ");
        Serial.print(temp / 10.0);
        Serial.println(" C");
        Serial.print("Umidade: ");
        Serial.print(umid / 10.0);
        Serial.println(" %");

        pms_sensor_data_t data;
        pms5003_read(&data);
        print_pms_sensor_data(&data);
  
        Serial.println("Reading ADC: ");
        Wire.flush();
        for (int i = 0; i <= TOTAL_ANALOG_PINS; i++)
        {

            Mux.selectOutput(i);
            //uint16_t adc = ads.readADC_SingleEnded(0);
            //v[i] = ads.computeVolts(adc);
            //Serial.print(v[i]);
            Serial.print(", ");
            ets_delay_us(10);
        }
        Serial.println("");

        Serial.println("Concentration:");

        cob4_s1.fourAlgorithms(1000 * v[CO_WE_PIN], 1000 * v[CO_AE_PIN], readings.co_ppb, readings.temp);
        no2.fourAlgorithms(1000 * v[NO2_WE_PIN], 1000 * v[NO2_AE_PIN], readings.no2_ppb, readings.temp);
        so2.fourAlgorithms(1000 * v[SO2_WE_PIN], 1000 * v[SO2_AE_PIN], readings.so2_ppb, readings.temp);
        ox.fourAlgorithms(1000 * v[OX_WE_PIN], 1000 * v[OX_AE_PIN], readings.ox_ppb, readings.no2_ppb[0], readings.temp);

        // Fill the struct with the voltage readings
        getSensorVoltage(v, &voltages);

        ds1307_date_t date;
        esp_err_t err = ds1307_read_date(&date);
        char time[26];
        snprintf(time, sizeof(time), "20%d-%d-%d-%d-%d-%d", date.year, date.month, date.day, date.hour, date.minute, date.second);

        if (err != ESP_OK)
        {
            ESP_LOGE("MAIN", "Error reading date from DS1307: %s", esp_err_to_name(err));
        }
        ESP_LOGI("MAIN", "Date: %d/%d/%d %d:%d:%d", date.day, date.month, date.year, date.hour, date.minute, date.second);

        // Old_Average + ((New_Sample – Old_Average) / (Sample_Size + 1))
        // readings.anem = readings.anem + (anem.windSpeed(v[ANEM_PIN]) - readings.anem) / 30;

        s_write_voltage_to_file("/sdcard/log.csv", "a",voltages, time);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}