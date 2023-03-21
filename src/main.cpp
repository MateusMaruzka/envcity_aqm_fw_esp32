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

u1_t NWKSKEY[16] = { 0xCC, 0x4D, 0x60, 0xEC, 0xAC, 0xEC, 0xBA, 0xF7, 0x35, 0xFA, 0xAE, 0xE7, 0x3E, 0x07, 0xC0, 0x3D };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
u1_t APPSKEY[16] = {0x0C, 0x7B, 0x26, 0x49, 0x3D, 0x59, 0x72, 0x12, 0x4D, 0xFF, 0x70, 0x99, 0xB5, 0x33, 0xAA, 0x23};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
u4_t DEVADDR = 0x260D8129 ; // <-- Change this address for every node!

osjob_t sendjob;



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
        unsigned char c[] = "Hello World";
        //Serial.print("COlmic: "); Serial.println(readings.co_ppb);
        //Serial.print("Size: "); Serial.println(sizeof(SensorsReadings));
        LMIC_setTxData2(1, (unsigned char *) c, sizeof(c), 0);
        Serial.println(F("Packet queued"));

    }
    // Next TX is scheduled after TX_COMPLETE event.
}

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = { /*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32 },
    //.rxtx_rx_active = 0,
    //.rssi_cal = 10,
    .spi_freq = 8000000     /* 8 MHz */
};

void init_lora(osjob_t *sendjob, u4_t DEVADDR, u1_t *NWKSKEY, u1_t *APPSKEY, u1_t subBand){

    // LMIC init
    os_init();
    //os_init_ex(&lmic_pins);
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);

    //LMIC_setClockError(65535  * 1 / 100);
    #if defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand((u1_t)subBand);

    #elif defined(CFG_as923)
    // Set up the channels used in your country. Only two are defined by default,
    // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
    // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);   
    #else
    # error Region not supported
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    //do_send(&sendjob);

    do_send(sendjob);
}

static esp_err_t s_example_write_file(const char *path, char *data)
{
    Serial.printf("Opening file %s\n", path);
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        Serial.printf("Failed to open file for writing\n");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    Serial.printf("File written\n");

    return ESP_OK;
}

#define TX_INTERVAL 30

void onEvent (ev_t ev) {


    //Serial.print(os_getTime());
    std::cout << os_getTime() << std::endl;
    //Serial.print(": ");
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    //serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
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
extern "C" void app_main() {

    initArduino();
    
    Serial.begin(9600);
    while(!Serial);

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
        return;
    }
    Serial.println("Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);


    
    SPI.begin(5, 19, 27, 18);
    init_lora(&sendjob, DEVADDR, NWKSKEY, APPSKEY, 1);
    
    Serial.print("Pins SPI ");
    Serial.println(SPI.pinSS());
    
    while(true){
       // Serial.println("Loop");
       os_runloop_once();
       vTaskDelay(pdMS_TO_TICKS(500));
       s_example_write_file("/sdcard/hello.txt", "Hello world!\n");
    }
}