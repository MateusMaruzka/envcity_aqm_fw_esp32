#include "envcity_lora_config.hpp"


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

    //
    //LMIC_setAdrMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,16);

    // Start job
    //do_send(&sendjob);
    do_send(sendjob);
}


void onEvent (ev_t ev) {

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
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            //os_setTimedCallback(&sendVoltjJob, os_getTime()+sec2osticks(10), sendVoltageCallback);

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

void requestNetworkTimeCallback(void *ptrUTCtime, int flagSuccess){

    uint32_t *pUserUTCTime = (uint32_t *) ptrUTCtime;

    lmic_time_reference_t lmicTimeReference;

    if (flagSuccess != 1) {
        std::cout <<  "USER CALLBACK: Not a success" << std::endl;
        return;
    }

    flagSuccess = LMIC_getNetworkTimeReference(&lmicTimeReference);
    if (flagSuccess != 1) {
        std::cout << "USER CALLBACK: LMIC_getNetworkTimeReference didn't succeed" << std::endl;
        return;
    }
        // Update userUTCTime, considering the difference between the GPS and UTC
    // epoch, and the leap seconds // 315.964.800 
    *pUserUTCTime = lmicTimeReference.tNetwork + 315964800 - 18; // GPStime is 18sec ahead utc time

    // Add the delay between the instant the time was transmitted and
    // the current time
    // Current time, in ticks
    ostime_t ticksNow = os_getTime();
    ostime_t ticksRequestSent = lmicTimeReference.tLocal;// Time when the request was sent, in ticks
    uint32_t requestDelaySec = osticks2ms(ticksNow - ticksRequestSent) / 1000;
    *pUserUTCTime += requestDelaySec;
    std::cout << "secs " << lmicTimeReference.tNetwork << std::endl;
    std::cout << "UTC " << *pUserUTCTime << std::endl;

    // Update the system time with the time read from the network
    //setTime(*pUserUTCTime); #include <TimeLib.h>
    // adapted from https://portal.vidadesilicio.com.br/esp32-utilizando-o-rtc-interno-para-datas/
    timeval tv;//Cria a estrutura temporaria para funcao abaixo.
    tv.tv_sec = *pUserUTCTime;//Atribui minha data atual. Voce pode usar o NTP para isso ou o site citado no artigo!
    settimeofday(&tv, NULL);//Configura o RTC para manter a data atribuida atualizada.

    time_t t;
    time (&t);
    struct tm *timeinfo;
    timeinfo = localtime(&t);

    std::cout << timeinfo->tm_hour << "h" << timeinfo->tm_min - 18 << "min" << timeinfo->tm_sec << "s" << std::endl;

    portENTER_CRITICAL(&timerMux);
    flagTimeReq = false;
    timeReq = 0;
    portEXIT_CRITICAL(&timerMux);

}   