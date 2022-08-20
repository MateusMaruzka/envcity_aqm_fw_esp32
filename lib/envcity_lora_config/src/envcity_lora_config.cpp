#include "envcity_lora_config.hpp"


void init_lora(osjob_t *sendjob, u4_t DEVADDR, u1_t *NWKSKEY, u1_t *APPSKEY){

    // LMIC init
    os_init();
    //os_init_ex(&lmic_pins);
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);

    #if defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
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
    LMIC_setDrTxpow(DR_SF12,14);

    // Start job
    //do_send(&sendjob);
    do_send(sendjob);
}


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
            std::cout << "EV_JOINED"<< std::endl;
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