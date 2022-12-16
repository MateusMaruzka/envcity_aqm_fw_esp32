#ifndef ENVCITY_LORA_CONFIG
#define ENVCITY_LORA_CONFIG

#include <Arduino.h>
#include <iostream>
#include "lmic.h"
#include "time.h"
#include "sys/time.h"

//extern void do_send(osjob_t* j);
extern osjob_t sendjob;

extern void do_send(osjob_t* j);

extern volatile bool flagTimeReq;
extern volatile uint32_t timeReq;
extern portMUX_TYPE timerMux;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60*5;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
//void os_getArtEui (u1_t* buf) { }
//void os_getDevEui (u1_t* buf) { }
//void os_getDevKey (u1_t* buf) { }
/*
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
*/

void onEvent (ev_t ev);

void init_lora(osjob_t *sendjob, u4_t DEVADDR, u1_t *NWKSKEY, u1_t *APPSKEY, u1_t subBand);

void requestNetworkTimeCallback(void *ptrUTCtime, int flagSuccess);


#endif /* ENVCITY_LORA_CONFIG */

