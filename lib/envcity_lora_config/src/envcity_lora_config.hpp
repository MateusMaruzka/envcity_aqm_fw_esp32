#ifndef ENVCITY_LORA_CONFIG
#define ENVCITY_LORA_CONFIG

#include <Arduino.h>
#include <iostream>
#include "lmic.h"
#include "time.h"
#include "sys/time.h"

//extern void do_send(osjob_t* j);
extern osjob_t sendjob;
extern osjob_t sendVoltjJob;

extern void do_send(osjob_t* j);
extern void sendVoltageCallback(osjob_t *job);
 

extern volatile bool flagTimeReq;
extern volatile uint32_t timeReq;
extern portMUX_TYPE timerMux;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).

#define TX_INTERVAL 60


void onEvent (ev_t ev);

void init_lora(osjob_t *sendjob, u4_t DEVADDR, u1_t *NWKSKEY, u1_t *APPSKEY, u1_t subBand);

void requestNetworkTimeCallback(void *ptrUTCtime, int flagSuccess);


#endif /* ENVCITY_LORA_CONFIG */

