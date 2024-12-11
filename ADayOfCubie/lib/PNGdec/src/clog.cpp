#include <Arduino.h>
extern "C"{
    #include "clog.h"
}

void aclog(const char * msg){
    Serial.print(msg);
    yield();
    delay(1);
}

void aclogN(const char * msg, int n){
    Serial.printf(msg, n);
    yield();
    delay(1);
}