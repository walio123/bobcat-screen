#pragma once
#include <Arduino.h>
#include <IWatchdog.h>

class WatchdogManager {
public:
    void begin(uint32_t timeout_us);
    void feed();                      // heartbeat (казваш "жив съм")
    void update(uint8_t state);       // основна проверка на логиката
    void setStateTimeout(uint8_t state, uint32_t timeout_ms);

private:
    uint32_t lastLogicTick = 0;
    uint32_t logicTick = 0;
    uint32_t lastLogicCheck = 0;
    uint32_t stateEnterTime = 0;
    uint8_t  currentState = 0xFF;
    uint32_t stateTimeouts[10] = {0}; 

    void fatal();
};
