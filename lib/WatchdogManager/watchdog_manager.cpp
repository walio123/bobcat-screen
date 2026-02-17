#include "watchdog_manager.h"

void WatchdogManager::begin(uint32_t timeout_us) {
    IWatchdog.begin(timeout_us);
}

void WatchdogManager::feed() {
    logicTick++;
}

void WatchdogManager::setStateTimeout(uint8_t state, uint32_t timeout_ms) {
    if (state < 10) stateTimeouts[state] = timeout_ms;
}

void WatchdogManager::update(uint8_t state) {
    uint32_t now = millis();

    // 1. Следене на смяна на състоянията
    if (state != currentState) {
        currentState = state;
        stateEnterTime = now;
    }

    // 2. Проверка за "зависване" в определен state (напр. постоянен ALARM)
    if (currentState < 10 && stateTimeouts[currentState] > 0) {
        if (now - stateEnterTime > stateTimeouts[currentState]) {
            fatal(); // Тайм-аут на състоянието -> рестарт
        }
    }

    // 3. Проверка дали loop() реално се върти (Freeze detection)
    if (now - lastLogicCheck > 3000) { // Проверка на всеки 3 секунди
        if (logicTick == lastLogicTick) {
            fatal(); // Логиката е замръзнала -> рестарт
        }
        lastLogicTick = logicTick;
        lastLogicCheck = now;
    }

    // Ако всичко е наред, "храним" хардуерния watchdog
    IWatchdog.reload();
}

void WatchdogManager::fatal() {
    // Влиза в безкраен цикъл без reload. 
    // Хардуерният Watchdog ще рестартира чипа след изтичане на timeout_us.
    while (1); 
}
