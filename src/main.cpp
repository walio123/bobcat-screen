#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <IWatchdog.h>
#include "AmpireHMI.h"
#include "watchdog_manager.h"
#include "EasyProfile.h"

// ========================== CONFIG ===============================
#define PULSES_PER_REV        90.0
#define UPDATE_MS             500
#define ENGINE_IDLE_RPM       450
#define OIL_GRACE_PERIOD      7000
#define OIL_FAIL_DELAY        1500
#define VOLT_GRACE_PERIOD     5000       
#define RPM_SIGNAL_TIMEOUT    2500       
#define EEPROM_ADDR           0x50
#define EEPROM_SAVE_PERIOD    300000     
#define HOURS_SAVE_THRESHOLD  0.1        
#define VOLT_DIVIDER_RATIO    7.25
#define SENSOR_TIMEOUT_MS     30000     
#define TEMP_FILTER_SIZE      5          
#define MIN_ALARM_TIME        2000      

#define OIL_MIN_V             0.5    
#define OIL_MAX_V             3.1    
#define OIL_V_RANGE           2.6    
#define OIL_ALARM_ON          0.7    
#define OIL_ALARM_OFF         1.0    

#define STM32_ADC_MAX         4095.0
#define STM32_VREF            3.3

// ========================== COLORS ===============================
#define C_BLACK   0x0000
#define C_CYAN    0x07FF
#define C_GREEN   0x07E0
#define C_YELLOW  0xFFE0
#define C_RED     0xF800

#define PIN_MODE_BUTTON  PB12

// ========================== OBJECTS & GLOBALS ====================
// 1. СЪСТОЯНИЯ (Enum се дефинира ПРЕДИ да се ползва)
enum SystemState { 
    SYS_BOOT, 
    SYS_IDLE, 
    SYS_ENGINE_STARTING, 
    SYS_ENGINE_RUNNING, 
    SYS_ALARM, 
    SYS_SENSOR_ERROR, 
    SYS_RPM_ERROR, 
    SYS_LEVELING 
};
enum StateEvent { EV_ENTER, EV_RUN, EV_EXIT };

// 2. Хардуерни портове
HardwareSerial SerialHMI(PA10, PA9);
HardwareSerial SerialSensor(PA3, PA2);

// 3. Хардуерни обекти
TwoWire Wire2(PB7, PB6);
Adafruit_ADS1115 ads;
AmpireHMI hmi(SerialHMI);
WatchdogManager wdm; 

EasyObjectDictionary eOD;     
EasyProfile eP(&eOD);         
Ep_Header epHeader;

// 4. Пинове
const int pinEngineRPM = PB8;
const int pinBuzzer = PB0;
const int pinOilAnalog = PA1;

// 5. СТРУКТУРИ (Чисти дефиниции)
struct EngineData {
    volatile uint32_t pulseCount = 0;
    int   waterTemp = 0;
    int   fuelLevel = 0;
    int   hydTemp = 0;
    float voltage = 0.0;
    float oilBar = 0.0;
    float totalHours = 0.0;
    int   rpm = 0;
};

struct AlarmState {
    bool oilLow = false;
    bool oilBroken = false;
    bool waterHigh = false;
    bool hydHigh = false;
    bool adsFault = false;
    bool rpmLost = false;
    bool voltLow = false;
    bool voltHigh = false;
};

struct LastDisplay {
    int   waterTemp = -1;
    int   fuelLevel = -1;
    int   hydTemp   = -1;
    int   rpm       = -1;
    float voltage   = -1.0;
    float oilBar    = -1.0;
};

// 6. ИНСТАНЦИИ (Създаване на обектите)
EngineData eng;
AlarmState alrm;
LastDisplay last;

SystemState sysState = SYS_BOOT;
SystemState prevState = SYS_BOOT;

// 7. МАКРОСИ
#define ENTER_STATE(s) {                         \
    handleState(sysState, EV_EXIT, millis());    \
    prevState = sysState;                        \
    sysState = (s);                              \
    stateEnterTime = millis();                   \
    handleState(sysState, EV_ENTER, millis());   \
}
// 8. Таймери и флагове
uint32_t lastUpdate = 0, stateEnterTime = 0, lastRpmPulseTime = 0, lastEepromSave = 0;
uint32_t oilFailTime = 0, rpmLossTime = 0, voltFailTime = 0;
uint32_t lastWaterOK = 0, lastHydOK = 0, alarmLatchTime = 0, lastButtonPress = 0;
float    lastSavedHours = 0.0, lastDisplayedHours = -1.0;
bool     firstRun = true, waterValid = false, hydValid = false;

float    waterBuffer[TEMP_FILTER_SIZE] = {0}, hydBuffer[TEMP_FILTER_SIZE] = {0};
uint8_t  waterIdx = 0, hydIdx = 0;

// Графика Leveler
float smoothRoll = 0, smoothPitch = 0, testAngle = 0;
uint16_t oldX = 320, oldY = 240;
int oldL = 0, oldR = 0;

// ========================== HELPERS ==============================
void countPulse() {
    static uint32_t lastMicros = 0;
    uint32_t nowU = micros();
    if (nowU - lastMicros > 180) { // Debounce
        eng.pulseCount++;
        lastMicros = nowU;
        lastRpmPulseTime = millis(); // ❗ Ето го сърцебиенето на мотора
    }
}

// ========================== RPM LOGIC ===============================
int getRPM(uint32_t now) {
    static uint32_t lastRpmCalc = 0;
    static int filteredRpm = 0;
    
    // Смятаме само ако са минали поне 100ms (10 пъти в секунда)
    uint32_t elapsed = now - lastRpmCalc;
    if (elapsed < 100) return filteredRpm;

    // Вземаме импулсите безопасно
    noInterrupts();
    long pulses = eng.pulseCount;
    eng.pulseCount = 0;
    interrupts();

    // 1. Изчисляваме суровия RPM спрямо реално изтеклото време
    // Формула: (импулси * 60 сек) / (зъби * време в сек)
    int rawRpm = (int)((pulses * 60000.0) / (PULSES_PER_REV * elapsed));

    // 2. Проверка за угаснал двигател (ако няма импулси над половин секунда)
    if (pulses == 0 && elapsed > 500) {
        rawRpm = 0;
    }

    // 3. МАШИНЕН ФИЛТЪР (Експоненциален)
    // Ако разликата е малка (<20), "заковаваме" цифрата, за да не трепти
    if (abs(rawRpm - filteredRpm) < 20) {
        filteredRpm = rawRpm;
    } else {
        // Ако разликата е голяма, местим плавно (20% от новата стойност)
        // Това убива "jitter"-а, но реагира на газта за около 1 сек.
        filteredRpm = (int)(filteredRpm * 0.8 + rawRpm * 0.2);
    }

    // 4. Sanity Check (Безопасност)
    if (filteredRpm > 4500) filteredRpm = 4500;
    if (filteredRpm < 0)    filteredRpm = 0;

    lastRpmCalc = now;
    return filteredRpm;
}

float adcToTemp(int16_t adc) {
    if (adc < 150 || adc > 26200) return -999.0; 
    const float adcMin = 800.0, adcMax = 26000.0;
    const float tMin = -40.0, tMax = 120.0; 
    float val = constrain((float)adc, adcMin, adcMax);
    return tMin + (val - adcMin) * (tMax - tMin) / (adcMax - adcMin);
}

float readTotalHours() {
    union { float f; byte b[4]; } d;
    Wire2.beginTransmission(EEPROM_ADDR); Wire2.write(0x00);
    if (Wire2.endTransmission() != 0) return 0.0;
    Wire2.requestFrom(EEPROM_ADDR, 4);
    for (int i = 0; i < 4; i++) if(Wire2.available()) d.b[i] = Wire2.read();
    return (isnan(d.f) || d.f < 0 || d.f > 100000) ? 0.0 : d.f;
}

void saveTotalHours(float h) {
    union { float f; byte b[4]; } d; d.f = h;
    Wire2.beginTransmission(EEPROM_ADDR); Wire2.write(0x00);
    for (int i = 0; i < 4; i++) Wire2.write(d.b[i]);
    Wire2.endTransmission(); delay(15); lastSavedHours = h;
}

// ========================== CORE LOGIC ===========================
// 1. Само чете данните
// --- 1. Четене на ADC (Само данни, без решения) ---
bool updateAnalogData() {
    Wire2.beginTransmission(0x48);
    if (Wire2.endTransmission() != 0) return false; // I2C грешка (ADS забил)

    eng.waterTemp = map(ads.readADC_SingleEnded(0), 0, 26000, 20, 120);
    eng.hydTemp   = map(ads.readADC_SingleEnded(2), 0, 26000, 20, 120);
    eng.fuelLevel = map(ads.readADC_SingleEnded(3), 0, 26000, 0, 100);
    eng.voltage   = (analogRead(PA0) * (3.3 / 4095.0)) * VOLT_DIVIDER_RATIO;
    return true;
}

// --- 2. Логика за Масло (с Hysteresis и Broken check) ---
void evaluateOil(uint32_t now) {
    int raw = ads.readADC_SingleEnded(1);
    float v = (raw * 0.1875) / 1000.0;

    if (v < 0.2 || v > 4.8) {
        alrm.oilBroken = true; alrm.oilLow = true; eng.oilBar = 0.0; return;
    }
    alrm.oilBroken = false;
    eng.oilBar = (v - OIL_MIN_V) * (5.0 / OIL_V_RANGE);
    if (eng.oilBar < 0) eng.oilBar = 0;

    if (eng.rpm > ENGINE_IDLE_RPM) {
        if (!alrm.oilLow && eng.oilBar < OIL_ALARM_ON) {
            if (oilFailTime == 0) oilFailTime = now;
            if (now - oilFailTime > OIL_FAIL_DELAY) alrm.oilLow = true;
        } else if (alrm.oilLow && eng.oilBar > OIL_ALARM_OFF) {
            alrm.oilLow = false; oilFailTime = 0;
        }
    } else { alrm.oilLow = false; oilFailTime = 0; }
}

// --- 3. Всички останали аларми на едно място ---
void evaluateSystemAlarms(uint32_t now, bool adsValid) {
    alrm.adsFault = !adsValid;
    // Проверка за загуба на RPM сигнал
    alrm.rpmLost = (now - lastRpmPulseTime > RPM_SIGNAL_TIMEOUT) && (sysState != SYS_IDLE);

    if (eng.rpm > ENGINE_IDLE_RPM) {
        // Вода Hysteresis: ON при 98, OFF при 92
        if (eng.waterTemp > 98) alrm.waterHigh = true;
        else if (eng.waterTemp < 92) alrm.waterHigh = false;

        // Хидравлика Hysteresis: ON при 90, OFF при 82
        if (eng.hydTemp > 90) alrm.hydHigh = true;
        else if (eng.hydTemp < 82) alrm.hydHigh = false;
    }
}

void updateDisplay(int w, int f, int h, float v, int rpm, bool adsValid, bool rpmLost) {
    char buf[24];
    if (!adsValid) { w = -1; h = -1; v = 0.0; }

    // Вода
    if (w != last.waterTemp || firstRun) {
        uint16_t c = (w < 0 || !waterValid) ? C_RED : (w < 40 ? C_CYAN : (w < 72 ? C_YELLOW : (w <= 95 ? C_GREEN : C_RED)));
        if (w < 0 || !waterValid) sprintf(buf,"ERR C"); else sprintf(buf,"%d C  ",w);
        hmi.HMI_Print(130, 130, buf, c, C_BLACK); 
        last.waterTemp = w;
    }
    // Гориво
    if (f != last.fuelLevel || firstRun) {
        sprintf(buf,"%d L  ", f); 
        hmi.HMI_Print(440, 130, buf, (f <= 10) ? C_RED : C_GREEN, C_BLACK); 
        last.fuelLevel = f;
    }
    // Хидравлика
    if (h != last.hydTemp || firstRun) {
        uint16_t c = (h < 0 || !hydValid) ? C_RED : (h < 30 ? C_CYAN : (h < 50 ? C_YELLOW : (h <= 85 ? C_GREEN : C_RED)));
        if (h < 0 || !hydValid) sprintf(buf,"ERR C"); else sprintf(buf,"%d C  ",h);
        hmi.HMI_Print(130, 265, buf, c, C_BLACK); 
        last.hydTemp = h;
    }
    // Волтаж
    if (abs(v - last.voltage) > 0.1 || firstRun) {
        if (v <= 0.1) sprintf(buf,"--.- V"); else sprintf(buf,"%d.%d V",(int)v,(int)(v*10)%10);
        hmi.HMI_Print(440, 265, buf, (v < 11.5 || v > 15.5) ? C_RED : C_GREEN, C_BLACK); 
        last.voltage = v;
    }
    // RPM
    if (rpm != last.rpm || firstRun || rpmLost) {
        if (rpmLost && sysState != SYS_IDLE) hmi.HMI_Print(50, 400, "RPM LOST", C_RED, C_BLACK);
        else { sprintf(buf, "%d RPM ", rpm); hmi.HMI_Print(50, 400, buf, (rpm > 2600) ? C_RED : C_GREEN, C_BLACK); }
        last.rpm = rpm;
    }
    // Масло
    if (abs(eng.oilBar - last.oilBar) > 0.1 || firstRun || alrm.oilBroken) {
        if (alrm.oilBroken) hmi.HMI_Print(470, 400, "SENS ERR", C_RED, C_BLACK);
        else { 
            sprintf(buf, "%d.%d BAR", (int)eng.oilBar, (int)(eng.oilBar * 10) % 10); 
            hmi.HMI_Print(470, 400, buf, (alrm.oilLow) ? C_RED : C_GREEN, C_BLACK); 
        }
        last.oilBar = eng.oilBar;
    }
    // Моточасове
    float dh = floor(eng.totalHours * 10) / 10.0;
    if (dh != lastDisplayedHours || firstRun) {
        sprintf(buf, "HRS: %d.%d", (int)eng.totalHours, (int)(eng.totalHours * 10) % 10);
        hmi.HMI_PrintHours(420, 20, buf, C_YELLOW, C_BLACK); 
        lastDisplayedHours = dh;
    }
    // Системни съобщения
    if (sysState == SYS_RPM_ERROR || sysState == SYS_SENSOR_ERROR) {
        hmi.HMI_PrintHours(220, 20, (sysState == SYS_RPM_ERROR) ? "[ RPM LOSS ]" : "[ SENS ERROR ]", C_RED, C_BLACK);
    } else if (firstRun || last.rpm != -1) {
        hmi.HMI_PrintHours(220, 20, "              ", C_BLACK, C_BLACK);
    }
}
void evaluateAlarms(uint32_t now, bool adsValid) {
    // 1. ADS се маркира винаги, но не сменя състояния тук
    alrm.adsFault = !adsValid;

    // 2. RPM ПРОВЕРКА - вече не е параноична
    // Валидна е само ако двигателят е обявен за работещ
    if (sysState == SYS_ENGINE_RUNNING || sysState == SYS_ALARM || sysState == SYS_ENGINE_STARTING) {
        alrm.rpmLost = (now - lastRpmPulseTime > RPM_SIGNAL_TIMEOUT);
    } else {
        alrm.rpmLost = false; // В IDLE/BOOT/LEVELING не ни пука за липса на импулси
    }

    // 3. ТЕРМОСТАТНА ЛОГИКА (Хистерезис) - само при работещ мотор
    if (eng.rpm > ENGINE_IDLE_RPM) {
        if (eng.waterTemp > 98) alrm.waterHigh = true;
        else if (eng.waterTemp < 92) alrm.waterHigh = false;

        if (eng.hydTemp > 90) alrm.hydHigh = true;
        else if (eng.hydTemp < 82) alrm.hydHigh = false;
    }
}
void updateLevelerGraphics(float rollDeg, float pitchDeg, float turretDeg) {
    static uint32_t lastDraw = 0;
    static bool first = true;
    static float sX = 0, sY = 0;
    static int oldL = 0, oldR = 0;
    static uint16_t oldX = 320, oldY = 240;

    // 1. АКО НЕ Е ВРЕМЕ ЗА РИСУВАНЕ, ИЗЛИЗАМЕ ВЕДНАГА
    if (millis() - lastDraw < 70) return;
    lastDraw = millis();

    // 2. МАТЕМАТИКАТА (Твоята ротация)
    float rad = turretDeg * 0.01745329f;
    float c = cosf(rad); float s = sinf(rad);
    float opX =  rollDeg * c + pitchDeg * s;   
    float opY = -rollDeg * s + pitchDeg * c;   
    sX += (opX - sX) * 0.25f;
    sY += (opY - sY) * 0.25f;

    // 3. ПОМОЩНА ФУНКЦИЯ ЗА ЧАКАНЕ (С ПРОВЕРКА НА БУТОНА)
    // Това е критично - ако PA8 е зает, пак да гледаме PB12
    auto waitDisplay = [&]() {
        uint32_t startWait = millis();
        while(digitalRead(PA8) == HIGH) {
            IWatchdog.reload();
            // Ако натиснеш бутона (PB12) за изход, докато чакаш дисплея - не блокирай!
            if (digitalRead(PB12) == LOW) return; 
            if (millis() - startWait > 100) break; // Safety timeout
        }
    };

    // 4. БАРОВЕ
    const float pxPerDeg = 6.0f;
    int L = (int)(sX * pxPerDeg);
    int R = -(int)(sX * pxPerDeg);
    L = constrain(L, -120, 120); R = constrain(R, -120, 120);

    if (L != oldL || R != oldR || first) {
        waitDisplay(); hmi.HMI_DrawBox(6, 240 - oldL - 10, 33, 240 - oldL + 10, C_BLACK, true);
        waitDisplay(); hmi.HMI_DrawBox(606, 240 - oldR - 10, 633, 240 - oldR + 10, C_BLACK, true);
        waitDisplay(); hmi.HMI_DrawBox(6, 240 - L - 10, 33, 240 - L + 10, C_CYAN, true);
        waitDisplay(); hmi.HMI_DrawBox(606, 240 - R - 10, 633, 240 - R + 10, C_CYAN, true);
        oldL = L; oldR = R;
    }

    // 5. ТОПЧЕ
    uint16_t nX = 320 + (int)(sX * 4.5f);
    uint16_t nY = 240 - (int)(sY * 4.5f);
    if (nX != oldX || nY != oldY || first) {
        waitDisplay(); hmi.HMI_DrawBox(oldX - 7, oldY - 7, oldX + 7, oldY + 7, C_BLACK, true);
        waitDisplay(); hmi.HMI_DrawBox(nX - 6, nY - 6, nX + 6, nY + 6, C_CYAN, true);
        oldX = nX; oldY = nY;
    }

    first = false;
}
void handleState(SystemState state, StateEvent ev, uint32_t now) {
    if (ev == EV_ENTER) {
        switch(state) {
            case SYS_BOOT:           hmi.HMI_ShowImage(0); break;
            case SYS_IDLE:           hmi.HMI_ShowImage(1); firstRun = true; break;
            case SYS_ENGINE_RUNNING: hmi.HMI_PrintHours(220, 20, "            ", C_BLACK, C_BLACK); break;
            case SYS_SENSOR_ERROR:   hmi.HMI_ShowImage(1); break; 
            case SYS_RPM_ERROR:      hmi.HMI_ShowImage(1); break; 
            case SYS_LEVELING:       hmi.HMI_ShowImage(2); break;
        }
    }
    if (ev == EV_RUN) {
        if (state == SYS_LEVELING) updateLevelerGraphics(smoothRoll, smoothPitch, testAngle);
    }
}
/// ========================== SETUP ===============================
void setup() {
    // 1. Пинове
    pinMode(PA8, INPUT);
    pinMode(PIN_MODE_BUTTON, INPUT_PULLUP);
    pinMode(pinBuzzer, OUTPUT); 
    digitalWrite(pinBuzzer, LOW); 
    pinMode(pinEngineRPM, INPUT_PULLUP);
    
    // 2. Дисплей - даваме му време
    hmi.begin(115200); 
    delay(3000); 
    //hmi.HMI_ShowImage(1); 

    // 3. Хардуер STM32
    analogReadResolution(12);
    attachInterrupt(digitalPinToInterrupt(pinEngineRPM), countPulse, FALLING);
    
    // 4. I2C Конфигурация
    // В setup()
Wire2.begin();
Wire2.setTimeout(100); // Това е правилното име за това ядро
Wire2.setClock(100000);
    // --- УМНА ИНИЦИАЛИЗАЦИЯ НА ADS1115 ---
    Wire2.beginTransmission(0x48);
    if (Wire2.endTransmission() == 0) {
        ads.begin(0x48, &Wire2); 
        ads.setGain(GAIN_ONE); 
        alrm.adsFault = false;
    } else {
        alrm.adsFault = true; // ADS липсва, но продължаваме напред!
    }
    
    // 5. ЕЕПРОМ
    Wire2.beginTransmission(EEPROM_ADDR);
    if (Wire2.endTransmission() == 0) {
        eng.totalHours = readTotalHours();
    } else {
        eng.totalHours = 0.0;
    }
    lastSavedHours = eng.totalHours;
    
    // 6. Сензор нивелация
    SerialSensor.begin(115200); 

    // 7. WATCHDOG - финално пускане
    wdm.begin(4000000); 
    wdm.setStateTimeout(SYS_ENGINE_STARTING, 30000);
    wdm.setStateTimeout(SYS_ALARM, 600000);
    wdm.setStateTimeout(SYS_SENSOR_ERROR, 300000);

    ENTER_STATE(SYS_BOOT); 
}
void loop() {
    // 1. КРИТИЧНО: update() реално презарежда хардуерния Watchdog (IWatchdog.reload())
    // Без него чипът се рестартира на всеки 4 секунди, независимо от feed().
    wdm.update((uint8_t)sysState); 
    wdm.feed(); 
    
    uint32_t now = millis();

    // 2. БУТОН - МОМЕНТАЛНА РЕАКЦИЯ
    if (digitalRead(PIN_MODE_BUTTON) == LOW) {
        if (now - lastButtonPress > 600) {
            lastButtonPress = now;
            if (sysState == SYS_LEVELING) { 
                ENTER_STATE(SYS_IDLE); 
            } else { 
                ENTER_STATE(SYS_LEVELING); 
            }
        }
    }

    // 3. ОСНОВЕН ЦИКЪЛ (500ms) - ЛОГИКА И ДАННИ
    if (now - lastUpdate >= UPDATE_MS) {
        lastUpdate = now;

        // --- ЧЕТЕНЕ НА ДАННИ ---
        eng.rpm = getRPM(now);
        eng.voltage = (analogRead(PA0) * (3.3 / 4095.0)) * VOLT_DIVIDER_RATIO;
        
        bool adsFound = updateAnalogData(); 
        if (adsFound) {
            evaluateOil(now);
            alrm.adsFault = false;
        } else {
            alrm.adsFault = true;
        }

        evaluateSystemAlarms(now, adsFound); 

        // --- STATE MACHINE (Управление на състоянията) ---
        switch(sysState) {
            case SYS_BOOT:
                if (now - stateEnterTime > 2000) { 
                    ENTER_STATE(SYS_IDLE); 
                }
                break;

            case SYS_IDLE:
                if (eng.rpm > ENGINE_IDLE_RPM && !alrm.rpmLost) { 
                    ENTER_STATE(SYS_ENGINE_STARTING); 
                }
                break;

            case SYS_ENGINE_STARTING:
                if (eng.rpm <= ENGINE_IDLE_RPM) { 
                    ENTER_STATE(SYS_IDLE); 
                } else if (now - stateEnterTime > OIL_GRACE_PERIOD) { 
                    ENTER_STATE(SYS_ENGINE_RUNNING); 
                }
                break;

            case SYS_ENGINE_RUNNING:
                if (alrm.oilLow || alrm.waterHigh || alrm.hydHigh) { 
                    ENTER_STATE(SYS_ALARM); 
                } else if (alrm.adsFault) { 
                    ENTER_STATE(SYS_SENSOR_ERROR); 
                } else if (alrm.rpmLost) { 
                    ENTER_STATE(SYS_RPM_ERROR); 
                }
                // Натрупване на моточасове
                eng.totalHours += (float)UPDATE_MS / 3600000.0;
                break;

            case SYS_ALARM:
                if (!alrm.oilLow && !alrm.waterHigh && !alrm.hydHigh && (now - stateEnterTime > MIN_ALARM_TIME)) {
                    ENTER_STATE(SYS_ENGINE_RUNNING);
                }
                if (eng.rpm <= ENGINE_IDLE_RPM) { 
                    ENTER_STATE(SYS_IDLE); 
                }
                break;

            case SYS_SENSOR_ERROR:
                if (!alrm.adsFault) { 
                    ENTER_STATE(SYS_IDLE); 
                }
                break;
                
            case SYS_RPM_ERROR:
                if (!alrm.rpmLost || eng.rpm <= ENGINE_IDLE_RPM) { 
                    ENTER_STATE(SYS_IDLE); 
                }
                break;

            default:
                break;
        }

        // --- ЕКРАН (Обновяване само ако не сме в BOOT или Нивелир) ---
        if (sysState != SYS_BOOT && sysState != SYS_LEVELING) {
            updateDisplay(eng.waterTemp, eng.fuelLevel, eng.hydTemp, eng.voltage, eng.rpm, !alrm.adsFault, alrm.rpmLost);
        }

        // --- EEPROM ЗАЩИТА ---
        if ((now - lastEepromSave >= EEPROM_SAVE_PERIOD) || (prevState == SYS_ENGINE_RUNNING && sysState == SYS_IDLE)) {
            saveTotalHours(eng.totalHours);
            lastEepromSave = now;
            // Презареждаме пак след бавния запис
            wdm.update((uint8_t)sysState);
        }
        
        firstRun = false;
    }

    // 4. ГРАФИКИ (Нивелирът се нуждае от максимална скорост)
    handleState(sysState, EV_RUN, now);
}