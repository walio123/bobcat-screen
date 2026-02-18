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

// ТУК СА ДЕФИНИЦИИТЕ НА ПОРТОВЕТЕ
HardwareSerial SerialHMI(PA10, PA9);
HardwareSerial SerialSensor(PA3, PA2);

enum SystemState { SYS_BOOT, SYS_IDLE, SYS_ENGINE_STARTING, SYS_ENGINE_RUNNING, SYS_ALARM, SYS_SENSOR_ERROR, SYS_RPM_ERROR, SYS_LEVELING };
SystemState sysState = SYS_BOOT;
SystemState prevState = SYS_BOOT;

#define ENTER_STATE(s) do { \
    prevState = sysState; \
    sysState = s; \
    stateEnterTime = millis(); \
    oilFailTime = 0; \
    alarmLatchTime = 0; \
    firstRun = true; \
} while(0)

#define PIN_MODE_BUTTON  PB12

// ========================== OBJECTS & GLOBALS ====================
float smoothRoll = 0, smoothPitch = 0, testAngle = 0;
uint16_t oldX = 320, oldY = 240;
int oldL = 0, oldR = 0;

EasyObjectDictionary eOD;     
EasyProfile eP(&eOD);         
Ep_Header epHeader;           

TwoWire Wire2(PB7, PB6);
Adafruit_ADS1115 ads;
AmpireHMI hmi(SerialHMI);
WatchdogManager wdm; 

const int pinEngineRPM = PB8;
const int pinBuzzer = PB0;
const int pinOilAnalog = PA1;

volatile uint32_t pulseCount = 0;
uint32_t lastUpdate = 0, stateEnterTime = 0, lastRpmPulseTime = 0, lastEepromSave = 0, oilFailTime = 0, rpmLossTime = 0;
uint32_t lastWaterOK = 0, lastHydOK = 0, alarmLatchTime = 0;
uint32_t lastButtonPress = 0;
float totalHours = 0.0, lastSavedHours = 0.0, lastDisplayedHours = -1.0;
bool firstRun = true, oilSensorBroken = false, oilLow = false;
int lastWater = -1, lastFuel = -1, lastHyd = -1, lastRpm = -1;
float lastVolts = -1.0, lastOilBar = -1.0, oilBar = 0.0;
int waterTemp = 0, fuelLevel = 0, hydTemp = 0; float voltage = 0.0;
bool waterValid = false, hydValid = false, waterAlarmActive = false, hydAlarmActive = false;

float waterBuffer[TEMP_FILTER_SIZE] = {0}, hydBuffer[TEMP_FILTER_SIZE] = {0};
uint8_t waterIdx = 0, hydIdx = 0;

// ========================== HELPERS ==============================
void countPulse() {
    static uint32_t lastMicros = 0;
    uint32_t now = micros();
    if (now - lastMicros > 180) { pulseCount++; lastMicros = now; }
}

int getRPM(uint32_t now, bool &signalLost) {
    noInterrupts(); uint32_t pulses = pulseCount; pulseCount = 0; interrupts();
    if (pulses > 0) lastRpmPulseTime = now;
    signalLost = (now - lastRpmPulseTime > RPM_SIGNAL_TIMEOUT);
    return signalLost ? 0 : (int)((pulses * 60000.0) / (PULSES_PER_REV * UPDATE_MS));
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
void readOilPressure(bool &alarm, int rpm) {
    float vOil = (analogRead(pinOilAnalog) / STM32_ADC_MAX) * STM32_VREF;
    if (vOil < 0.2 || vOil > (STM32_VREF - 0.1)) {
        oilSensorBroken = true; oilBar = 0.0; alarm = true; 
    } else {
        oilSensorBroken = false;
        oilBar = (vOil - OIL_MIN_V) * (10.0 / OIL_V_RANGE);
        if (oilBar < 0) oilBar = 0;
        if (rpm > ENGINE_IDLE_RPM) {
            if (!oilLow && oilBar < OIL_ALARM_ON) oilLow = true;
            else if (oilLow && oilBar > OIL_ALARM_OFF) oilLow = false;
        } else oilLow = false;
        if (oilLow) alarm = true;
    }
}

bool readSensors(int &w, int &f, int &h, float &v, bool &alarm, bool voltageAllowed, int rpm, uint32_t now) {
    alarm = false; 
    Wire2.beginTransmission(0x48);
    if (Wire2.endTransmission() != 0) return false;

    int16_t rW = ads.readADC_SingleEnded(0), rF = ads.readADC_SingleEnded(1);
    int16_t rH = ads.readADC_SingleEnded(2), rV = ads.readADC_SingleEnded(3);

    static int16_t lastRV = 0; static uint32_t sCnt = 0;
    if (abs(rV - lastRV) < 2 && rpm > ENGINE_IDLE_RPM) {
        if (++sCnt > 100) return false; 
    } else sCnt = 0;
    lastRV = rV;

    float tW = adcToTemp(rW);
    if (tW > -900.0) {
        waterValid = true; lastWaterOK = now;
        waterBuffer[waterIdx++] = tW; if (waterIdx >= TEMP_FILTER_SIZE) waterIdx = 0;
        float wSum = 0; for(int i=0; i<TEMP_FILTER_SIZE; i++) wSum += waterBuffer[i];
        w = (int)(wSum / TEMP_FILTER_SIZE);
        if (rpm > ENGINE_IDLE_RPM && w > 95) waterAlarmActive = true; 
        else if (w < 90) waterAlarmActive = false;
    } else if (now - lastWaterOK > SENSOR_TIMEOUT_MS) { waterValid = false; alarm = true; }

    float tH = adcToTemp(rH);
    if (tH > -900.0) {
        hydValid = true; lastHydOK = now;
        hydBuffer[hydIdx++] = tH; if (hydIdx >= TEMP_FILTER_SIZE) hydIdx = 0;
        float hSum = 0; for(int i=0; i<TEMP_FILTER_SIZE; i++) hSum += hydBuffer[i];
        h = (int)(hSum / TEMP_FILTER_SIZE);
        if (rpm > ENGINE_IDLE_RPM && h > 85) hydAlarmActive = true; 
        else if (h < 80) hydAlarmActive = false;
    } else if (now - lastHydOK > SENSOR_TIMEOUT_MS) { hydValid = false; alarm = true; }

    f = constrain(map(rF, 500, 20000, 0, 60), 0, 60);
    v = ads.computeVolts(rV) * VOLT_DIVIDER_RATIO;
    if (waterAlarmActive || hydAlarmActive) alarm = true;
    if (voltageAllowed && (v < 11.5 || v > 15.5)) alarm = true; 
    return true;
}

void updateDisplay(int w, int f, int h, float v, int rpm, bool adsValid, bool rpmLost) {
    char buf[24];
    if (!adsValid) { w = -1; h = -1; v = 0.0; }

    if (w != lastWater || firstRun) {
        uint16_t c = (w < 0 || !waterValid) ? C_RED : (w < 40 ? C_CYAN : (w < 72 ? C_YELLOW : (w <= 95 ? C_GREEN : C_RED)));
        if (w < 0 || !waterValid) sprintf(buf,"ERR C"); else sprintf(buf,"%d C  ",w);
        hmi.HMI_Print(130,130,buf,c,C_BLACK); lastWater = w;
    }
    if (f != lastFuel || firstRun) {
        sprintf(buf,"%d L  ", f); hmi.HMI_Print(440,130,buf, (f <= 10) ? C_RED : C_GREEN, C_BLACK); lastFuel = f;
    }
    if (h != lastHyd || firstRun) {
        uint16_t c = (h < 0 || !hydValid) ? C_RED : (h < 30 ? C_CYAN : (h < 50 ? C_YELLOW : (h <= 85 ? C_GREEN : C_RED)));
        if (h < 0 || !hydValid) sprintf(buf,"ERR C"); else sprintf(buf,"%d C  ",h);
        hmi.HMI_Print(130,265,buf,c,C_BLACK); lastHyd = h;
    }
    if (abs(v - lastVolts) > 0.1 || firstRun) {
        if (v <= 0.1) sprintf(buf,"--.- V"); else sprintf(buf,"%d.%d V",(int)v,(int)(v*10)%10);
        hmi.HMI_Print(440,265,buf,(v < 11.5 || v > 15.5) ? C_RED : C_GREEN, C_BLACK); lastVolts = v;
    }
    if (rpm != lastRpm || firstRun || rpmLost) {
        if (rpmLost && sysState != SYS_IDLE) hmi.HMI_Print(50,400,"RPM LOST",C_RED,C_BLACK);
        else { sprintf(buf,"%d RPM ", rpm); hmi.HMI_Print(50,400,buf, (rpm > 2600) ? C_RED : C_GREEN, C_BLACK); }
        lastRpm = rpm;
    }
    if (abs(oilBar - lastOilBar) > 0.1 || firstRun || oilSensorBroken) {
        if (oilSensorBroken) hmi.HMI_Print(470,400,"SENS ERR",C_RED,C_BLACK);
        else { sprintf(buf, "%d.%d BAR", (int)oilBar, (int)(oilBar * 10) % 10); hmi.HMI_Print(470,400,buf, (oilLow) ? C_RED : C_GREEN, C_BLACK); }
        lastOilBar = oilBar;
    }
    float dh = floor(totalHours * 10) / 10.0;
    if (dh != lastDisplayedHours || firstRun) {
        sprintf(buf,"HRS: %d.%d",(int)totalHours,(int)(totalHours*10)%10);
        hmi.HMI_PrintHours(420,20,buf,C_YELLOW,C_BLACK); lastDisplayedHours = dh;
    }
    if (sysState == SYS_RPM_ERROR || sysState == SYS_SENSOR_ERROR) {
        hmi.HMI_PrintHours(220, 20, (sysState == SYS_RPM_ERROR) ? "[ RPM LOSS ]" : "[ SENS ERROR ]", C_RED, C_BLACK);
    } else if (firstRun || lastRpm != -1) hmi.HMI_PrintHours(220, 20, "              ", C_BLACK, C_BLACK);
}

void updateLevelerGraphics() {
    static uint32_t lastDraw = 0;
    static bool first = true; 
    static float lastVRoll = 0.0f; 
    static float smoothL = 0.0f, smoothR = 0.0f;
    static float smoothX = 320.0f, smoothY = 240.0f;

    if (millis() - lastDraw < 50) return; 
    lastDraw = millis();

    float angleRad = testAngle * (3.14159265f / 180.0f);
    float cosA = cosf(angleRad);
    float sinA = sinf(angleRad);
    float vRoll = smoothRoll * cosA - smoothPitch * sinA;

    if (fabs(vRoll) < 0.2f) vRoll = 0.0f;
    float stepFactor = constrain(fabs(vRoll - lastVRoll), 0.0f, 1.0f);

    if (fabs(vRoll - lastVRoll) < 0.5f) {
        vRoll = lastVRoll;
    } else {
        lastVRoll = vRoll;
    }

    float maxStep = 2.0f + stepFactor * 8.0f; 
    float targetL = -vRoll * 5.0f;
    float targetR =  vRoll * 5.0f;

    smoothL += constrain(targetL - smoothL, -maxStep, maxStep);
    smoothR += constrain(targetR - smoothR, -maxStep, maxStep);

    int L = (int)smoothL;
    int R = (int)smoothR;

    if (first) { oldL = L; oldR = R; }

    if (L != oldL || R != oldR) {
        hmi.HMI_DrawBox(6,   240 - oldL - 5, 33,  240 - oldL + 5, C_BLACK, true);
        hmi.HMI_DrawBox(606, 240 - oldR - 5, 633, 240 - oldR + 5, C_BLACK, true);
        hmi.HMI_DrawBox(6,   240 - L - 5, 33,  240 - L + 5, C_CYAN, true);
        hmi.HMI_DrawBox(606, 240 - R - 5, 633, 240 - R + 5, C_CYAN, true);
        oldL = L; oldR = R;
    }

    float targetX = 320.0f + 212.0f * sinf(angleRad);
    float targetY = 240.0f - 212.0f * cosf(angleRad);

    float maxStepBall = 3.0f + stepFactor * 5.0f; 
    smoothX += constrain(targetX - smoothX, -maxStepBall, maxStepBall);
    smoothY += constrain(targetY - smoothY, -maxStepBall, maxStepBall);

    uint16_t nX = (uint16_t)smoothX;
    uint16_t nY = (uint16_t)smoothY;

    if (first) { oldX = nX; oldY = nY; }

    if (nX != oldX || nY != oldY) {
        hmi.HMI_DrawBox(oldX - 6, oldY - 6, oldX + 6, oldY + 6, C_BLACK, true);
        hmi.HMI_DrawBox(nX  - 5, nY  - 5, nX  + 5, nY  + 5, C_CYAN,  true);
        oldX = nX; oldY = nY;
    }
    first = false; 
}

// ========================== SETUP ===============================
void setup() {
    pinMode(PIN_MODE_BUTTON, INPUT_PULLUP);
    pinMode(pinBuzzer, OUTPUT); digitalWrite(pinBuzzer, LOW);
    pinMode(pinEngineRPM, INPUT_PULLUP);
    
    hmi.begin(115200); 
    delay(3000); 
    hmi.HMI_ShowImage(1); 

    analogReadResolution(12);
    attachInterrupt(digitalPinToInterrupt(pinEngineRPM), countPulse, FALLING);
    
    Wire2.begin(); 
    Wire2.setClock(100000);

    if (ads.begin(0x48, &Wire2)) {
        ads.setGain(GAIN_ONE);
    } 
    
    totalHours = readTotalHours(); 
    lastSavedHours = totalHours;
    
    SerialSensor.begin(115200); 

    wdm.begin(4000000); 
    wdm.setStateTimeout(SYS_ENGINE_STARTING, 30000);
    wdm.setStateTimeout(SYS_ALARM, 600000);
    wdm.setStateTimeout(SYS_SENSOR_ERROR, 300000);

    ENTER_STATE(SYS_BOOT);
}

// ========================== LOOP ================================
void loop() {
    wdm.feed();
    wdm.update((uint8_t)sysState);
    uint32_t now = millis();

    if (digitalRead(PB12) == LOW && (now - lastButtonPress > 600)) {
        lastButtonPress = now;
        if (sysState == SYS_LEVELING) {
            hmi.HMI_ShowImage(1); 
            ENTER_STATE(SYS_IDLE); 
        } else {
            hmi.HMI_ShowImage(2); 
            oldX = 320; oldY = 240; 
            ENTER_STATE(SYS_LEVELING);
        }
    }

    while (SerialSensor.available()) {
        uint8_t c = SerialSensor.read();
        if (eP.On_RecvPkg((char*)&c, 1, &epHeader) == EP_SUCC_) {
            if (epHeader.cmd == EP_CMD_RPY_) {
                Ep_RPY rpy;
                if (eOD.Read_Ep_RPY(&rpy) == EP_SUCC_) {
                    smoothRoll = rpy.roll;
                    smoothPitch = rpy.pitch;
                }
            }
        }
    }

    if (now - lastUpdate >= UPDATE_MS) {
        lastUpdate = now;

        bool rLost = false; int rpm = getRPM(now, rLost);
        bool oAlarm = false; readOilPressure(oAlarm, rpm);
        bool sAlarm = false;
        bool vCheck = (sysState == SYS_ENGINE_RUNNING && (now - stateEnterTime > VOLT_GRACE_PERIOD));
        bool adsV = readSensors(waterTemp, fuelLevel, hydTemp, voltage, sAlarm, vCheck, rpm, now);

        switch(sysState) {
            case SYS_BOOT:
                if (now - stateEnterTime > 2000) ENTER_STATE(SYS_IDLE);
                break;

            case SYS_IDLE: 
                digitalWrite(pinBuzzer, LOW);
                if (rpm > ENGINE_IDLE_RPM && !rLost) ENTER_STATE(SYS_ENGINE_STARTING); 
                break;

            case SYS_ENGINE_STARTING: 
                if (rpm <= ENGINE_IDLE_RPM && !rLost) ENTER_STATE(SYS_IDLE);
                else if (now - stateEnterTime > OIL_GRACE_PERIOD) ENTER_STATE(SYS_ENGINE_RUNNING);
                break;

            case SYS_ENGINE_RUNNING:
                if (!adsV) { ENTER_STATE(SYS_SENSOR_ERROR); break; }
                if (rpm <= ENGINE_IDLE_RPM && !rLost) { ENTER_STATE(SYS_IDLE); break; }
                
                if (oAlarm) { 
                    if (!oilFailTime) oilFailTime = now; 
                    if (now - oilFailTime > OIL_FAIL_DELAY) sAlarm = true; 
                } else oilFailTime = 0;
                
                if (sAlarm) ENTER_STATE(SYS_ALARM);
                
                totalHours += (float)UPDATE_MS / 3600000.0;
                break;

            case SYS_LEVELING:
                if (oAlarm || sAlarm) { 
                    digitalWrite(pinBuzzer, HIGH); 
                } else {
                    digitalWrite(pinBuzzer, LOW);
                }
                updateLevelerGraphics(); 
                totalHours += (float)UPDATE_MS / 3600000.0;
                break;

            case SYS_ALARM:
                digitalWrite(pinBuzzer, HIGH);
                if (firstRun) alarmLatchTime = now;
                if (!sAlarm && !oAlarm && (now - alarmLatchTime > MIN_ALARM_TIME)) { 
                    digitalWrite(pinBuzzer, LOW); 
                    ENTER_STATE(rpm > ENGINE_IDLE_RPM ? SYS_ENGINE_RUNNING : SYS_IDLE); 
                }
                break;

            case SYS_SENSOR_ERROR:
            case SYS_RPM_ERROR:
                digitalWrite(pinBuzzer, HIGH);
                if (adsV && !rLost) { 
                    digitalWrite(pinBuzzer, LOW); 
                    ENTER_STATE(rpm > ENGINE_IDLE_RPM ? SYS_ENGINE_RUNNING : SYS_IDLE); 
                }
                break;
        }

        if (sysState != SYS_LEVELING && sysState != SYS_BOOT) {
            updateDisplay(waterTemp, fuelLevel, hydTemp, voltage, rpm, adsV, rLost);
        }

        firstRun = false;
    }
}