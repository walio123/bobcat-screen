#ifndef AMPIRE_HMI_H
#define AMPIRE_HMI_H

#include <Arduino.h>

// Структура за лесно управление на бутони
struct TouchButton {
    uint16_t x1, y1, x2, y2;
    const char* label;
    bool state;
};

class AmpireHMI {
public:
    AmpireHMI(HardwareSerial& serial);
    void begin(unsigned long baud = 115200);

    // Оригинални функции
    void HMI_SetColors(uint16_t f, uint16_t b);
    void HMI_DrawBox(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, bool fill);
    void HMI_Print(uint16_t x, uint16_t y, const char* str, uint16_t f, uint16_t b);
    
    void HMI_PrintHours(uint16_t x, uint16_t y, const char* str, uint16_t f, uint16_t b);
    void HMI_Screenshot(uint8_t picNum);
    void HMI_ShowImage(uint8_t picNum);

    // НОВИ ФУНКЦИИ ЗА ТЪЧ
    void drawButton(TouchButton& btn, uint16_t colorOn, uint16_t colorOff);
    bool isPressed(uint16_t tx, uint16_t ty, TouchButton& btn);

private:
    HardwareSerial& _port;
    const uint8_t _footer[4] = {0xCC, 0x33, 0xC3, 0x3C};
    void sendFooter();
};

#endif