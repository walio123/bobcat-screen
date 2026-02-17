#include "AmpireHMI.h"

AmpireHMI::AmpireHMI(HardwareSerial& serial) : _port(serial) {}

void AmpireHMI::begin(unsigned long baud) {
    _port.begin(baud);
}

// Изпращане на задължителния край на всяка команда
void AmpireHMI::sendFooter() {
    _port.write(_footer, 4); // 0xCC, 0x33, 0xC3, 0x3C [cite: 30]
}

// Задаване на цветове (Преден и Фон) - Команда 0x40 [cite: 40]
void AmpireHMI::HMI_SetColors(uint16_t f, uint16_t b) {
    uint8_t buf[] = {0xAA, 0x40, (uint8_t)(f >> 8), (uint8_t)f, (uint8_t)(b >> 8), (uint8_t)b};
    _port.write(buf, 6);
    sendFooter();
}

// Чертане на кутия/бутон - Използва 0x5D за запълване (Full Area) 
void AmpireHMI::HMI_DrawBox(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, bool fill) {
    HMI_SetColors(color, color);
    // 0x5D е "Full area with Front color", 0x5B е "Rectangle Front color" (рамка) 
    uint8_t cmd = fill ? 0x5D : 0x5B; 
    uint8_t buf[] = {0xAA, cmd, (uint8_t)(x1 >> 8), (uint8_t)x1, (uint8_t)(y1 >> 8), (uint8_t)y1,
                                (uint8_t)(x2 >> 8), (uint8_t)x2, (uint8_t)(y2 >> 8), (uint8_t)y2};
    _port.write(buf, 10);
    sendFooter();
}

// Изписване на текст (стандартен 16x16) - Команда 0x54 (команда 0x6F за 24х24)  и команда 0х55 за 32х32
void AmpireHMI::HMI_Print(uint16_t x, uint16_t y, const char* str, uint16_t f, uint16_t b) {
    HMI_SetColors(f, b);
    uint8_t head[] = {0xAA, 0x55, (uint8_t)(x >> 8), (uint8_t)x, (uint8_t)(y >> 8), (uint8_t)y};
    _port.write(head, 6);
    _port.print(str);
    sendFooter();
}
void AmpireHMI::HMI_PrintHours(uint16_t x, uint16_t y, const char* str, uint16_t f, uint16_t b) {
    HMI_SetColors(f, b);
    uint8_t head[] = {0xAA, 0x54, (uint8_t)(x >> 8), (uint8_t)x, (uint8_t)(y >> 8), (uint8_t)y};
    _port.write(head, 6);
    _port.print(str);
    sendFooter();
}

void AmpireHMI::HMI_Screenshot(uint8_t picNum) {
    uint8_t cmd[] = {0xAA, 0xE2, picNum}; _port.write(cmd, 3); sendFooter();
}

void AmpireHMI::HMI_ShowImage(uint8_t picNum) {
    uint8_t cmd[] = {0xAA, 0x70, picNum}; _port.write(cmd, 3); sendFooter();
}