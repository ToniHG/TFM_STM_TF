#include "crc16.h"

/* * Implementación de CRC16-CCITT
 * Polinomio: 0x1021 (x^16 + x^12 + x^5 + 1)
 * Valor inicial: 0xFFFF
 */
uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF; // Valor inicial estándar para CCITT

    for (size_t i = 0; i < length; i++) {
        // Hacemos XOR del byte actual (desplazado a la parte alta) con el CRC
        crc ^= (uint16_t)(data[i] << 8);

        // Procesamos los 8 bits del byte actual
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                // Si el bit más significativo es 1, desplazamos y aplicamos XOR con el polinomio
                crc = (crc << 1) ^ 0x1021;
            } else {
                // Si es 0, solo desplazamos
                crc = (crc << 1);
            }
        }
    }

    return crc;
}
