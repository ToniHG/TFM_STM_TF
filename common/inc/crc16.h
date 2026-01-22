#ifndef CRC16_H_
#define CRC16_H_

#include <stdint.h>
#include <stddef.h>

/**
 * @brief  Calcula el CRC16 (CCITT) de un buffer de datos.
 * @param  data: Puntero al array de bytes que se va a evaluar.
 * @param  length: Número de bytes a evaluar.
 * @retval Valor de 16 bits correspondiente al CRC calculado.
 */
uint16_t calculate_crc16(const uint8_t *data, size_t length);

#endif /* CRC16_H_ */
