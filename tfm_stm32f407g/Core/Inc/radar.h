/**
 * @file    radar.h
 * @brief   API pública del módulo de radar redundante (nodo Esclavo).
 */

#ifndef RADAR_H
#define RADAR_H

#include "FreeRTOS.h"

BaseType_t Radar_Init(void);

void vRadarTask(void *argument);

/**
 * @brief  ISR de TIM2 — captura los flancos del pulso de eco.
 * NOTA: Asegúrate de desmarcar "Generate IRQ handler" en CubeMX
 * para que esta función actúe como el vector nativo.
 */
void TIM2_IRQHandler(void);

#endif /* RADAR_H */