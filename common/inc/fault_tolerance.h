#ifndef FAULT_TOLERANCE_H_
#define FAULT_TOLERANCE_H_

#include "can_protocol.h"
#include <stdint.h>

/* ========================================================================= */
/* CÓDIGOS DE ESTADO DEL ALGORITMO                                           */
/* ========================================================================= */
typedef enum {
    FT_OK = 0,                 // Trama correcta y en orden
    FT_ERR_CRC_FAILED = -1,    // Fallo de integridad: los datos se han corrompido
    FT_ERR_FRAME_LOST = -2,    // Salto de secuencia: se han perdido tramas en el bus
    FT_ERR_DUPLICATE  = -3     // Trama duplicada o desordenada (antigua)
} ft_status_t;

/* ========================================================================= */
/* CONTEXTO DE TOLERANCIA A FALLOS                                           */
/* Guarda el estado actual de una conexión (Ej: del Máster hacia el Esclavo) */
/* ========================================================================= */
typedef struct {
    uint8_t tx_seq_num;      // Siguiente número de secuencia que VAMOS a enviar
    uint8_t rx_expected_seq; // Siguiente número de secuencia que ESPERAMOS recibir

    // Contadores estadísticos (¡Ideales para mostrarlos en el display del Máster!)
    uint32_t stats_frames_lost;
    uint32_t stats_crc_errors;
    uint32_t stats_frames_ok;
} ft_context_t;

/* ========================================================================= */
/* PROTOTIPOS DE FUNCIONES                                                   */
/* ========================================================================= */

// Inicializa a cero los contadores de un contexto
void ft_init_context(ft_context_t *ctx);

// Prepara una trama para enviarla (Añade Secuencia y calcula CRC)
ft_status_t ft_prepare_tx_frame(can_frame_payload_t *frame, ft_context_t *ctx);

// Verifica una trama recién recibida (Comprueba CRC y Secuencia)
ft_status_t ft_verify_rx_frame(const can_frame_payload_t *frame, ft_context_t *ctx);

#endif /* FAULT_TOLERANCE_H_ */
