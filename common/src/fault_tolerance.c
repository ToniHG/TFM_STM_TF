#include "fault_tolerance.h"
#include "crc16.h"
#include <stddef.h> // Para la macro offsetof()

void ft_init_context(ft_context_t *ctx) {
    ctx->tx_seq_num = 0;
    ctx->rx_expected_seq = 0;
    ctx->stats_frames_lost = 0;
    ctx->stats_crc_errors = 0;
    ctx->stats_frames_ok = 0;
}

ft_status_t ft_prepare_tx_frame(can_frame_payload_t *frame, ft_context_t *ctx) {
    // 1. Asignar el número de secuencia actual y preparar el siguiente (Algoritmo 1)
    frame->seq_number = ctx->tx_seq_num;
    ctx->tx_seq_num++; // Al llegar a 255, volverá a 0 automáticamente (uint8_t)

    // 2. Calcular los bytes útiles (todo lo que hay antes del campo crc16)
    size_t data_length = sizeof(can_frame_payload_t) - sizeof(uint16_t);

    // 3. Calcular e insertar el CRC (Algoritmo 2)
    frame->crc16 = calculate_crc16((const uint8_t *)frame, data_length);

    return FT_OK;
}

ft_status_t ft_verify_rx_frame(const can_frame_payload_t *frame, ft_context_t *ctx) {
    // ---------------------------------------------------------------------
    // PASO 1: VERIFICAR INTEGRIDAD (CRC) - ¡Siempre se hace primero!
    // ---------------------------------------------------------------------
    size_t data_length = sizeof(can_frame_payload_t) - sizeof(uint16_t);
    uint16_t calculated_crc = calculate_crc16((const uint8_t *)frame, data_length);

    if (calculated_crc != frame->crc16) {
        ctx->stats_crc_errors++;
        // Si el CRC falla, no nos fiamos del número de secuencia. Abortamos.
        return FT_ERR_CRC_FAILED;
    }

    // ---------------------------------------------------------------------
    // PASO 2: VERIFICAR PÉRDIDA DE TRAMAS (NÚMERO DE SECUENCIA)
    // ---------------------------------------------------------------------
    // Al restar dos uint8_t, C maneja el salto de 255 a 0 perfectamente.
    uint8_t diff = frame->seq_number - ctx->rx_expected_seq;

    if (diff == 0) {
        // Trama recibida en perfecto orden cronológico
        ctx->rx_expected_seq++;
        ctx->stats_frames_ok++;
        return FT_OK;

    } else if (diff < 128) {
        // Si hay un salto positivo (ej. esperábamos la 5 y llega la 8) -> diff = 3
        // Significa que hemos perdido (diff) tramas por el camino.
        ctx->stats_frames_lost += diff;
        ctx->rx_expected_seq = frame->seq_number + 1; // Resincronizamos la secuencia
        return FT_ERR_FRAME_LOST;

    } else {
        // Si diff es mayor a 128 (en uint8_t), equivale a un número "negativo".
        // Ej: Esperábamos la 10 y llega la 9. Es un mensaje duplicado o atrasado.
        // No actualizamos rx_expected_seq para no desincronizarnos.
        return FT_ERR_DUPLICATE;
    }
}
