#include "can_protocol.h"
#include <string.h> // Necesario para la función memset()

/* ========================================================================= */
/* FUNCIONES DE INICIALIZACIÓN Y EMPAQUETADO (PACKING)                       */
/* ========================================================================= */

/**
 * @brief  Limpia una trama poniéndola toda a ceros para evitar basura en memoria.
 */
void can_proto_init_frame(can_frame_payload_t *frame) {
    if (frame != NULL) {
        // Llenamos los 8 bytes de la estructura con ceros
        memset(frame, 0, sizeof(can_frame_payload_t));
    }
}

/**
 * @brief  Prepara un mensaje de tipo Telemetría/Sensor.
 * @param  sensor_value: El valor de 32 bits que queremos enviar.
 */
void can_proto_pack_sensor_data(can_frame_payload_t *frame, uint32_t sensor_value) {
    can_proto_init_frame(frame);
    frame->msg_type = MSG_TYPE_SENSOR_DATA;
    frame->payload_data = sensor_value;

    // Nota: El seq_number y el crc16 NO se rellenan aquí.
    // Se encarga de ello la función ft_prepare_tx_frame() de tolerancia a fallos.
}

/**
 * @brief  Prepara un mensaje tipo "Latido" (Heartbeat) para indicar que el nodo está vivo.
 * @param  status_code: Un código de estado (ej. 0 = OK, 1 = Error de hardware local).
 */
void can_proto_pack_heartbeat(can_frame_payload_t *frame, uint32_t status_code) {
    can_proto_init_frame(frame);
    frame->msg_type = MSG_TYPE_HEARTBEAT;
    frame->payload_data = status_code;
}

/**
 * @brief  Prepara un mensaje para inyectar un fallo intencionado (Útil para pruebas del TFM).
 * @param  fault_code: El tipo de fallo a simular en el nodo receptor.
 */
void can_proto_pack_fault_injection(can_frame_payload_t *frame, uint32_t fault_code) {
    can_proto_init_frame(frame);
    frame->msg_type = MSG_TYPE_FAULT_INJECT;
    frame->payload_data = fault_code;
}
