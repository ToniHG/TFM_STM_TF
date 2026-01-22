#ifndef CAN_PROTOCOL_H_
#define CAN_PROTOCOL_H_

#include <stdint.h>

/* ========================================================================= */
/* 1. DEFINICIÓN DE IDENTIFICADORES CAN (CAN IDs)                            */
/* Usamos identificadores estándar de 11 bits. En CAN, el ID más bajo        */
/* tiene la máxima prioridad en el bus.                                      */
/* ========================================================================= */

#define CAN_ID_MASTER_CMD       0x100  // Comandos del Master al Esclavo (Alta prioridad)
#define CAN_ID_SLAVE_TELEMETRY  0x200  // Datos normales del Esclavo al Master
#define CAN_ID_SLAVE_ERROR      0x050  // Reporte de error del Esclavo (¡Máxima prioridad!)

/* ========================================================================= */
/* 2. TIPOS DE MENSAJE                                                       */
/* ========================================================================= */
typedef enum {
    MSG_TYPE_HEARTBEAT = 0x01,   // Latido para comprobar que el nodo sigue vivo
    MSG_TYPE_SENSOR_DATA = 0x02, // Datos de operación normal
    MSG_TYPE_ACTUATOR_CMD= 0x03, // Comando para accionar algo en el esclavo
    MSG_TYPE_FAULT_INJECT = 0xFF // Comando especial para simular un fallo en el TFM
} msg_type_t;

/* ========================================================================= */
/* 3. ESTRUCTURA DEL PAYLOAD (MÁXIMO 8 BYTES)                                */
/* Utilizamos __attribute__((packed)) para evitar que el compilador añada  */
/* bytes de padding (relleno) ocultos, asegurando que ocupe exactamente 8B.*/
/* ========================================================================= */

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;      // Byte 0: Tipo de mensaje (msg_type_t)
    uint8_t  seq_number;    // Byte 1: Número de secuencia (Algoritmo Tolerancia a Fallos 1)
    uint32_t payload_data;  // Bytes 2-5: Los datos reales (ej. valor de un sensor o comando)
    uint16_t crc16;         // Bytes 6-7: Checksum CRC16 (Algoritmo Tolerancia a Fallos 2)
} can_frame_payload_t;

/* ========================================================================= */
/* 4. PROTOTIPOS DE FUNCIONES (Helpers de empaquetado)                       */
/* ========================================================================= */

// Funciones definidas en can_protocol.c para armar las tramas fácilmente
void can_proto_init_frame(can_frame_payload_t *frame);
void can_proto_pack_sensor_data(can_frame_payload_t *frame, uint32_t sensor_value);
void can_proto_pack_heartbeat(can_frame_payload_t *frame, uint32_t status_code);
void can_proto_pack_fault_injection(can_frame_payload_t *frame, uint32_t fault_code);

#endif /* CAN_PROTOCOL_H_ */
