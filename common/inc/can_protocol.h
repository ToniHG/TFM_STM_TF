#ifndef CAN_PROTOCOL_H_
#define CAN_PROTOCOL_H_

#include <stdint.h>

/* ========================================================================= */
/* CAN IDs                                                                   */
/* ========================================================================= */

#define CAN_ID_MASTER_CMD       0x100  // Commands from Master to Slave (High priority)
#define CAN_ID_SLAVE_TELEMETRY  0x200  // Normal data from Slave to Master
#define CAN_ID_SLAVE_ERROR      0x050  // Error report from Slave (Maximum priority)

/* ========================================================================= */
/* MESSAGE TYPES                                                             */
/* ========================================================================= */
typedef enum {
    MSG_TYPE_HEARTBEAT = 0x01,   // Heartbeat to check if the node is alive
    MSG_TYPE_SENSOR_DATA = 0x02, // Normal operation data
    MSG_TYPE_ACTUATOR_CMD= 0x03, // Command to actuate something on the slave
    MSG_TYPE_FAULT_INJECT = 0xFF // Special command to simulate a fault in the TFM
} msg_type_t;

/* ========================================================================= */
/* STRUCTURE OF THE PAYLOAD (MAX 8 BYTES)                                    */
/* ========================================================================= */

typedef struct __attribute__((packed)) {
    uint8_t  msg_type       : 8;   // Byte 0: Message type (ej. heartbeat, sensor data, etc.)
    uint8_t  seq_number     : 8;   // Byte 1: Sequence number for fault tolerance (Algorithm 1)
    uint32_t payload_data   : 32;  // Bytes 2-5: Data for the message (ej. sensor reading, status code, etc.)
    uint16_t crc16          : 16;  // Bytes 6-7: Checksum CRC16 (Algorithm 2)
} can_frame_payload_t;

/* ========================================================================= */
/* FUNCTIONS PROTOTYPES                                                      */
/* ========================================================================= */

void can_proto_init_frame(can_frame_payload_t *frame);
void can_proto_pack_sensor_data(can_frame_payload_t *frame, uint32_t sensor_value);
void can_proto_pack_heartbeat(can_frame_payload_t *frame, uint32_t status_code);
void can_proto_pack_fault_injection(can_frame_payload_t *frame, uint32_t fault_code);

#endif /* CAN_PROTOCOL_H_ */
