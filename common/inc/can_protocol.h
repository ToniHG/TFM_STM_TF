#ifndef CAN_PROTOCOL_H_
#define CAN_PROTOCOL_H_

#include <stdint.h>

/* ========================================================================= */
/* CAN IDs                                                                   */
/* ========================================================================= */

#define MASTER_ID               0x100  // Commands from Master to Slave (High priority)
#define SLAVE1_ID               0x101  // ID for Slave 1
#define SLAVE2_ID               0x102  // ID for Slave 2
#define SLAVE3_ID               0x103  // ID for Slave 3
#define CAN_ID_SLAVE_ERROR      0x050  // Error report from Slave (Maximum priority)

/* ========================================================================= */
/* MESSAGE TYPES                                                             */
/* ========================================================================= */
typedef enum {
    MSG_TYPE_HEARTBEAT     = 0x01, // Heartbeat to check if the node is alive
    MSG_TYPE_SENSOR_DATA   = 0x02, // Sensor data from Slave to Master (Low priority)
    MSG_TYPE_ACTUATOR_CMD  = 0x03, // Commands from Master to Slave to control actuators
    MSG_TYPE_FAULT_INJECT  = 0xFE, // Message to inject a fault for testing purposes (used in fault tolerance algorithm 2)
    MSG_TYPE_SYNC_REQUIRED = 0xFF  // Message from Master to Slaves to request synchronization (used in fault tolerance algorithm 1)
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

/* Structure for RTOS CAN messages */
typedef struct {
    uint32_t sender_id;
    can_frame_payload_t frame;
} rtos_can_msg_t;

/* ========================================================================= */
/* FUNCTIONS PROTOTYPES                                                      */
/* ========================================================================= */

void can_proto_init_frame(can_frame_payload_t *frame);
void can_proto_pack_sensor_data(can_frame_payload_t *frame, uint32_t sensor_value);
void can_proto_pack_heartbeat(can_frame_payload_t *frame, uint32_t status_code);
void can_proto_pack_fault_injection(can_frame_payload_t *frame, uint32_t fault_code);

#endif /* CAN_PROTOCOL_H_ */
