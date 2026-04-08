#ifndef FAULT_TOLERANCE_H_
#define FAULT_TOLERANCE_H_

#include "can_protocol.h"
#include <stdint.h>

/* Defines configuration */
#define MAX_SEQ_NUM 255
#define MAX_FRAME_LOSS 5
#define MAX_FRAME_LOSS_CONSC 3
#define MAX_CRC_ERRORS 3
#define SENSOR_MAX_VALID 5000
/* ========================================================================= */
/* CODES FOR DATA PROCESSING                                                 */
/* ========================================================================= */

typedef enum {
    FT_OK               = 0,
    FT_ERR_CRC_FAILED   = -1,
    FT_ERR_FRAME_LOST   = -2,
    FT_ERR_MUTED        = -3,
    FT_ERR_CREDIBILITY  = -4,
    FT_SYNC_REQUIRED    = -5
} ft_status_t;

/* ========================================================================= */
/* CONTEXT FOR FAULT TOLERANCE                                               */
/* Save fault tolerance information for each slave                           */
/* ========================================================================= */
typedef struct {
    uint32_t slave_id;
    uint32_t last_valid_data;
    uint32_t expected_seq_num;
    uint32_t stats_crc_errors;
    uint32_t stats_frames_lost;
    uint32_t stats_frames_ok;
    uint8_t  consecutive_crc_errors; 
    uint8_t  consecutive_seq_errors;
    uint8_t  sync_attempts;
    uint8_t  is_muted;
} ft_context_t;

/* Context with fault tolerance information for each slave */
extern ft_context_t slave_contexts[3]; // El array para los 3 esclavos

/* ========================================================================= */
/* FUNCTIONS PROTOTYPES                                                      */
/* ========================================================================= */

/* Initialize fault tolerance context */
void ft_init_context();

/* Prepare a frame for transmission (Adds sequence number and calculates CRC) */
ft_status_t ft_prepare_tx_frame(can_frame_payload_t *frame, uint32_t can_id);

/* Process a received message (Checks CRC and sequence) */
ft_status_t ft_process_message(can_frame_payload_t *frame, uint32_t can_id);

#endif /* FAULT_TOLERANCE_H_ */
