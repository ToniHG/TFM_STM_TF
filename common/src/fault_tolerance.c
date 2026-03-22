#include "fault_tolerance.h"
#include "crc16.h"
#include <stddef.h>

/* Context with fault tolerance information for each slave */
ft_context_t slave_contexts[3];

/* Initialize fault tolerance context */
void ft_init_context() {
    for(int i = 0; i < 3; i++) {
        slave_contexts[i].expected_seq_num = 0;
        slave_contexts[i].stats_crc_errors = 0;
        slave_contexts[i].stats_frames_lost = 0;
        slave_contexts[i].consecutive_crc_errors = 0;
        slave_contexts[i].is_muted = 0;
        slave_contexts[i].last_valid_data = 0;
    }
}

/* Get the index of a slave based on its CAN ID */
static int get_slave_index(uint32_t can_id) {
    if (can_id == SLAVE1_ID) return 0;
    if (can_id == SLAVE2_ID) return 1;
    if (can_id == SLAVE3_ID) return 2;
    return -1; // ID no reconocido
}

ft_status_t ft_prepare_tx_frame(can_frame_payload_t *frame, uint32_t can_id) {
    int idx = get_slave_index(can_id);
    if (idx < 0) return FT_ERR_FRAME_LOST;

    ft_context_t *ctx = &slave_contexts[idx];

    /* Assign the current sequence number and prepare the next one (Algorithm 1) */
    frame->seq_number = ctx->expected_seq_num;
    ctx->expected_seq_num++; 

    /* Calculate the length of the data to be CRC'd */
    size_t data_length = sizeof(can_frame_payload_t) - sizeof(uint16_t);

    /* Calculate the CRC */
    frame->crc16 = calculate_crc16((const uint8_t *)frame, data_length);

    return FT_OK;
}

ft_status_t ft_process_message(can_frame_payload_t *frame, uint32_t can_id) {
    int idx = get_slave_index(can_id);
    if (idx < 0) return FT_ERR_FRAME_LOST;      // If the CAN ID is not recognized, we consider it as a lost frame (or you could define another error code)

    ft_context_t *ctx = &slave_contexts[idx];   // Get the context for this slave

    /* Check if the slave is muted */
    if (ctx->is_muted) {
        return FT_ERR_MUTED; 
    }

    /* Check CRC */
    uint16_t calculated_crc = calculate_crc16((uint8_t*)frame, sizeof(can_frame_payload_t) - 2);
    if (calculated_crc != frame->crc16) {
        ctx->stats_crc_errors++;
        ctx->consecutive_crc_errors++;
        
        /* If too many CRC errors, mute the slave */
        if (ctx->consecutive_crc_errors >= MAX_CRC_ERRORS) {
            ctx->is_muted = 1;          // Mute the slave
            ctx->stats_frames_lost = 0; // Reset counter after reaching the threshold
            ctx->expected_seq_num = 0;  // Reset expected sequence number 
        }
        return FT_ERR_CRC_FAILED;
    }
    
    /* Reset consecutive CRC errors */
    ctx->consecutive_crc_errors = 0;

    /* Check sensor data credibility */
    if (frame->payload_data > SENSOR_MAX_VALID) {
        return FT_ERR_CREDIBILITY;
    }

    /* Check sequence number */
    if (frame->seq_number != ctx->expected_seq_num) {
        ctx->stats_frames_lost++;
        /* Update expected sequence number */
        ctx->expected_seq_num = frame->seq_number + 1;
        
        /* If we lost too many frames, we might want to consider the node as failed and require resynchronization */
        if (ctx->stats_frames_lost >= MAX_FRAME_LOSS) {
            ctx->stats_frames_lost = 0; // Reset counter after reaching the threshold
            ctx->expected_seq_num = 0;  // Reset expected sequence number
            return FT_SYNC_REQUIRED; 
        }
        return FT_SYNC_REQUIRED; 
    }
    /* No errors, update the expected sequence number and save the valid data */
    ctx->expected_seq_num++;
    ctx->last_valid_data = frame->payload_data;

    return FT_OK;
}