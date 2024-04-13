#ifndef IBUS_PRIVATE_H
#define IBUS_PRIVATE_H

#include "ibus.h"

#define IBUS_MAX_BUFFER_LENGTH   0x20

#define IBUS_PROTOCOL_OVERHEAD   3     // Don't include 1-byte length or 2-byte checksum 

typedef enum
{
   IBUS_CMD_SERVO       = 4,
   IBUS_CMD_DISCIVER    = 8,
} ibus_cmd_e;

typedef enum
{
   IBSTATE_GET_LENGTH = 0,
   IBSTATE_GET_DATA,
   IBSTATE_GET_CHECKSUM_LOW,
   IBSTATE_GET_CHECKSUM_HIGH
} ibus_state_e;

typedef struct ibus_state_s
{
   ibus_state_e state;
   uint8_t ptr;
   uint8_t len;
   uint16_t checksum;
   uint16_t checksum_expected;
   uint8_t buffer[IBUS_MAX_BUFFER_LENGTH];
   uint64_t rx_count;

   uint64_t rx_timeout;
   uint64_t timestamp_max;
   uint64_t min_delay;
   uint64_t max_delay;
   uint64_t last_rx; /* timestamp of most recent rx character */   

   ibus_results_t results;
   void *lowlevel;
} ibus_state_t;

int ibus_ll_init(ibus_state_t *state, ibus_lowlevel_config *config);
int ibus_ll_deinit(ibus_state_t *state);

#endif /* IBUS_PRIVATE_H */
