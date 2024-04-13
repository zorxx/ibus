/*! \copyright 2024 Zorxx Software. All rights reserved.
 *  \file ibus.c
 *  \brief Flysky IBUS protocol parser implementation
 */
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "helpers.h"
#include "ibus_private.h"

/* ----------------------------------------------------------------------------------------
 * Private Functions
 */

static void ibus_ChannelInit(ibus_channel_t *channel)
{
   memset(channel, 0, sizeof(*channel));
   channel->min = USHRT_MAX; 
}

static void ibus_UpdateChannel(ibus_channel_t *channel, uint16_t value)
{
   channel->value = value; 
   if(channel->min > value)
      channel->min = value;
   if(channel->max < value)
      channel->max = value;
   ++(channel->update_count);
}

static void ibus_HandleBuffer(ibus_state_t *state)
{
   //uint8_t address = ctx->buffer[0] & 0x0f;
   uint8_t command = (state->buffer[0] & 0xf0) >> 4;
   switch(command)
   {
      case IBUS_CMD_SERVO:
         if(state->len != 29)
            break;
         for(int i = 0; i < IBUS_CHANNEL_COUNT; ++i)
         {
            ibus_channel_t *channel = &state->results.channel[i];
            uint16_t value = state->buffer[1 + (i*2)]
                           | ((uint16_t)(state->buffer[2 + (i*2)]) << 8);
            ibus_UpdateChannel(channel, value);
         }
         break;
   }
}

/* ----------------------------------------------------------------------------------------
 * Exported Functions
 */

ibus_context_t ibus_init(ibus_lowlevel_config *config)
{
   ibus_state_t *state = (ibus_state_t *) malloc(sizeof(*state));
   if(NULL == state)
      return NULL;

   memset(state, 0, sizeof(*state));
   state->state = IBSTATE_GET_LENGTH;
   state->min_delay = UINT_MAX;
   state->max_delay = 0;
   for(int idx = 0; idx < IBUS_CHANNEL_COUNT; ++idx)
      ibus_ChannelInit(&state->results.channel[idx]);

   if(!ibus_ll_init(state, config))
   {
      IBUSERR("[%s] Low-level initialization failed\n", __func__);
      free(state);
      return NULL;
   }


   return (ibus_context_t) state;
}

bool ibus_deinit(ibus_context_t context)
{
   ibus_state_t *state = (ibus_state_t *) context;

   if(NULL != state)
   {
      ibus_ll_deinit(state);
      free(state);
   }
   return true;
}

void ibus_handle_byte(ibus_context_t context, uint8_t val, uint64_t timestamp)
{
   ibus_state_t *state = (ibus_state_t *) context;
   uint64_t delta_rx;

   ++(state->rx_count);

   if(timestamp > 0)
   {
      if(timestamp < state->last_rx)
         delta_rx = state->timestamp_max - state->last_rx + timestamp;
      else
         delta_rx = timestamp - state->last_rx;
      if(delta_rx > state->rx_timeout) 
      {
         ++(state->results.timeout_count);
         state->state = IBSTATE_GET_LENGTH;
      }
      if(state->max_delay < delta_rx)
         state->max_delay = delta_rx;
      if(state->min_delay > delta_rx)
         state->min_delay = delta_rx;
   }
   state->last_rx = timestamp;

   switch(state->state)
   {
      case IBSTATE_GET_LENGTH:
         if(val <= IBUS_MAX_BUFFER_LENGTH && val > IBUS_PROTOCOL_OVERHEAD)
         {
            state->ptr = 0;
            state->len = val - IBUS_PROTOCOL_OVERHEAD;
            state->checksum = 0xFFFF - val;
            state->state = IBSTATE_GET_DATA;
        }
        break;

      case IBSTATE_GET_DATA:
         state->ptr++;
         state->checksum -= val;
         if(state->ptr >= state->len)
            state->state = IBSTATE_GET_CHECKSUM_LOW;
         break;
        
      case IBSTATE_GET_CHECKSUM_LOW:
         state->checksum_expected = val;
         state->state = IBSTATE_GET_CHECKSUM_HIGH;
         break;

      case IBSTATE_GET_CHECKSUM_HIGH:
         state->checksum_expected |= ((uint16_t) val) << 8;
         if(state->checksum == state->checksum_expected) 
         {
            ibus_HandleBuffer(state);
            ++(state->results.rx_success_count);
         }
         else
         {
            ++(state->results.checksum_fail_count);
         }
         state->state = IBSTATE_GET_LENGTH;
         break;

      default:
        break;
   }
}