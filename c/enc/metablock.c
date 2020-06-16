/* Copyright 2015 Google Inc. All Rights Reserved.

   Distributed under MIT license.
   See file LICENSE for detail or copy at https://opensource.org/licenses/MIT
*/

/* Algorithms for distributing the literals and commands of a metablock between
   block types and contexts. */

#include "./metablock.h"

#include "../common/constants.h"
#include "../common/context.h"
#include "../common/platform.h"
#include <brotli/types.h>
#include "./bit_cost.h"
#include "./block_splitter.h"
#include "./cluster.h"
#include "./entropy_encode.h"
#include "./histogram.h"
#include "./memory.h"
#include "./quality.h"

#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {
#endif

void BrotliInitDistanceParams(BrotliEncoderParams* params,
    uint32_t npostfix, uint32_t ndirect) {
  BrotliDistanceParams* dist_params = &params->dist;
  uint32_t alphabet_size_max;
  uint32_t alphabet_size_limit;
  uint32_t max_distance;

  dist_params->distance_postfix_bits = npostfix;
  dist_params->num_direct_distance_codes = ndirect;

  alphabet_size_max = BROTLI_DISTANCE_ALPHABET_SIZE(
      npostfix, ndirect, BROTLI_MAX_DISTANCE_BITS);
  alphabet_size_limit = alphabet_size_max;
  max_distance = ndirect + (1U << (BROTLI_MAX_DISTANCE_BITS + npostfix + 2)) -
      (1U << (npostfix + 2));

  if (params->large_window) {
    BrotliDistanceCodeLimit limit = BrotliCalculateDistanceCodeLimit(
        BROTLI_MAX_ALLOWED_DISTANCE, npostfix, ndirect);
    alphabet_size_max = BROTLI_DISTANCE_ALPHABET_SIZE(
        npostfix, ndirect, BROTLI_LARGE_MAX_DISTANCE_BITS);
    alphabet_size_limit = limit.max_alphabet_size;
    max_distance = limit.max_distance;
  }

  dist_params->alphabet_size_max = alphabet_size_max;
  dist_params->alphabet_size_limit = alphabet_size_limit;
  dist_params->max_distance = max_distance;
}

static void RecomputeDistancePrefixes(Command* cmds,
                                      size_t num_commands,
                                      const BrotliDistanceParams* orig_params,
                                      const BrotliDistanceParams* new_params) {
  size_t i;

  if (orig_params->distance_postfix_bits == new_params->distance_postfix_bits &&
      orig_params->num_direct_distance_codes ==
      new_params->num_direct_distance_codes) {
    return;
  }

  for (i = 0; i < num_commands; ++i) {
    Command* cmd = &cmds[i];
    if (CommandCopyLen(cmd) && cmd->cmd_prefix_ >= 128) {
      PrefixEncodeCopyDistance(CommandRestoreDistanceCode(cmd, orig_params),
                               new_params->num_direct_distance_codes,
                               new_params->distance_postfix_bits,
                               &cmd->dist_prefix_,
                               &cmd->dist_extra_);
    }
  }
}

static BROTLI_BOOL ComputeDistanceCost(const Command* cmds,
                                       size_t num_commands,
                                       const BrotliDistanceParams* orig_params,
                                       const BrotliDistanceParams* new_params,
                                       double* cost) {
  size_t i;
  BROTLI_BOOL equal_params = BROTLI_FALSE;
  uint16_t dist_prefix;
  uint32_t dist_extra;
  double extra_bits = 0.0;
  HistogramDistance histo;
  HistogramClearDistance(&histo);

  if (orig_params->distance_postfix_bits == new_params->distance_postfix_bits &&
      orig_params->num_direct_distance_codes ==
      new_params->num_direct_distance_codes) {
    equal_params = BROTLI_TRUE;
  }

  for (i = 0; i < num_commands; i++) {
    const Command* cmd = &cmds[i];
    if (CommandCopyLen(cmd) && cmd->cmd_prefix_ >= 128) {
      if (equal_params) {
        dist_prefix = cmd->dist_prefix_;
      } else {
        uint32_t distance = CommandRestoreDistanceCode(cmd, orig_params);
        if (distance > new_params->max_distance) {
          return BROTLI_FALSE;
        }
        PrefixEncodeCopyDistance(distance,
                                 new_params->num_direct_distance_codes,
                                 new_params->distance_postfix_bits,
                                 &dist_prefix,
                                 &dist_extra);
      }
      HistogramAddDistance(&histo, dist_prefix & 0x3FF);
      extra_bits += dist_prefix >> 10;
    }
  }

  *cost = BrotliPopulationCostDistance(&histo) + extra_bits;
  return BROTLI_TRUE;
}

void BrotliBuildMetaBlock(MemoryManager* m,
                          const uint8_t* ringbuffer,
                          const size_t pos,
                          const size_t mask,
                          BrotliEncoderParams* params,
                          uint8_t prev_byte,
                          uint8_t prev_byte2,
                          Command* cmds,
                          size_t num_commands,
                          ContextType literal_context_mode,
                          MetaBlockSplit* mb) {
  /* Histogram ids need to fit in one byte. */
  static const size_t kMaxNumberOfHistograms = 256;
  HistogramDistance* distance_histograms;
  HistogramLiteral* literal_histograms;
  ContextType* literal_context_modes = NULL;
  size_t literal_histograms_size;
  size_t distance_histograms_size;
  size_t i;
  size_t literal_context_multiplier = 1;
  uint32_t npostfix;
  uint32_t ndirect_msb = 0;
  BROTLI_BOOL check_orig = BROTLI_TRUE;
  double best_dist_cost = 1e99;
  BrotliEncoderParams orig_params = *params;
  BrotliEncoderParams new_params = *params;

  for (npostfix = 0; npostfix <= BROTLI_MAX_NPOSTFIX; npostfix++) {
    for (; ndirect_msb < 16; ndirect_msb++) {
      uint32_t ndirect = ndirect_msb << npostfix;
      BROTLI_BOOL skip;
      double dist_cost;
      BrotliInitDistanceParams(&new_params, npostfix, ndirect);
      if (npostfix == orig_params.dist.distance_postfix_bits &&
          ndirect == orig_params.dist.num_direct_distance_codes) {
        check_orig = BROTLI_FALSE;
      }
      skip = !ComputeDistanceCost(
          cmds, num_commands,
          &orig_params.dist, &new_params.dist, &dist_cost);
      if (skip || (dist_cost > best_dist_cost)) {
        break;
      }
      best_dist_cost = dist_cost;
      params->dist = new_params.dist;
    }
    if (ndirect_msb > 0) ndirect_msb--;
    ndirect_msb /= 2;
  }
  if (check_orig) {
    double dist_cost;
    ComputeDistanceCost(cmds, num_commands,
                        &orig_params.dist, &orig_params.dist, &dist_cost);
    if (dist_cost < best_dist_cost) {
      /* NB: currently unused; uncomment when more param tuning is added. */
      /* best_dist_cost = dist_cost; */
      params->dist = orig_params.dist;
    }
  }
  RecomputeDistancePrefixes(cmds, num_commands,
                            &orig_params.dist, &params->dist);

  BrotliSplitBlock(m, cmds, num_commands,
                   ringbuffer, pos, mask, params,
                   &mb->literal_split,
                   &mb->command_split,
                   &mb->distance_split);
  if (BROTLI_IS_OOM(m)) return;

  if (!params->disable_literal_context_modeling) {
    literal_context_multiplier = 1 << BROTLI_LITERAL_CONTEXT_BITS;
    literal_context_modes =
        BROTLI_ALLOC(m, ContextType, mb->literal_split.num_types);
    if (BROTLI_IS_OOM(m) || BROTLI_IS_NULL(literal_context_modes)) return;
    for (i = 0; i < mb->literal_split.num_types; ++i) {
      literal_context_modes[i] = literal_context_mode;
    }
  }

  literal_histograms_size =
      mb->literal_split.num_types * literal_context_multiplier;
  literal_histograms =
      BROTLI_ALLOC(m, HistogramLiteral, literal_histograms_size);
  if (BROTLI_IS_OOM(m) || BROTLI_IS_NULL(literal_histograms)) return;
  ClearHistogramsLiteral(literal_histograms, literal_histograms_size);

  distance_histograms_size =
      mb->distance_split.num_types << BROTLI_DISTANCE_CONTEXT_BITS;
  distance_histograms =
      BROTLI_ALLOC(m, HistogramDistance, distance_histograms_size);
  if (BROTLI_IS_OOM(m) || BROTLI_IS_NULL(distance_histograms)) return;
  ClearHistogramsDistance(distance_histograms, distance_histograms_size);

  BROTLI_DCHECK(mb->command_histograms == 0);
  mb->command_histograms_size = mb->command_split.num_types;
  mb->command_histograms =
      BROTLI_ALLOC(m, HistogramCommand, mb->command_histograms_size);
  if (BROTLI_IS_OOM(m) || BROTLI_IS_NULL(mb->command_histograms)) return;
  ClearHistogramsCommand(mb->command_histograms, mb->command_histograms_size);

  BrotliBuildHistogramsWithContext(cmds, num_commands,
      &mb->literal_split, &mb->command_split, &mb->distance_split,
      ringbuffer, pos, mask, prev_byte, prev_byte2, literal_context_modes,
      literal_histograms, mb->command_histograms, distance_histograms);
  BROTLI_FREE(m, literal_context_modes);

  BROTLI_DCHECK(mb->literal_context_map == 0);
  mb->literal_context_map_size =
      mb->literal_split.num_types << BROTLI_LITERAL_CONTEXT_BITS;
  mb->literal_context_map =
      BROTLI_ALLOC(m, uint32_t, mb->literal_context_map_size);
  if (BROTLI_IS_OOM(m) || BROTLI_IS_NULL(mb->literal_context_map)) return;

  BROTLI_DCHECK(mb->literal_histograms == 0);
  mb->literal_histograms_size = mb->literal_context_map_size;
  mb->literal_histograms =
      BROTLI_ALLOC(m, HistogramLiteral, mb->literal_histograms_size);
  if (BROTLI_IS_OOM(m) || BROTLI_IS_NULL(mb->literal_histograms)) return;

  BrotliClusterHistogramsLiteral(m, literal_histograms, literal_histograms_size,
      kMaxNumberOfHistograms, mb->literal_histograms,
      &mb->literal_histograms_size, mb->literal_context_map);
  if (BROTLI_IS_OOM(m)) return;
  BROTLI_FREE(m, literal_histograms);

  if (params->disable_literal_context_modeling) {
    /* Distribute assignment to all contexts. */
    for (i = mb->literal_split.num_types; i != 0;) {
      size_t j = 0;
      i--;
      for (; j < (1 << BROTLI_LITERAL_CONTEXT_BITS); j++) {
        mb->literal_context_map[(i << BROTLI_LITERAL_CONTEXT_BITS) + j] =
            mb->literal_context_map[i];
      }
    }
  }

  BROTLI_DCHECK(mb->distance_context_map == 0);
  mb->distance_context_map_size =
      mb->distance_split.num_types << BROTLI_DISTANCE_CONTEXT_BITS;
  mb->distance_context_map =
      BROTLI_ALLOC(m, uint32_t, mb->distance_context_map_size);
  if (BROTLI_IS_OOM(m) || BROTLI_IS_NULL(mb->distance_context_map)) return;

  BROTLI_DCHECK(mb->distance_histograms == 0);
  mb->distance_histograms_size = mb->distance_context_map_size;
  mb->distance_histograms =
      BROTLI_ALLOC(m, HistogramDistance, mb->distance_histograms_size);
  if (BROTLI_IS_OOM(m) || BROTLI_IS_NULL(mb->distance_histograms)) return;

  BrotliClusterHistogramsDistance(m, distance_histograms,
                                  mb->distance_context_map_size,
                                  kMaxNumberOfHistograms,
                                  mb->distance_histograms,
                                  &mb->distance_histograms_size,
                                  mb->distance_context_map);
  if (BROTLI_IS_OOM(m)) return;
  BROTLI_FREE(m, distance_histograms);
}

#define FN(X) X ## Literal
#include "utf8_util.h"
#include "./metablock_split_lit_inc.h"  /* NOLINT(build/include) */
#undef FN

#define FN(X) X ## Command
#include "./metablock_inc.h"  /* NOLINT(build/include) */
#undef FN

#define FN(X) X ## Distance
#include "./metablock_inc.h"  /* NOLINT(build/include) */
#undef FN

static void BrotliBuildMetaBlockGreedy(
    MemoryManager* m, const uint8_t* ringbuffer, size_t pos, size_t mask,
    const Command* commands, size_t n_commands, MetaBlockSplit* mb) {
  BlockSplitterLiteral lit_blocks;
  BlockSplitterCommand cmd_blocks;
  BlockSplitterDistance dist_blocks;
  size_t num_literals = 0;
  size_t i;
  for (i = 0; i < n_commands; ++i) {
    num_literals += commands[i].insert_len_;
  }
  size_t literals_remaining = num_literals;

  InitBlockSplitterLiteral(m, &lit_blocks, num_literals, &mb->literal_split,
      &mb->literal_histograms, &mb->literal_histograms_size);
  if (BROTLI_IS_OOM(m)) return;
  InitBlockSplitterCommand(m, &cmd_blocks, BROTLI_NUM_COMMAND_SYMBOLS, 1024,
      500.0, n_commands, &mb->command_split, &mb->command_histograms,
      &mb->command_histograms_size);
  if (BROTLI_IS_OOM(m)) return;
  InitBlockSplitterDistance(m, &dist_blocks, 64, 512, 100.0, n_commands,
      &mb->distance_split, &mb->distance_histograms,
      &mb->distance_histograms_size);
  if (BROTLI_IS_OOM(m)) return;

  for (i = 0; i < n_commands; ++i) {
    const Command cmd = commands[i];
    size_t j;
    BlockSplitterAddSymbolCommand(&cmd_blocks, cmd.cmd_prefix_);
    for (j = cmd.insert_len_; j != 0; --j) {
      BlockSplitterAddSymbolLiteral(&lit_blocks, &ringbuffer[pos & mask], literals_remaining);
      ++pos;
      --literals_remaining;
    }
    pos += CommandCopyLen(&cmd);
    if (CommandCopyLen(&cmd)) {
      if (cmd.cmd_prefix_ >= 128) {
        BlockSplitterAddSymbolDistance(&dist_blocks, cmd.dist_prefix_ & 0x3FF);
      }
    }
  }

  BlockSplitterFinishBlockLiteral(&lit_blocks, /* is_final = */ BROTLI_TRUE);
  BlockSplitterFinishBlockCommand(&cmd_blocks, /* is_final = */ BROTLI_TRUE);
  BlockSplitterFinishBlockDistance(&dist_blocks, /* is_final = */ BROTLI_TRUE);
}

void BrotliOptimizeHistograms(uint32_t num_distance_codes,
                              MetaBlockSplit* mb) {
  uint8_t good_for_rle[BROTLI_NUM_COMMAND_SYMBOLS];
  size_t i;
  for (i = 0; i < mb->literal_histograms_size; ++i) {
    BrotliOptimizeHuffmanCountsForRle(256, mb->literal_histograms[i].data_,
                                      good_for_rle);
  }
  for (i = 0; i < mb->command_histograms_size; ++i) {
    BrotliOptimizeHuffmanCountsForRle(BROTLI_NUM_COMMAND_SYMBOLS,
                                      mb->command_histograms[i].data_,
                                      good_for_rle);
  }
  for (i = 0; i < mb->distance_histograms_size; ++i) {
    BrotliOptimizeHuffmanCountsForRle(num_distance_codes,
                                      mb->distance_histograms[i].data_,
                                      good_for_rle);
  }
}

#if defined(__cplusplus) || defined(c_plusplus)
}  /* extern "C" */
#endif
