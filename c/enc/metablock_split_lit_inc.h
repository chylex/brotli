/* NOLINT(build/header_guard) */
/* Copyright 2015 Google Inc. All Rights Reserved.

   Distributed under MIT license.
   See file LICENSE for detail or copy at https://opensource.org/licenses/MIT
*/

/* template parameters: FN */

#define HistogramType FN(Histogram)

static const size_t min_block_size = 1000;

/* UTF-8 aware block splitter for literals.
*/
typedef struct FN(BlockSplitter) {
  size_t num_blocks_;
  BlockSplit* split_;  /* not owned */
  HistogramType* histograms_;  /* not owned */
  size_t* histograms_size_;  /* not owned */

  /* The number of symbols in the current histogram. */
  size_t block_size_;
  /* Offset of the current histogram. */
  size_t curr_histogram_ix_;

  size_t prev_utf_code_point_length;
} FN(BlockSplitter);

static void FN(InitBlockSplitter)(
    MemoryManager* m, FN(BlockSplitter)* self, size_t num_symbols,
    BlockSplit* split, HistogramType** histograms, size_t* histograms_size) {
  size_t max_num_blocks = num_symbols / min_block_size + 1;
  /* We have to allocate one more histogram than the maximum number of block
     types for the current histogram when the meta-block is too big. */
  size_t max_num_types =
      BROTLI_MIN(size_t, max_num_blocks, BROTLI_MAX_NUMBER_OF_BLOCK_TYPES + 1);
  self->num_blocks_ = 0;
  self->split_ = split;
  self->histograms_size_ = histograms_size;
  self->block_size_ = 0;
  self->curr_histogram_ix_ = 0;
  self->prev_utf_code_point_length = 0;
  BROTLI_ENSURE_CAPACITY(m, uint8_t,
      split->types, split->types_alloc_size, max_num_blocks);
  BROTLI_ENSURE_CAPACITY(m, uint32_t,
      split->lengths, split->lengths_alloc_size, max_num_blocks);
  if (BROTLI_IS_OOM(m)) return;
  self->split_->num_blocks = max_num_blocks;
  BROTLI_DCHECK(*histograms == 0);
  *histograms_size = max_num_types;
  *histograms = BROTLI_ALLOC(m, HistogramType, *histograms_size);
  self->histograms_ = *histograms;
  if (BROTLI_IS_OOM(m) || BROTLI_IS_NULL(*histograms)) return;
  /* Clear only current histogram. */
  FN(HistogramClear)(&self->histograms_[0]);
}

/* Does either of three things:
     (1) emits the current block with a new block type;
     (2) emits the current block with the type of the second last block;
     (3) merges the current block with the last block. */
static void FN(BlockSplitterFinishBlock)(
    FN(BlockSplitter)* self, BROTLI_BOOL is_final) {
  BlockSplit* split = self->split_;
  HistogramType* histograms = self->histograms_;
  if (self->num_blocks_ == 0) {
    /* Create first block. */
    split->lengths[0] = (uint32_t)self->block_size_;
    split->types[0] = 0;
    ++self->num_blocks_;
    ++split->num_types;
    ++self->curr_histogram_ix_;
    if (self->curr_histogram_ix_ < *self->histograms_size_)
      FN(HistogramClear)(&histograms[self->curr_histogram_ix_]);
    self->block_size_ = 0;
  } else if (self->block_size_ > 0) {
    if (split->num_types < BROTLI_MAX_NUMBER_OF_BLOCK_TYPES) {
      /* Create new block. */
      split->lengths[self->num_blocks_] = (uint32_t)self->block_size_;
      split->types[self->num_blocks_] = (uint8_t)split->num_types;
      ++self->num_blocks_;
      ++split->num_types;
      ++self->curr_histogram_ix_;
      if (self->curr_histogram_ix_ < *self->histograms_size_)
        FN(HistogramClear)(&histograms[self->curr_histogram_ix_]);
      self->block_size_ = 0;
    } else {
      /* Combine this block with last block. */
      split->lengths[self->num_blocks_ - 1] += (uint32_t)self->block_size_;
      FN(HistogramAddHistogram)(&histograms[self->curr_histogram_ix_ - 1],
          &histograms[self->curr_histogram_ix_]);
      self->block_size_ = 0;
    }
  }
  if (is_final) {
    *self->histograms_size_ = split->num_types;
    split->num_blocks = self->num_blocks_;
  }
}

/* Adds the next symbol to the current histogram. When the current histogram
   reaches the target size, decides on merging the block. */
static void FN(BlockSplitterAddSymbol)(FN(BlockSplitter)* self, const uint8_t* input, const size_t size) {
  int ignore;
  const size_t code_point_length = BrotliParseAsUTF8(&ignore, input, size);

  FN(HistogramAdd)(&self->histograms_[self->curr_histogram_ix_], input[0]);
  ++self->block_size_;
  if (self->prev_utf_code_point_length == 0) {
    self->prev_utf_code_point_length = code_point_length;
  } else if (self->prev_utf_code_point_length != code_point_length && self->block_size_ >= min_block_size) {
    self->prev_utf_code_point_length = code_point_length;
    FN(BlockSplitterFinishBlock)(self, /* is_final = */ BROTLI_FALSE);
  }
}

#undef HistogramType
