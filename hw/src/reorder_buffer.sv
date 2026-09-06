// Copyright 2023 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Matheus Cavalcante, ETH Zurich
//
// This generic module provides an interface through which responses can
// be read in order, despite being written out of order. The responses
// must be indexed with an ID that identifies it within the ROB.
//
// Extension (default OFF -> bit-identical legacy elaboration):
// - BlockWords>1: BLOCK ID RESERVATION (docs/spatz_mlp_design_plan.md §5.1). One
//   id_req_block_i reserves BlockWords consecutive ids in a SINGLE cycle instead of
//   walking them one per cycle. The ROB owns the room check (room_block_o) so a
//   requester bug cannot over-allocate the id space, and block_mask_o exports the
//   granted window for the requester's own per-id bookkeeping. BlockWords==1 is the
//   feature-absent default: every added statement const-folds out.
//
// id_valid_o is derived from the occupancy counter. Ids are allocated and freed
// strictly in order, so a free-id bitmap would be a redundant encoding of
// status_cnt_q -- see the derivation at the assign below.

module reorder_buffer
  import cf_math_pkg::idx_width;
#(
  parameter int unsigned DataWidth  = 0,
  parameter int unsigned NumWords   = 0,
  parameter bit FallThrough         = 1'b0,
  // Block id reservation width. 1 = feature absent (bit-identical legacy elaboration).
  // When > 1 it must be exactly NumWords/2: the window mask below exploits that identity
  // to reduce "(i - wp) mod NumWords < BlockWords" to a single msb test (checked below).
  parameter int unsigned BlockWords = 1,
  // Dependant parameters. Do not change!
  parameter int unsigned IdWidth    = idx_width(NumWords),
  parameter type data_t             = logic [DataWidth-1:0],
  parameter type id_t               = logic [IdWidth-1:0]
) (
  input  logic  clk_i,
  input  logic  rst_ni,
  // Data write
  input  data_t data_i,
  input  id_t   id_i,
  input  logic  push_i,
  // Data read
  output data_t data_o,
  output logic  valid_o,
  output id_t   id_read_o,
  input  logic  pop_i,
  // ID request
  input  logic  id_req_i,
  // Allocate this id as a DUMMY: it is marked valid immediately and carries no data, so the
  // in-order read head passes over it. Used to keep every buffer's allocation count equal
  // when a burst's last row does not cover every lane, which is what lets one base id
  // describe the whole burst.
  input  logic  id_dummy_i,
  // Block grant only: how many ids at the END of the granted window carry no beat.
  // A burst reserves a whole window in one cycle, but a short burst does not fill it;
  // those trailing ids are marked here exactly as a walked dummy would be.
  input  logic [IdWidth:0] id_dummy_cnt_i,
  output logic  dummy_o,     // the current head is a dummy: pop it, do not consume it
  output id_t   id_o,
  output logic  id_valid_o,  // is the next id valid?
  output logic  full_o,
  output logic  empty_o,
  // Block ID reservation (used when BlockWords > 1; tie off otherwise). One cycle reserves
  // the whole window [id_o, id_o+BlockWords). room_block_o is the ROB's OWN room guard --
  // block_fire is built from it internally, so the requester cannot bypass it.
  input  logic                id_req_block_i,
  output logic                room_block_o,
  output logic [NumWords-1:0] block_mask_o
);

  /****************
   *  Parameters  *
   ****************/

  // Split point of every id for the window compare: i = {i_hi, i_lo} at bit log2(BlockWords).
  // (IdWidth-1 coincides with idx_width(BlockWords) only when BlockWords == NumWords/2; at
  // NumWords=64/BlockWords=16 the window is a QUARTER of the ring and the split moves to bit 4.)
  // Held at 1 when the feature is absent so the slice stays legal in the off branch.
  localparam int unsigned BlkLoW = (BlockWords > 1) ? idx_width(BlockWords) : 1;
  // Width of the high ("quadrant") part of an id above the BlkLoW split.
  localparam int unsigned QSelW  = (BlockWords > 1) ? (IdWidth - BlkLoW) : 1;

  /*************
   *  Signals  *
   *************/

  id_t              read_pointer_d, read_pointer_q;
  id_t              write_pointer_d, write_pointer_q;
  // Keep track of the ROB utilization
  logic [IdWidth:0] status_cnt_d, status_cnt_q;

  // Block reservation: the granted window as a bitmap, and the single fire condition that
  // both the pointer update and the counter fixups are built from.
  logic                  block_fire;
  logic [NumWords-1:0]   block_mask;
  // Shared thermometer decode for the window mask: blk_lt[j] = (j < write_pointer_q low bits).
  logic [BlockWords-1:0] blk_lt;
  // Quadrant decomposition of the write pointer and the per-quadrant equality one-hots.
  // Declared at module level (project convention: no signals inside generate loops).
  logic [BlkLoW-1:0]   wp_lo;
  logic [QSelW-1:0]    wp_hi, wp_hi1;
  logic [2**QSelW-1:0] eq_wp, eq_wp1;

  // Memory
  data_t [NumWords-1:0] mem_d, mem_q;
  logic  [NumWords-1:0] valid_d, valid_q;
  logic  [NumWords-1:0] dummy_d, dummy_q;

  // Status flags
  assign full_o    = (status_cnt_q == NumWords);
  assign empty_o   = (status_cnt_q == 'd0);
  assign id_o      = write_pointer_q;
  assign id_read_o = read_pointer_q;

  // "Are the next two ids free?" -- the VLSU burst allocator demands two (see its
  // rob_id_valid use). Ids are allocated and freed strictly in order, so the allocated
  // set is always the contiguous ring [read_pointer_q, write_pointer_q) whose cardinality
  // *is* status_cnt_q (checked by cnt_ptr_coherent below). Writing cnt for status_cnt_q
  assign id_valid_o = (status_cnt_q <= (NumWords - 2));

  // Room for a WHOLE block. The NON-STRICT <= is load-bearing, not stylistic
  //: after the first burst of a two-burst load
  // handshakes, status_cnt_q is EXACTLY NumWords-BlockWords, so a strict < would silently
  // re-serialise the second burst -- and the symptom is "the change did nothing", not a
  // failure. Note this compares against a COMPILE-TIME CONSTANT, never against a requested
  // length: keeping the burst-length cone out of full_o/room_block_o is what holds this at
  // ~2 levels from a flop instead of ~28 (T3).
  assign room_block_o = (BlockWords > 1) ? (status_cnt_q <= (NumWords - BlockWords)) : 1'b0;
  assign block_fire   = (BlockWords > 1) && id_req_block_i && room_block_o;
  assign block_mask_o = block_mask;

  // Window mask: block_mask[i] = 1 iff i is in [wp, wp+BlockWords) mod NumWords. Split every
  // id at the BlkLoW bit into {i_hi, i_lo}; then
  //   (i - wp) mod NumWords < BlockWords
  //     <=> (i_hi == wp_hi && i_lo >= wp_lo) || (i_hi == wp_hi+1 && i_lo < wp_lo)
  if (BlockWords > 1) begin : gen_block_mask
    assign wp_lo  = write_pointer_q[BlkLoW-1:0];
    assign wp_hi  = write_pointer_q[IdWidth-1:BlkLoW];
    assign wp_hi1 = wp_hi + QSelW'(1);
    for (genvar q = 0; q < 2**QSelW; q++) begin : gen_quad_eq
      assign eq_wp[q]  = (wp_hi  == QSelW'(q));
      assign eq_wp1[q] = (wp_hi1 == QSelW'(q));
    end : gen_quad_eq
    for (genvar j = 0; j < BlockWords; j++) begin : gen_block_mask_bit
      assign blk_lt[j] = (BlkLoW'(j) < wp_lo);
    end : gen_block_mask_bit
    for (genvar q = 0; q < 2**QSelW; q++) begin : gen_quad_mask
      for (genvar j = 0; j < BlockWords; j++) begin : gen_quad_mask_bit
        assign block_mask[q*BlockWords + j] = (eq_wp[q]  & ~blk_lt[j]) |
                                              (eq_wp1[q] &  blk_lt[j]);
      end : gen_quad_mask_bit
    end : gen_quad_mask
  end else begin : gen_no_block_mask
    assign wp_lo      = '0;
    assign wp_hi      = '0;
    assign wp_hi1     = '0;
    assign eq_wp      = '0;
    assign eq_wp1     = '0;
    assign blk_lt     = '0;
    assign block_mask = '0;
  end

  // Read and Write logic
  always_comb begin: read_write_comb
    // Maintain state
    read_pointer_d  = read_pointer_q;
    write_pointer_d = write_pointer_q;
    status_cnt_d    = status_cnt_q;
    mem_d           = mem_q;
    valid_d         = valid_q;
    dummy_d         = dummy_q;

    // Output data
    data_o  = mem_q[read_pointer_q];
    valid_o = valid_q[read_pointer_q];
    dummy_o = dummy_q[read_pointer_q];

    // Reserve a whole block of ids in ONE cycle. Written as the head of an if / else-if with
    // the single-id request so the two are mutually exclusive BY CONSTRUCTION rather than by
    // an external promise (A4): if a requester ever asserts both, the block wins here and in
    // every counter fixup below, consistently.
    if (block_fire) begin
      // NumWords is a power of two whenever BlockWords > 1 (checked below), so id_t
      // arithmetic wraps naturally; with BlockWords == NumWords/2 this is one inverter on
      // the pointer msb, cheaper than the +1 incrementer it parallels.
      write_pointer_d = id_t'(write_pointer_q + BlockWords);
      status_cnt_d = status_cnt_q + BlockWords;
      // Pre-fill the window's trailing dummies so the in-order head passes over them.
      for (int j = 0; j < BlockWords; j++)
        if (j >= (BlockWords - int'(id_dummy_cnt_i))) begin
          valid_d[id_t'(write_pointer_q + j)] = 1'b1;
          dummy_d[id_t'(write_pointer_q + j)] = 1'b1;
        end
    // Request an ID.
    end else if (id_req_i && !full_o) begin
      // Increment the write pointer
      if (write_pointer_q == NumWords-1) begin
        write_pointer_d = 0;
      end else begin
        write_pointer_d = write_pointer_q + 1;
      end
      // Increment the overall counter
      status_cnt_d = status_cnt_q + 1;
      // A dummy needs no response: mark it filled here so the read head can pass it.
      if (id_dummy_i) begin
        valid_d[write_pointer_q] = 1'b1;
        dummy_d[write_pointer_q] = 1'b1;
      end
    end

    // Push data
    if (push_i) begin
      mem_d[id_i]   = data_i;
      valid_d[id_i] = 1'b1;
      dummy_d[id_i] = 1'b0;
    end

    // Second slot-addressed write port

    // ROB is in fall-through mode -> do not change the pointers
    if (FallThrough && push_i && (id_i == read_pointer_q)) begin
      data_o  = data_i;
      valid_o = 1'b1;
      if (pop_i) begin
        valid_d[id_i] = 1'b0;
      end
    end

    // Pop data
    if (pop_i && valid_o) begin
      // Word was consumed
      valid_d[read_pointer_q] = 1'b0;
      dummy_d[read_pointer_q] = 1'b0;

      // Increment the read pointer
      if (read_pointer_q == NumWords-1)
        read_pointer_d = '0;
      else
        read_pointer_d = read_pointer_q + 1;
      // Decrement the overall counter
      status_cnt_d = status_cnt_q - 1;
    end

    // Keep the overall counter stable if we request new ID and pop at the same time
    if ((id_req_i && !full_o) && (pop_i && valid_o)) begin
      status_cnt_d = status_cnt_q;
    end

    // Same fixup for a block reservation coincident with a pop: +BlockWords from the block,
    // -1 from the pop. Placed AFTER the single-id fixup so the block wins if both were
    // somehow requested -- the same priority the allocation above uses. The popped id can
    // never lie inside the window (room_block_o bounds the occupancy so read_pointer_q sits
    // at or beyond wp+BlockWords), and the pop's counter update below runs later anyway.
    if (block_fire && (pop_i && valid_o)) begin
      status_cnt_d = status_cnt_q + BlockWords - 1;
    end

  end: read_write_comb

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      read_pointer_q  <= '0;
      write_pointer_q <= '0;
      status_cnt_q    <= '0;
      mem_q           <= '0;
      valid_q         <= '0;
      dummy_q         <= '0;
    end else begin
      read_pointer_q  <= read_pointer_d;
      write_pointer_q <= write_pointer_d;
      status_cnt_q    <= status_cnt_d;
      mem_q           <= mem_d;
      valid_q         <= valid_d;
      dummy_q         <= dummy_d;
    end
  end


  /****************
   *  Assertions  *
   ****************/

  if (NumWords == 0)
    $error("NumWords cannot be 0.");
  // BlockWords must be a power-of-two PROPER divisor of NumWords (16/32 and 16/64 verified):
  // the window mask splits each id at log2(BlockWords) and reduces bit-identically to the
  // legacy msb-XOR form when BlockWords == NumWords/2.
  if ((BlockWords != 1) &&
      !((BlockWords > 1) && (NumWords % BlockWords == 0) && (BlockWords == 2**$clog2(BlockWords))))
    $error("BlockWords must be 1 (off) or a power-of-two divisor of NumWords.");
  if ((BlockWords > 1) && (NumWords != 2**IdWidth))
    $error("BlockWords > 1 requires power-of-two NumWords (write pointer + BlockWords wrap).");
  if ((BlockWords > 1) && FallThrough)
    $error("FallThrough is not supported with block ID reservation.");

  `ifndef VERILATOR
  // pragma translate_off
  full_write : assert property(
      @(posedge clk_i) disable iff (!rst_ni) (full_o |-> !id_req_i))
  else $fatal (1, "Trying to request an ID although the ROB is full.");

  empty_read : assert property(
      @(posedge clk_i) disable iff (!rst_ni) (!valid_o |-> !pop_i))
  else $fatal (1, "Trying to pop data although the top of the ROB is not valid.");

  // A pop must never underflow status_cnt_q. valid_o is valid_q[read_pointer_q],
  // which is NOT the same as !empty_o: a valid head with status_cnt_q == 0 (a push
  // landing outside the allocated [read_pointer_q, write_pointer_q) window) makes
  // status_cnt_d wrap to 2**(IdWidth+1)-1, after which full_o and empty_o never
  // assert again and every later id request / drain misbehaves. empty_read above
  // does not cover this case (it only relates pop_i to valid_o).
  pop_no_underflow : assert property(
      @(posedge clk_i) disable iff (!rst_ni) (pop_i |-> !empty_o))
  else $fatal (1, "ROB pop while empty: status_cnt_q would underflow.");

  // A7: the invariant id_valid_o rests on -- ids are allocated
  // and freed strictly in order, so the occupancy counter and the (write - read) ring
  // distance always agree mod NumWords, and the counter never exceeds NumWords. Armed
  // in both elaborations (sim-only): it validates the legacy build before the knob is
  // turned on, and it also catches the status_cnt_q wrap of pop_no_underflow above.
  if (NumWords == 2**IdWidth) begin : gen_cnt_ptr_assert
    cnt_ptr_coherent : assert property(
        @(posedge clk_i) disable iff (!rst_ni)
        ((status_cnt_q <= NumWords) &&
         (id_t'(status_cnt_q) == id_t'(write_pointer_q - read_pointer_q))))
    else $fatal (1, "status_cnt_q incoherent with the (write - read) pointer distance.");
  end

  if (BlockWords > 1) begin : gen_block_asserts
    // A3: the block may only ever fire on the ROOM check (cnt <= NumWords-BlockWords), never
    // on full_o (cnt <= NumWords-1). Gating it on full_o instead would let status_cnt_d reach
    // NumWords+BlockWords-1 WITHOUT wrapping, double-allocating BlockWords-1 slots -- wrong
    // data with no other symptom. Tautological against the assign above BY DESIGN: it is the
    // tripwire for a future edit that rewrites block_fire.
    blk_room : assert property(
        @(posedge clk_i) disable iff (!rst_ni) (block_fire |-> room_block_o))
    else $fatal (1, "Block reservation fired without room for a whole block.");
    blk_no_overflow : assert property(
        @(posedge clk_i) disable iff (!rst_ni) (block_fire |-> (status_cnt_d <= NumWords)))
    else $fatal (1, "Block reservation overflows the ROB occupancy counter.");
    // A4: block and single id request are mutually exclusive. The allocation gives the block
    // priority, so a coincident single request is silently dropped -- and its requester then
    // uses an id the ROB never handed out.
    blk_single_exclusive : assert property(
        @(posedge clk_i) disable iff (!rst_ni) (!(id_req_block_i && id_req_i)))
    else $fatal (1, "Block and single ID request asserted in the same cycle.");
  end

  // pragma translate_on
  `endif

endmodule: reorder_buffer
