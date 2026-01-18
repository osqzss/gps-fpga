// gp2021_axi_wrapper.sv (single-channel, Option A)
// AXI-lite register bank + TIC generator (from samp_clk only) + CDC.
//
// Channel controls are written via AXI-lite (no external GPIO):
//   - Writing PRN=0 disables channel.
//   - Writing PRN!=0 sets PRN and enables channel.
//   - Slew is a one-shot pulse generated in samp_clk domain on AXI write.
//
// Register map (all 32-bit):
//   0x0000 ID                (RO) 0x47503231 "GP21"
//   0x0004 VERSION           (RO) 0x00010000
//   0x0008 PROG_TIC_CYCLES   (RW) default 1,636,800 (0.1s at 16.368MHz)
//   0x000C CONTROL           (RW) bit0 enable_tic (default 1)
//   0x0010 TIC_COUNT         (RO) number of TIC events seen (AXI domain)
//   0x0014 STATUS            (RO/W1C) bit0 sticky_tic_seen, bit1 sticky_dump_seen
//
//   0x0020 CH_PRN            (RW) [7:0] PRN. write 0 disables channel.
//   0x0024 CH_CARR_INCR      (RW) carrier NCO phase increment (PHASE_W=32)
//   0x0028 CH_CODE_INCR      (RW) code NCO phase increment (PHASE_W=32)
//   0x002C CH_SLEW_HALFCHIPS (RW) [10:0] halfchip amount
//   0x0030 CH_SLEW_REQ       (WO) write bit0=1 to request slew (one-shot)
//   0x0034 CH_ENABLE         (RO) bit0 enable (derived from PRN write)
//
// Channel window base: 0x0100 (single channel)
//   +0x00 DUMP_SEQ           (RO)
//   +0x04 I_E                (RO, sign-extended)
//   +0x08 Q_E
//   +0x0C I_P
//   +0x10 Q_P
//   +0x14 I_L
//   +0x18 Q_L
//   +0x1C DUMP_EPOCH         (RO) {16'h0, epoch}
//   +0x20 MEAS_SEQ           (RO)
//   +0x24 TIC_EPOCH          (RO) {16'h0, epoch}
//   +0x28 TIC_CODE_PHASE     (RO) {21'h0, halfchip[10:0]}
//   +0x2C TIC_CODE_NCO_PHASE (RO) {22'h0, msb10[9:0]}
//   +0x30 TIC_CARR_CYCLE     (RO) {12'h0, cycle[19:0]}
//   +0x34 TIC_CARR_NCO_PHASE (RO) {22'h0, msb10[9:0]}

`timescale 1ns/1ps

module gp2021_axi_wrapper #(
  parameter int ACC_W = 18
)(
  // ---------------- Sample clock domain ----------------
  input  wire                    samp_clk,
  input  wire                    samp_rstn,

  // Channel controls OUT (sample domain)
  output wire                    ch_enable_samp,
  output wire [7:0]              ch_prn_samp,
  output wire [31:0]             ch_carr_incr_samp,
  output wire [31:0]             ch_code_incr_samp,
  output wire                    ch_slew_req_samp,        // 1-cycle pulse
  output wire [10:0]             ch_slew_halfchips_samp,  // stable value

  // Dump pulse + dump values IN (sample domain)
  input  wire                    dump_pulse,
  input  wire signed [ACC_W-1:0] i_early,
  input  wire signed [ACC_W-1:0] q_early,
  input  wire signed [ACC_W-1:0] i_prompt,
  input  wire signed [ACC_W-1:0] q_prompt,
  input  wire signed [ACC_W-1:0] i_late,
  input  wire signed [ACC_W-1:0] q_late,
  input  wire [15:0]             epoch_dump,

  // TIC-latched measurement taps IN (sample domain)
  input  wire [15:0]             tic_epoch,
  input  wire [10:0]             tic_code_phase_halfchip,
  input  wire [9:0]              tic_code_nco_phase,
  input  wire [19:0]             tic_carrier_cycle,
  input  wire [9:0]              tic_carrier_nco_phase,

  // Export TIC pulse in sample domain (optional)
  output wire                    tic_pulse_samp,

  // ---------------- AXI-lite clock domain ----------------
  input  wire                     axi_clk,
  input  wire                     axi_rstn,

  // AXI-lite slave (32-bit)
  input  wire [31:0]              s_axi_awaddr,
  input  wire                     s_axi_awvalid,
  output reg                      s_axi_awready,

  input  wire [31:0]              s_axi_wdata,
  input  wire [3:0]               s_axi_wstrb,
  input  wire                     s_axi_wvalid,
  output reg                      s_axi_wready,

  output reg  [1:0]               s_axi_bresp,
  output reg                      s_axi_bvalid,
  input  wire                     s_axi_bready,

  input  wire [31:0]              s_axi_araddr,
  input  wire                     s_axi_arvalid,
  output reg                      s_axi_arready,

  output reg  [31:0]              s_axi_rdata,
  output reg  [1:0]               s_axi_rresp,
  output reg                      s_axi_rvalid,
  input  wire                     s_axi_rready
);

  // ============================================================
  // 1) Global control regs (AXI domain)
  // ============================================================
  localparam logic [31:0] ID_REG  = 32'h4750_3231; // "GP21"
  localparam logic [31:0] VER_REG = 32'h0001_0000;

  reg [31:0] prog_tic_cycles_axi;
  reg        tic_enable_axi;

  // Channel control regs (AXI domain)
  reg        ch_enable_axi;
  reg [7:0]  ch_prn_axi;
  reg [31:0] ch_carr_incr_axi;
  reg [31:0] ch_code_incr_axi;
  reg [10:0] ch_slew_halfchips_axi;
  reg        ch_slew_toggle_axi; // toggled on write => edge => pulse in samp

  initial begin
    prog_tic_cycles_axi   = 32'd1636800;
    tic_enable_axi        = 1'b1;
    ch_enable_axi         = 1'b0;
    ch_prn_axi            = 8'd0;
    ch_carr_incr_axi      = 32'h4000_0000;
    ch_code_incr_axi      = 32'h1000_0000;
    ch_slew_halfchips_axi = 11'd0;
    ch_slew_toggle_axi    = 1'b0;
  end

  // ============================================================
  // 2) CDC: controls AXI -> samp_clk
  // ============================================================
  cdc_sync_bus #(.W(1))  u_cdc_en   (.dst_clk(samp_clk), .dst_rstn(samp_rstn), .src_bus(ch_enable_axi),    .dst_bus(ch_enable_samp));
  cdc_sync_bus #(.W(8))  u_cdc_prn  (.dst_clk(samp_clk), .dst_rstn(samp_rstn), .src_bus(ch_prn_axi),       .dst_bus(ch_prn_samp));
  cdc_sync_bus #(.W(32)) u_cdc_carr (.dst_clk(samp_clk), .dst_rstn(samp_rstn), .src_bus(ch_carr_incr_axi), .dst_bus(ch_carr_incr_samp));
  cdc_sync_bus #(.W(32)) u_cdc_code (.dst_clk(samp_clk), .dst_rstn(samp_rstn), .src_bus(ch_code_incr_axi), .dst_bus(ch_code_incr_samp));
  cdc_sync_bus #(.W(11)) u_cdc_slew (.dst_clk(samp_clk), .dst_rstn(samp_rstn), .src_bus(ch_slew_halfchips_axi), .dst_bus(ch_slew_halfchips_samp));

  // Slew request pulse: sync toggle then edge detect in samp
  wire ch_slew_toggle_samp;
  cdc_sync_bus #(.W(1)) u_cdc_slew_tog (.dst_clk(samp_clk), .dst_rstn(samp_rstn), .src_bus(ch_slew_toggle_axi), .dst_bus(ch_slew_toggle_samp));

  reg ch_slew_toggle_d;
  reg ch_slew_req_pulse;
  assign ch_slew_req_samp = ch_slew_req_pulse;

  always @(posedge samp_clk) begin
    if (!samp_rstn) begin
      ch_slew_toggle_d <= 1'b0;
      ch_slew_req_pulse <= 1'b0;
    end else begin
      ch_slew_req_pulse <= 1'b0;
      if (ch_slew_toggle_samp ^ ch_slew_toggle_d) begin
        ch_slew_req_pulse <= 1'b1; // one-shot
      end
      ch_slew_toggle_d <= ch_slew_toggle_samp;
    end
  end

  // TIC control regs -> samp
  wire [31:0] prog_tic_cycles_samp;
  wire        tic_enable_samp;
  cdc_sync_bus #(.W(32)) u_cdc_tic_cycles (.dst_clk(samp_clk), .dst_rstn(samp_rstn), .src_bus(prog_tic_cycles_axi), .dst_bus(prog_tic_cycles_samp));
  cdc_sync_bus #(.W(1))  u_cdc_tic_en     (.dst_clk(samp_clk), .dst_rstn(samp_rstn), .src_bus(tic_enable_axi),      .dst_bus(tic_enable_samp));

  // ============================================================
  // 3) TIC generator in sample domain (independent of dump)
  // ============================================================
  tic_gen_from_clk u_tic (
    .clk(samp_clk),
    .rstn(samp_rstn),
    .enable(tic_enable_samp),
    .period_cycles(prog_tic_cycles_samp),
    .tic_pulse(tic_pulse_samp)
  );

  // ============================================================
  // 4) Samp-domain latching of DUMP + TIC measurement
  // ============================================================
  reg [31:0] dump_seq_samp;
  reg signed [ACC_W-1:0] dump_ie_samp, dump_qe_samp;
  reg signed [ACC_W-1:0] dump_ip_samp, dump_qp_samp;
  reg signed [ACC_W-1:0] dump_il_samp, dump_ql_samp;
  reg [15:0]             dump_epoch_samp;
  reg                    dump_stb_samp;

  reg [31:0] meas_seq_samp;
  reg [15:0] tic_epoch_samp;
  reg [10:0] tic_code_phase_samp;
  reg [9:0]  tic_code_nco_phase_samp;
  reg [19:0] tic_carrier_cycle_samp;
  reg [9:0]  tic_carrier_nco_phase_samp;
  reg        meas_stb_samp;

  always @(posedge samp_clk) begin
    if (!samp_rstn) begin
      dump_seq_samp <= 32'd0;
      dump_stb_samp <= 1'b0;
      dump_ie_samp <= '0; dump_qe_samp <= '0;
      dump_ip_samp <= '0; dump_qp_samp <= '0;
      dump_il_samp <= '0; dump_ql_samp <= '0;
      dump_epoch_samp <= 16'd0;

      meas_seq_samp <= 32'd0;
      meas_stb_samp <= 1'b0;
      tic_epoch_samp <= 16'd0;
      tic_code_phase_samp <= 11'd0;
      tic_code_nco_phase_samp <= 10'd0;
      tic_carrier_cycle_samp <= 20'd0;
      tic_carrier_nco_phase_samp <= 10'd0;
    end else begin
      if (dump_pulse) begin
        dump_ie_samp <= i_early;
        dump_qe_samp <= q_early;
        dump_ip_samp <= i_prompt;
        dump_qp_samp <= q_prompt;
        dump_il_samp <= i_late;
        dump_ql_samp <= q_late;
        dump_epoch_samp <= epoch_dump;
        dump_seq_samp <= dump_seq_samp + 1;
        dump_stb_samp <= ~dump_stb_samp;
      end

      if (tic_pulse_samp) begin
        tic_epoch_samp <= tic_epoch;
        tic_code_phase_samp <= tic_code_phase_halfchip;
        tic_code_nco_phase_samp <= tic_code_nco_phase;
        tic_carrier_cycle_samp <= tic_carrier_cycle;
        tic_carrier_nco_phase_samp <= tic_carrier_nco_phase;
        meas_seq_samp <= meas_seq_samp + 1;
        meas_stb_samp <= ~meas_stb_samp;
      end
    end
  end

  // ============================================================
  // 5) CDC of DUMP/MEAS into AXI domain and shadow regs
  // ============================================================
  wire [31:0] dump_seq_sync;
  wire signed [ACC_W-1:0] dump_ie_sync, dump_qe_sync, dump_ip_sync, dump_qp_sync, dump_il_sync, dump_ql_sync;
  wire [15:0] dump_epoch_sync;
  wire dump_stb_sync;

  wire [31:0] meas_seq_sync;
  wire [15:0] tic_epoch_sync;
  wire [10:0] tic_code_phase_sync;
  wire [9:0]  tic_code_nco_phase_sync;
  wire [19:0] tic_carrier_cycle_sync;
  wire [9:0]  tic_carrier_nco_phase_sync;
  wire meas_stb_sync;

  cdc_sync_bus #(.W(32))   u_sd0(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(dump_seq_samp),   .dst_bus(dump_seq_sync));
  cdc_sync_bus #(.W(ACC_W))u_sd1(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(dump_ie_samp),    .dst_bus(dump_ie_sync));
  cdc_sync_bus #(.W(ACC_W))u_sd2(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(dump_qe_samp),    .dst_bus(dump_qe_sync));
  cdc_sync_bus #(.W(ACC_W))u_sd3(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(dump_ip_samp),    .dst_bus(dump_ip_sync));
  cdc_sync_bus #(.W(ACC_W))u_sd4(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(dump_qp_samp),    .dst_bus(dump_qp_sync));
  cdc_sync_bus #(.W(ACC_W))u_sd5(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(dump_il_samp),    .dst_bus(dump_il_sync));
  cdc_sync_bus #(.W(ACC_W))u_sd6(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(dump_ql_samp),    .dst_bus(dump_ql_sync));
  cdc_sync_bus #(.W(16))   u_sd7(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(dump_epoch_samp), .dst_bus(dump_epoch_sync));
  cdc_sync_bus #(.W(1))    u_sd8(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(dump_stb_samp),   .dst_bus(dump_stb_sync));

  cdc_sync_bus #(.W(32)) u_sm0(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(meas_seq_samp), .dst_bus(meas_seq_sync));
  cdc_sync_bus #(.W(16)) u_sm1(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(tic_epoch_samp), .dst_bus(tic_epoch_sync));
  cdc_sync_bus #(.W(11)) u_sm2(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(tic_code_phase_samp), .dst_bus(tic_code_phase_sync));
  cdc_sync_bus #(.W(10)) u_sm3(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(tic_code_nco_phase_samp), .dst_bus(tic_code_nco_phase_sync));
  cdc_sync_bus #(.W(20)) u_sm4(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(tic_carrier_cycle_samp), .dst_bus(tic_carrier_cycle_sync));
  cdc_sync_bus #(.W(10)) u_sm5(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(tic_carrier_nco_phase_samp), .dst_bus(tic_carrier_nco_phase_sync));
  cdc_sync_bus #(.W(1))  u_sm6(.dst_clk(axi_clk), .dst_rstn(axi_rstn), .src_bus(meas_stb_samp), .dst_bus(meas_stb_sync));

  // Shadow regs (AXI domain)
  reg [31:0] dump_seq_axi;
  reg signed [ACC_W-1:0] dump_ie_axi, dump_qe_axi, dump_ip_axi, dump_qp_axi, dump_il_axi, dump_ql_axi;
  reg [15:0] dump_epoch_axi;

  reg [31:0] meas_seq_axi;
  reg [15:0] tic_epoch_axi;
  reg [10:0] tic_code_phase_axi;
  reg [9:0]  tic_code_nco_phase_axi;
  reg [19:0] tic_carrier_cycle_axi;
  reg [9:0]  tic_carrier_nco_phase_axi;

  reg dump_stb_d;
  reg meas_stb_d;

  reg [31:0] tic_count_axi;
  reg        sticky_tic_seen;
  reg        sticky_dump_seen;

  // ------------------------------------------------------------
  // W1C clear requests (from AXI writes)
  //
  // Vivado reports a critical warning if sticky_tic_seen / sticky_dump_seen
  // are driven from multiple always blocks (multiple drivers).
  // To keep sticky_* driven from a single always block, we generate
  // "clear requests" in the AXI write-handshake always block, and consume
  // them here.
  //
  // These are one-cycle pulses in axi_clk domain.
  reg w1c_clr_tic_pulse;
  reg w1c_clr_dump_pulse;

  always @(posedge axi_clk) begin
    if (!axi_rstn) begin
      dump_stb_d <= 1'b0;
      meas_stb_d <= 1'b0;
      tic_count_axi <= 32'd0;
      sticky_tic_seen <= 1'b0;
      sticky_dump_seen <= 1'b0;

      dump_seq_axi <= 32'd0;
      dump_ie_axi <= '0; dump_qe_axi <= '0;
      dump_ip_axi <= '0; dump_qp_axi <= '0;
      dump_il_axi <= '0; dump_ql_axi <= '0;
      dump_epoch_axi <= 16'd0;

      meas_seq_axi <= 32'd0;
      tic_epoch_axi <= 16'd0;
      tic_code_phase_axi <= 11'd0;
      tic_code_nco_phase_axi <= 10'd0;
      tic_carrier_cycle_axi <= 20'd0;
      tic_carrier_nco_phase_axi <= 10'd0;
    end else begin
      // Apply W1C clears (from AXI writes) first.
      // Note: w1c_clr_*_pulse is generated in the AXI write-handshake block,
      // so sticky_* will clear one axi_clk later than the write acceptance.
      if (w1c_clr_tic_pulse)  sticky_tic_seen  <= 1'b0;
      if (w1c_clr_dump_pulse) sticky_dump_seen <= 1'b0;

      dump_stb_d <= dump_stb_sync;
      meas_stb_d <= meas_stb_sync;

      if (dump_stb_d ^ dump_stb_sync) begin
        sticky_dump_seen <= 1'b1;
        dump_seq_axi <= dump_seq_sync;
        dump_ie_axi  <= dump_ie_sync;
        dump_qe_axi  <= dump_qe_sync;
        dump_ip_axi  <= dump_ip_sync;
        dump_qp_axi  <= dump_qp_sync;
        dump_il_axi  <= dump_il_sync;
        dump_ql_axi  <= dump_ql_sync;
        dump_epoch_axi <= dump_epoch_sync;
      end

      if (meas_stb_d ^ meas_stb_sync) begin
        sticky_tic_seen <= 1'b1;
        tic_count_axi <= tic_count_axi + 1;
        meas_seq_axi <= meas_seq_sync;
        tic_epoch_axi <= tic_epoch_sync;
        tic_code_phase_axi <= tic_code_phase_sync;
        tic_code_nco_phase_axi <= tic_code_nco_phase_sync;
        tic_carrier_cycle_axi <= tic_carrier_cycle_sync;
        tic_carrier_nco_phase_axi <= tic_carrier_nco_phase_sync;
      end
    end
  end

  // ============================================================
  // 6) AXI-lite slave (minimal, single outstanding R/B)
  // ============================================================
  reg [31:0] awaddr_q;
  reg        have_aw;

  always @(posedge axi_clk) begin
    if (!axi_rstn) begin
      s_axi_awready <= 1'b1;
      s_axi_wready  <= 1'b1;
      s_axi_bvalid  <= 1'b0;
      s_axi_bresp   <= 2'b00;
      have_aw       <= 1'b0;
      awaddr_q      <= 32'd0;

      // W1C clear pulses
      w1c_clr_tic_pulse  <= 1'b0;
      w1c_clr_dump_pulse <= 1'b0;
    end else begin
      // Default: no clear request. (do_axi_write may assert for one cycle)
      w1c_clr_tic_pulse  <= 1'b0;
      w1c_clr_dump_pulse <= 1'b0;

      if (s_axi_awready && s_axi_awvalid) begin
        awaddr_q <= s_axi_awaddr;
        have_aw  <= 1'b1;
      end

      if (have_aw && s_axi_wvalid && s_axi_wready && !s_axi_bvalid) begin
        do_axi_write(awaddr_q, s_axi_wdata, s_axi_wstrb);
        s_axi_bvalid <= 1'b1;
        s_axi_bresp  <= 2'b00;
        have_aw      <= 1'b0;
      end

      if (s_axi_bvalid && s_axi_bready) begin
        s_axi_bvalid <= 1'b0;
      end

      s_axi_awready <= !s_axi_bvalid && !have_aw;
      s_axi_wready  <= !s_axi_bvalid;
    end
  end

  always @(posedge axi_clk) begin
    if (!axi_rstn) begin
      s_axi_arready <= 1'b1;
      s_axi_rvalid  <= 1'b0;
      s_axi_rresp   <= 2'b00;
      s_axi_rdata   <= 32'd0;
    end else begin
      if (s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
        s_axi_rdata <= do_axi_read(s_axi_araddr);
        s_axi_rresp <= 2'b00;
        s_axi_rvalid <= 1'b1;
      end

      if (s_axi_rvalid && s_axi_rready) begin
        s_axi_rvalid <= 1'b0;
      end

      s_axi_arready <= !s_axi_rvalid;
    end
  end

  // ============================================================
  // 7) AXI read/write decode
  // ============================================================
  function automatic [31:0] do_axi_read(input [31:0] addr);
    logic [31:0] off;
    begin
      do_axi_read = 32'd0;

      case (addr[15:0])
        16'h0000: do_axi_read = ID_REG;
        16'h0004: do_axi_read = VER_REG;
        16'h0008: do_axi_read = prog_tic_cycles_axi;
        16'h000C: do_axi_read = {31'd0, tic_enable_axi};
        16'h0010: do_axi_read = tic_count_axi;
        16'h0014: do_axi_read = {30'd0, sticky_dump_seen, sticky_tic_seen};

        16'h0020: do_axi_read = {24'd0, ch_prn_axi};
        16'h0024: do_axi_read = ch_carr_incr_axi;
        16'h0028: do_axi_read = ch_code_incr_axi;
        16'h002C: do_axi_read = {21'd0, ch_slew_halfchips_axi};
        16'h0034: do_axi_read = {31'd0, ch_enable_axi};
        default:  do_axi_read = 32'd0;
      endcase

      if (addr[15:8] == 8'h01) begin
        off = addr[7:0];
        case (off)
          8'h00: do_axi_read = dump_seq_axi;
          8'h04: do_axi_read = {{(32-ACC_W){dump_ie_axi[ACC_W-1]}}, dump_ie_axi};
          8'h08: do_axi_read = {{(32-ACC_W){dump_qe_axi[ACC_W-1]}}, dump_qe_axi};
          8'h0C: do_axi_read = {{(32-ACC_W){dump_ip_axi[ACC_W-1]}}, dump_ip_axi};
          8'h10: do_axi_read = {{(32-ACC_W){dump_qp_axi[ACC_W-1]}}, dump_qp_axi};
          8'h14: do_axi_read = {{(32-ACC_W){dump_il_axi[ACC_W-1]}}, dump_il_axi};
          8'h18: do_axi_read = {{(32-ACC_W){dump_ql_axi[ACC_W-1]}}, dump_ql_axi};
          8'h1C: do_axi_read = {16'd0, dump_epoch_axi};

          8'h20: do_axi_read = meas_seq_axi;
          8'h24: do_axi_read = {16'd0, tic_epoch_axi};
          8'h28: do_axi_read = {21'd0, tic_code_phase_axi};
          8'h2C: do_axi_read = {22'd0, tic_code_nco_phase_axi};
          8'h30: do_axi_read = {12'd0, tic_carrier_cycle_axi};
          8'h34: do_axi_read = {22'd0, tic_carrier_nco_phase_axi};
          default: do_axi_read = 32'd0;
        endcase
      end
    end
  endfunction

  task automatic do_axi_write(
    input [31:0] addr,
    input [31:0] wdata,
    input [3:0]  wstrb
  );
    begin
      case (addr[15:0])
        16'h0008: begin
          if (wstrb[0]) prog_tic_cycles_axi[7:0]   <= wdata[7:0];
          if (wstrb[1]) prog_tic_cycles_axi[15:8]  <= wdata[15:8];
          if (wstrb[2]) prog_tic_cycles_axi[23:16] <= wdata[23:16];
          if (wstrb[3]) prog_tic_cycles_axi[31:24] <= wdata[31:24];
        end
        16'h000C: begin
          if (wstrb[0]) tic_enable_axi <= wdata[0];
        end
        16'h0014: begin
          // STATUS W1C: bit0 clears sticky_tic_seen, bit1 clears sticky_dump_seen
          // IMPORTANT: sticky_* is ONLY driven in the "CDC latch" always block.
          // To avoid multiple drivers (Vivado critical warning), we generate
          // one-cycle clear pulses here and let the other always block apply them.
          if (wstrb[0] && wdata[0]) w1c_clr_tic_pulse  <= 1'b1;
          if (wstrb[0] && wdata[1]) w1c_clr_dump_pulse <= 1'b1;
          //if (wstrb[0] && wdata[0]) sticky_tic_seen  <= 1'b0;
          //if (wstrb[0] && wdata[1]) sticky_dump_seen <= 1'b0;
        end

        // Channel controls
        16'h0020: begin
          if (wstrb[0]) begin
            ch_prn_axi <= wdata[7:0];
            ch_enable_axi <= (wdata[7:0] != 8'd0);
          end
        end
        16'h0024: begin
          if (wstrb[0]) ch_carr_incr_axi[7:0]   <= wdata[7:0];
          if (wstrb[1]) ch_carr_incr_axi[15:8]  <= wdata[15:8];
          if (wstrb[2]) ch_carr_incr_axi[23:16] <= wdata[23:16];
          if (wstrb[3]) ch_carr_incr_axi[31:24] <= wdata[31:24];
        end
        16'h0028: begin
          if (wstrb[0]) ch_code_incr_axi[7:0]   <= wdata[7:0];
          if (wstrb[1]) ch_code_incr_axi[15:8]  <= wdata[15:8];
          if (wstrb[2]) ch_code_incr_axi[23:16] <= wdata[23:16];
          if (wstrb[3]) ch_code_incr_axi[31:24] <= wdata[31:24];
        end
        16'h002C: begin
          if (wstrb[0]) ch_slew_halfchips_axi[7:0] <= wdata[7:0];
          if (wstrb[1]) ch_slew_halfchips_axi[10:8] <= wdata[10:8];
        end
        16'h0030: begin
          // write bit0=1 to request a slew (toggle)
          if (wstrb[0] && wdata[0]) ch_slew_toggle_axi <= ~ch_slew_toggle_axi;
        end
        default: begin end
      endcase
    end
  endtask

endmodule

// ============================================================
// CDC: 2FF sync for a bus
// ============================================================
module cdc_sync_bus #(
  parameter int W = 1
)(
  input  wire         dst_clk,
  input  wire         dst_rstn,
  input  wire [W-1:0] src_bus,
  output reg  [W-1:0] dst_bus
);
  reg [W-1:0] ff1;
  always @(posedge dst_clk) begin
    if (!dst_rstn) begin
      ff1     <= '0;
      dst_bus <= '0;
    end else begin
      ff1     <= src_bus;
      dst_bus <= ff1;
    end
  end
endmodule

// ============================================================
// TIC generator from samp_clk only
// ============================================================
module tic_gen_from_clk(
  input  wire        clk,
  input  wire        rstn,
  input  wire        enable,
  input  wire [31:0] period_cycles,
  output reg         tic_pulse
);
  reg [31:0] cnt;

  always @(posedge clk) begin
    if (!rstn) begin
      cnt <= 32'd0;
      tic_pulse <= 1'b0;
    end else begin
      tic_pulse <= 1'b0;

      if (!enable) begin
        cnt <= 32'd0;
      end else begin
        if (period_cycles <= 32'd1) begin
          tic_pulse <= 1'b1;
          cnt <= 32'd0;
        end else if (cnt == period_cycles - 32'd1) begin
          tic_pulse <= 1'b1;
          cnt <= 32'd0;
        end else begin
          cnt <= cnt + 32'd1;
        end
      end
    end
  end
endmodule
