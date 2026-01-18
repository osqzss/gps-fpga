// top_singlech_axi_hw.sv (Option A)
// Synthesizable hardware top for Basys3 + MicroBlaze.
// - Uses external 16.368 MHz sampling clock from RF front-end.
// - Uses 2-bit sign/magnitude sample from RF front-end (mapped to -3,-1,+1,+3).
// - Channel control and data read-out via AXI4-Lite (gp2021_axi_wrapper).
//
// Notes:
// - AXI clock is expected from the MicroBlaze system clock (e.g., 100 MHz).
// - Sampling clock is independent; gp2021_axi_wrapper handles CDC.

`timescale 1ns/1ps

module top_singlech_axi_hw #(
  parameter int PHASE_W = 32,
  parameter int ACC_W   = 18
)(
  // -------------- Front-end sample clock / reset --------------
  input  wire                 fe_samp_clk_in,  // 16.368 MHz from RF front-end (MRCC)
  input  wire                 fe_samp_rstn,    // active-low reset synced externally or from MB

  // -------------- Front-end 2-bit sign/magnitude samples --------------
  input  wire                 fe_sign,         // 0: positive, 1: negative (assumed)
  input  wire                 fe_mag,          // 0: magnitude=1, 1: magnitude=3 (assumed)

  // -------------- AXI clock / reset (MicroBlaze domain) --------------
  input  wire                 axi_clk,
  input  wire                 axi_rstn,

  // -------------- AXI4-Lite slave --------------
  input  wire [31:0]          s_axi_awaddr,
  input  wire                 s_axi_awvalid,
  output wire                 s_axi_awready,
  input  wire [31:0]          s_axi_wdata,
  input  wire [3:0]           s_axi_wstrb,
  input  wire                 s_axi_wvalid,
  output wire                 s_axi_wready,
  output wire [1:0]           s_axi_bresp,
  output wire                 s_axi_bvalid,
  input  wire                 s_axi_bready,
  input  wire [31:0]          s_axi_araddr,
  input  wire                 s_axi_arvalid,
  output wire                 s_axi_arready,
  output wire [31:0]          s_axi_rdata,
  output wire [1:0]           s_axi_rresp,
  output wire                 s_axi_rvalid,
  input  wire                 s_axi_rready,

  // Optional debug (route to LEDs / ILA as you like)
  output wire                 dbg_dump_pulse
);

  // ---- Clocking ----
  wire samp_clk;
  BUFG u_bufg_samp (.I(fe_samp_clk_in), .O(samp_clk));

  // ---- Map 2-bit sign/mag -> signed 3-bit level (-3,-1,+1,+3) ----
  wire signed [2:0] fe_val = fe_sign ? (fe_mag ? -3'sd3 : -3'sd1)
                                     : (fe_mag ?  3'sd3 :  3'sd1);

  // ---- Channel controls from wrapper (samp domain) ----
  wire                    ch_enable;
  wire [7:0]              ch_prn;
  wire [PHASE_W-1:0]      ch_carr_incr;
  wire [PHASE_W-1:0]      ch_code_incr;
  wire                    ch_slew_req;
  wire [10:0]             ch_slew_halfchips;

  // ---- TIC pulse (samp domain) ----
  wire tic_pulse_samp;

  // ---- Channel outputs ----
  wire dump;
  wire [31:0] dump_count;
  wire signed [ACC_W-1:0] i_early, q_early, i_prompt, q_prompt, i_late, q_late;
  wire [15:0] epoch_dump;

  wire [15:0] tic_epoch;
  wire [10:0] tic_code_phase_halfchip;
  wire [9:0]  tic_code_nco_phase;
  wire [19:0] tic_carrier_cycle;
  wire [9:0]  tic_carrier_nco_phase;

  assign dbg_dump_pulse = dump;

  gps_ca_correlator_channel #(
    .PHASE_W(PHASE_W),
    .ACC_W(ACC_W)
  ) u_ch (
    .clk(samp_clk),
    .rstn(fe_samp_rstn),

    .enable(ch_enable),
    .prn(ch_prn),
    .carr_incr(ch_carr_incr),
    .code_incr(ch_code_incr),

    .slew_req(ch_slew_req),
    .slew_halfchips(ch_slew_halfchips),

    .tic_pulse(tic_pulse_samp),

    .fe_val(fe_val),

    .dump(dump),
    .dump_count(dump_count),

    .i_early(i_early),
    .q_early(q_early),
    .i_prompt(i_prompt),
    .q_prompt(q_prompt),
    .i_late(i_late),
    .q_late(q_late),

    .epoch(epoch_dump),

    .tic_epoch(tic_epoch),
    .tic_code_phase_halfchip(tic_code_phase_halfchip),
    .tic_code_nco_phase(tic_code_nco_phase),
    .tic_carrier_cycle(tic_carrier_cycle),
    .tic_carrier_nco_phase(tic_carrier_nco_phase)
  );

  gp2021_axi_wrapper #(
    .ACC_W(ACC_W)
  ) u_axi (
    .samp_clk(samp_clk),
    .samp_rstn(fe_samp_rstn),

    .ch_enable_samp(ch_enable),
    .ch_prn_samp(ch_prn),
    .ch_carr_incr_samp(ch_carr_incr),
    .ch_code_incr_samp(ch_code_incr),
    .ch_slew_req_samp(ch_slew_req),
    .ch_slew_halfchips_samp(ch_slew_halfchips),

    .dump_pulse(dump),
    .i_early(i_early),
    .q_early(q_early),
    .i_prompt(i_prompt),
    .q_prompt(q_prompt),
    .i_late(i_late),
    .q_late(q_late),
    .epoch_dump(epoch_dump),

    .tic_epoch(tic_epoch),
    .tic_code_phase_halfchip(tic_code_phase_halfchip),
    .tic_code_nco_phase(tic_code_nco_phase),
    .tic_carrier_cycle(tic_carrier_cycle),
    .tic_carrier_nco_phase(tic_carrier_nco_phase),

    .tic_pulse_samp(tic_pulse_samp),

    .axi_clk(axi_clk),
    .axi_rstn(axi_rstn),

    .s_axi_awaddr(s_axi_awaddr),
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_wdata(s_axi_wdata),
    .s_axi_wstrb(s_axi_wstrb),
    .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_bresp(s_axi_bresp),
    .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready),
    .s_axi_araddr(s_axi_araddr),
    .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready),
    .s_axi_rdata(s_axi_rdata),
    .s_axi_rresp(s_axi_rresp),
    .s_axi_rvalid(s_axi_rvalid),
    .s_axi_rready(s_axi_rready)
  );

endmodule
