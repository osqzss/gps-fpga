// tb_gps_ca_correlator_channel.v
// Icarus-friendly testbench:
// - Reads fe_val samples (+1,+3,-1,-3) line-by-line from gps_if.txt (default)
// - Drives DUT at 16.368 MHz sample clock
// - Writes Early/Prompt/Late I/Q accumulations at each DUMP to gps_dump.txt
//
// Build (example):
//   iverilog -g2012 -o sim tb_gps_ca_correlator_channel.v gps_ca_correlator_channel.v
// Run:
//   vvp sim
//
// Notes:
// - Set PRN, CARR_DOPPLER_HZ, CODE_DOPPLER_HZ, CODE_DELAY_HALFCHIPS to match your IF generator.
// - carr_lo frequency should match the IF carrier you simulated (nominal 4.092 MHz + Doppler).
// - code_incr is for HALF-CHIP ticking (nominal 2.046 MHz). Adjust with CODE_DOPPLER_HZ if you simulated it.
//
// If your simulator includes an initial code delay, you may want the DUT to start with an equivalent offset.
// This TB applies a one-shot slew_req, but in the current DUT implementation slew inhibits integration only.
// If you need code-phase shift via slew, modify the DUT to freeze code during slewing (as commented there).

`timescale 1ns/1ps

module tb_gps_ca_correlator_channel;

  // ---------------- User settings ----------------
  localparam int PHASE_W = 32;
  localparam int ACC_W   = 18;

  // Sample rate and nominal IF
  localparam real FS_HZ          = 16.368e6;
  localparam real IF_HZ_NOMINAL  = 4.092e6;   // nominal center frequency

  // Match your IF simulator settings:
  localparam int  PRN_ID               = 1;        // 1..32
  localparam real CARR_DOPPLER_HZ      = 1500.0;      // e.g., +2500.0
  localparam real CODE_DOPPLER_HZ      = -0.5e3;      // e.g., +1.0 (chips/sec), if simulated
  localparam int  CODE_DELAY_HALFCHIPS = 0;        // initial code delay (halfchips), if you want to request slew

  // Input/output file names (default gps_if.txt)
  localparam string INFILE  = "../sim/gps_if.txt";
  localparam string OUTFILE = "gps_dump.txt";

  // Derived frequencies
  localparam real CARR_WIPE_HZ   = IF_HZ_NOMINAL + CARR_DOPPLER_HZ; // LO frequency
  localparam real CHIP_HZ        = 1.023e6 + CARR_DOPPLER_HZ / 1540.0 + CODE_DOPPLER_HZ;       // half-chip ticking rate

  // ---------------- Clock generation ----------------
  // 16.368 MHz => period = 1/FS = 61.103... ns
  real TCLK_NS;
  initial TCLK_NS = 1.0e9 / FS_HZ;

  reg clk = 1'b0;
  always #(TCLK_NS/2.0) clk = ~clk;

  // ---------------- Reset/enable ----------------
  reg rstn = 1'b0;
  reg enable = 1'b0;

  // ---------------- DUT controls ----------------
  reg  [7:0]  prn;
  reg  [31:0] carr_incr;
  reg  [31:0] code_incr;

  reg         slew_req;
  reg  [10:0] slew_halfchips;

  // fe_val stimulus
  reg signed [2:0] fe_val;

  // DUT outputs
  wire dump;

  wire signed [ACC_W-1:0] i_early, q_early;
  wire signed [ACC_W-1:0] i_prompt, q_prompt;
  wire signed [ACC_W-1:0] i_late, q_late;

  wire [15:0] code_phase, carr_phase, epoch;
  wire [31:0] dump_count;

  // TIC-latched outputs
  wire tic_pulse_samp;
  wire [15:0] tic_epoch;
  wire [10:0] tic_code_phase_halfchip;
  wire [9:0]  tic_code_nco_phase;
  wire [19:0] tic_carrier_cycle;
  wire [9:0]  tic_carrier_nco_phase;

  // ---------------- DUT instantiation ----------------
  gps_ca_correlator_channel #(
    .PHASE_W(PHASE_W),
    .ACC_W(ACC_W)
  ) dut (
    .clk(clk),
    .rstn(rstn),
    .enable(enable),
    .prn(prn),
    .carr_incr(carr_incr),
    .code_incr(code_incr),
    .slew_req(slew_req),
    .slew_halfchips(slew_halfchips),
    .fe_val(fe_val),
    .dump(dump),
    .i_early(i_early),
    .q_early(q_early),
    .i_prompt(i_prompt),
    .q_prompt(q_prompt),
    .i_late(i_late),
    .q_late(q_late),
    .code_phase(code_phase),
    .carr_phase(carr_phase),
    .epoch(epoch),
    .dump_count(dump_count),
    .tic_pulse(tic_pulse_samp),
    .tic_epoch(tic_epoch),
    .tic_code_phase_halfchip(tic_code_phase_halfchip),
    .tic_code_nco_phase(tic_code_nco_phase),
    .tic_carrier_cycle(tic_carrier_cycle),
    .tic_carrier_nco_phase(tic_carrier_nco_phase)
  );

  // ---------------- Utility: compute NCO tuning word ----------------
  function automatic [31:0] nco_word(input real f_hz, input real fs_hz);
    real word_r;
    begin
      // word = round(2^PHASE_W * f/fs)
      word_r = (f_hz * (2.0**PHASE_W)) / fs_hz;
      if (word_r < 0.0) word_r = word_r + (2.0**PHASE_W); // wrap for negative
      nco_word = $rtoi(word_r);
    end
  endfunction

  // ---------------- File I/O ----------------
  integer fin, fout;
  integer rc;
  integer sample_i;
  integer stim_int;
  integer line_no;

  // For sanity timing: count *samp_clk* cycles between dumps
  integer samp_cycles_since_dump;

  initial begin
    // Init
    prn = PRN_ID[7:0];
    carr_incr = nco_word(CARR_WIPE_HZ, FS_HZ);
    code_incr = nco_word(CHIP_HZ,  FS_HZ);

    $display("[%0t] TB start", $time);
    $display("[%0t] FS_HZ=%f => TCLK_NS=%f", $time, FS_HZ, TCLK_NS);
    $display("[%0t] carr_incr=0x%08x code_incr=0x%08x", $time, carr_incr, code_incr);

    slew_req = 1'b0;
    slew_halfchips = CODE_DELAY_HALFCHIPS[10:0];

    fe_val = 3'sd0;
    line_no = 0;

    // Open files
    fin = $fopen(INFILE, "r");
    if (fin == 0) begin
      $display("ERROR: cannot open input file '%s'", INFILE);
      $finish;
    end
    fout = $fopen(OUTFILE, "w");
    if (fout == 0) begin
      $display("ERROR: cannot open output file '%s'", OUTFILE);
      $finish;
    end

    // Write header (replace your existing header write)
    $fwrite(fout,
      "# dump_count i_early q_early i_prompt q_prompt i_late q_late ie_pow ip_pow il_pow\n");

    // Reset sequence
    rstn = 1'b0;
    enable = 1'b0;
    repeat (20) @(posedge clk);
    rstn = 1'b1;

    // Optional one-shot slew request at start (see note in header)
    if (CODE_DELAY_HALFCHIPS != 0) begin
      @(posedge clk);
      slew_req <= 1'b1;
      @(posedge clk);
      slew_req <= 1'b0;
    end

    // Enable tracking
    repeat (5) @(posedge clk);
    enable = 1'b1;

    // For saniy timing
    samp_cycles_since_dump = 0;

    // Main loop: drive one fe_val per sample clock
    while (!$feof(fin)) begin
      rc = $fscanf(fin, "%d\n", stim_int);
      if (rc == 1) begin
        line_no = line_no + 1;
        // Clamp to allowed values (optional safety)
        case (stim_int)
          1, 3, -1, -3: fe_val <= stim_int;
          default: begin
            $display("WARN: line %0d has invalid fe_val=%0d (expected +/-1 or +/-3). Forcing 0.",
                     line_no, stim_int);
            fe_val <= 0;
          end
        endcase
      end

      // Now consume this sample at the next posedge
      @(posedge clk);
      samp_cycles_since_dump = samp_cycles_since_dump + 1;

      // On dump, write outputs
      if (dump) begin
        // Compute powers as 64-bit to avoid overflow: P = I^2 + Q^2
        longint ie_pow, ip_pow, il_pow;
        longint ie_i, ie_q, ip_i, ip_q, il_i, il_q;

        ie_i = i_early;   ie_q = q_early;
        ip_i = i_prompt;  ip_q = q_prompt;
        il_i = i_late;    il_q = q_late;

        ie_pow = ie_i*ie_i + ie_q*ie_q;
        ip_pow = ip_i*ip_i + ip_q*ip_q;
        il_pow = il_i*il_i + il_q*il_q;

        $display("[%0t] SANITY samp_clk cycles since last dump = %0d (expect 16368)",
                     $time, samp_cycles_since_dump);
        samp_cycles_since_dump = 0;

        $display("[%0t] IE=%0d QE=%0d IP=%0d QP=%0d IL=%0d QL=%0d",
                     $time,
                     $signed(i_early),  $signed(q_early),
                     $signed(i_prompt), $signed(q_prompt),
                     $signed(i_late),   $signed(q_late));

        $fwrite(fout,
          "%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d\n",
          dump_count,
          i_early, q_early,
          i_prompt, q_prompt,
          i_late, q_late,
          ie_pow, ip_pow, il_pow
        );

        if (ie_pow > 40000000) begin
          code_incr = 32'h1000_0000;
          $display("[%0t] Signal was found. Set code_incr=0x%08x", $time, code_incr);
        end
      end
    end

    // Flush a few more cycles to catch last dump if needed
    repeat (50) @(posedge clk);

    $display("Done. Read %0d samples from %s, wrote dumps to %s",
             line_no, INFILE, OUTFILE);
    $fclose(fin);
    $fclose(fout);
    $finish;
  end

endmodule
