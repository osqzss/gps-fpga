// tb_singlech_axi.sv (Option A)
// Icarus-friendly TB for top_singlech_axi_sim where channel controls are via AXI-lite.
//
// Build (from your script):
//   iverilog -g2012 -Wall -o sim \
//     ../test/tb_singlech_axi.sv \
//     ../rtl/gps_ca_correlator_channel.sv \
//     ../rtl/gp2021_axi_wrapper.sv \
//     ../rtl/top_singlech_axi_sim.sv
// Run:
//   vvp sim

`timescale 1ns/1ps

module tb_singlech_axi;

  // ---------------- User settings ----------------
  localparam int PHASE_W = 32;
  localparam int ACC_W   = 18;

  // Sample rate and nominal IF
  localparam real FS_HZ          = 16.368e6;
  localparam real IF_HZ_NOMINAL  = 4.092e6;   // nominal center frequency

  // Match your IF simulator settings:
  localparam int  PRN_ID               = 1;
  localparam real CARR_DOPPLER_HZ      = 1500.0;
  localparam real CODE_DOPPLER_HZ      = 0.0; // to search code delay

  // Derived frequencies
  localparam real CARR_WIPE_HZ    = IF_HZ_NOMINAL + CARR_DOPPLER_HZ; // LO frequency
  localparam real CHIP_HZ_NOMINAL = 1.023e6 + CARR_DOPPLER_HZ / 1540.0; 
  localparam real CHIP_HZ         = CHIP_HZ_NOMINAL + CODE_DOPPLER_HZ; 

  // Input/output file names
  localparam string INFILE  = "../sim/gps_if.txt";
  localparam string OUTFILE = "gps_dump_axi.txt";

  // ---------------- Clocks ----------------
  localparam real TCLK_NS = 1e9 / FS_HZ;

  reg samp_clk = 1'b0;
  reg axi_clk  = 1'b0;
  always #(TCLK_NS/2.0) samp_clk = ~samp_clk;
  always #5 axi_clk = ~axi_clk; // 100 MHz

  // ---------------- Resets ----------------
  reg samp_rstn = 1'b0;
  reg axi_rstn  = 1'b0;

  // ---------------- Frontend sample ----------------
  reg signed [2:0] fe_val;

  // ---------------- AXI-Lite (master in TB) ----------------
  reg  [31:0] awaddr;
  reg         awvalid;
  wire        awready;
  reg  [31:0] wdata;
  reg  [3:0]  wstrb;
  reg         wvalid;
  wire        wready;
  wire [1:0]  bresp;
  wire        bvalid;
  reg         bready;

  reg  [31:0] araddr;
  reg         arvalid;
  wire        arready;
  wire [31:0] rdata;
  wire [1:0]  rresp;
  wire        rvalid;
  reg         rready;

  // ---------------- DUT ----------------
  top_singlech_axi_sim #(.PHASE_W(PHASE_W), .ACC_W(ACC_W)) dut (
    .samp_clk(samp_clk),
    .samp_rstn(samp_rstn),
    .axi_clk(axi_clk),
    .axi_rstn(axi_rstn),
    .fe_val(fe_val),

    .s_axi_awaddr(awaddr),
    .s_axi_awvalid(awvalid),
    .s_axi_awready(awready),
    .s_axi_wdata(wdata),
    .s_axi_wstrb(wstrb),
    .s_axi_wvalid(wvalid),
    .s_axi_wready(wready),
    .s_axi_bresp(bresp),
    .s_axi_bvalid(bvalid),
    .s_axi_bready(bready),
    .s_axi_araddr(araddr),
    .s_axi_arvalid(arvalid),
    .s_axi_arready(arready),
    .s_axi_rdata(rdata),
    .s_axi_rresp(rresp),
    .s_axi_rvalid(rvalid),
    .s_axi_rready(rready)
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

  // ---------------- Simple AXI tasks ----------------
  task automatic axi_write32(input [31:0] addr, input [31:0] data);
    integer guard;
    begin
      @(posedge axi_clk);
      awaddr  <= addr;
      awvalid <= 1'b1;
      wdata   <= data;
      wstrb   <= 4'hF;
      wvalid  <= 1'b1;
      bready  <= 1'b1;

      guard = 0;
      while (!(awready && awvalid)) begin
        @(posedge axi_clk);
        guard = guard + 1;
        if (guard > 100000) begin
          $display("[%0t] AXI_WRITE TIMEOUT AW addr=0x%08x", $time, addr);
          $finish;
        end
      end

      guard = 0;
      while (!(wready && wvalid)) begin
        @(posedge axi_clk);
        guard = guard + 1;
        if (guard > 100000) begin
          $display("[%0t] AXI_WRITE TIMEOUT W addr=0x%08x", $time, addr);
          $finish;
        end
      end

      @(posedge axi_clk);
      awvalid <= 1'b0;
      wvalid  <= 1'b0;

      guard = 0;
      while (!bvalid) begin
        @(posedge axi_clk);
        guard = guard + 1;
        if (guard > 100000) begin
          $display("[%0t] AXI_WRITE TIMEOUT B addr=0x%08x", $time, addr);
          $finish;
        end
      end

      @(posedge axi_clk);
      bready <= 1'b0;
    end
  endtask

  task automatic axi_read32(input [31:0] addr, output [31:0] data);
    integer guard;
    begin
      @(posedge axi_clk);
      araddr  <= addr;
      arvalid <= 1'b1;
      rready  <= 1'b1;

      guard = 0;
      while (!(arready && arvalid)) begin
        @(posedge axi_clk);
        guard = guard + 1;
        if (guard > 100000) begin
          $display("[%0t] AXI_READ TIMEOUT AR addr=0x%08x", $time, addr);
          $finish;
        end
      end

      @(posedge axi_clk);
      arvalid <= 1'b0;

      guard = 0;
      while (!rvalid) begin
        @(posedge axi_clk);
        guard = guard + 1;
        if (guard > 100000) begin
          $display("[%0t] AXI_READ TIMEOUT R addr=0x%08x", $time, addr);
          $finish;
        end
      end

      data = rdata;
      @(posedge axi_clk);
      rready <= 1'b0;
    end
  endtask

  // ---------------- Register map ----------------
  localparam [31:0] REG_PROG_TIC_CYCLES = 32'h0000_0008;
  localparam [31:0] REG_CONTROL         = 32'h0000_000C;

  localparam [31:0] REG_CH0_PRN          = 32'h0000_0020;
  localparam [31:0] REG_CH0_CARR_INCR    = 32'h0000_0024;
  localparam [31:0] REG_CH0_CODE_INCR    = 32'h0000_0028;
  localparam [31:0] REG_CH0_SLEW_HC      = 32'h0000_002C;
  localparam [31:0] REG_CH0_SLEW_REQ     = 32'h0000_0030;

  // dump regs (ch0 window 0x0100)
  localparam [31:0] CH0_BASE     = 32'h0000_0100;
  localparam [31:0] REG_DUMP_SEQ = CH0_BASE + 32'h00;
  localparam [31:0] REG_IE       = CH0_BASE + 32'h04;
  localparam [31:0] REG_QE       = CH0_BASE + 32'h08;
  localparam [31:0] REG_IP       = CH0_BASE + 32'h0C;
  localparam [31:0] REG_QP       = CH0_BASE + 32'h10;
  localparam [31:0] REG_IL       = CH0_BASE + 32'h14;
  localparam [31:0] REG_QL       = CH0_BASE + 32'h18;

  // ---------------- IF feeding ----------------
  integer fi;
  integer rc;
  integer samp;
  integer zeros;
  integer bad_rc;

  // write dump file
  integer dumpfo;

  // Dump monitoring
  reg [31:0] dump_seq_prev, dump_seq_now;
  reg [31:0] r_ie, r_qe, r_ip, r_qp, r_il, r_ql;

  integer last_dump_samp;

  function automatic integer pow2(input integer a, input integer b);
    begin
      pow2 = a*a + b*b;
    end
  endfunction

  // ---------------- Threads ----------------

  task automatic feed_if_thread;
    integer tmp;
    begin
      samp = 0;
      zeros = 0;
      bad_rc = 0;

      $display("[%0t] FEED_IF start (continuous, no gaps)", $time);

      // We already preloaded IF[1] into fe_val.
      while (!$feof(fi)) begin
        rc = $fscanf(fi, "%d\n", tmp);
        if (rc != 1) begin
          bad_rc = bad_rc + 1;
          fe_val <= 0;
        end else begin
          fe_val <= tmp[2:0];
          if (tmp == 0) zeros = zeros + 1;
        end

        samp = samp + 1;
        //if (samp <= 16) $display("[%0t] IF[%0d]=%0d", $time, samp+1, tmp);

        @(posedge samp_clk);

        if ((samp % 16368) == 0) begin
          $display("[%0t] FEED_IF samp=%0d fe_val=%0d bad_rc=%0d zeros=%0d", $time, samp, fe_val, bad_rc, zeros);
        end
      end

      $display("[%0t] FEED_IF reached EOF after samp=%0d bad_rc=%0d zeros=%0d", $time, samp, bad_rc, zeros);
    end
  endtask

  task automatic axi_poll_thread;
    integer ie_i, qe_i, ip_i, qp_i, il_i, ql_i;
    integer ppow, ie_pow, ip_pow, il_pow;
    integer samples_since_last;
    begin
      axi_read32(REG_DUMP_SEQ, dump_seq_prev);
      $display("[%0t] AXI_POLL start dump_seq_prev=%0d", $time, dump_seq_prev);

      forever begin
        // Poll every ~64 samples
        repeat (64) @(posedge samp_clk);

        axi_read32(REG_DUMP_SEQ, dump_seq_now);
        if (dump_seq_now != dump_seq_prev) begin
          dump_seq_prev = dump_seq_now;

          samples_since_last = (last_dump_samp < 0) ? -1 : (samp - last_dump_samp);
          last_dump_samp = samp;

          $display("[%0t] DUMP seen: dump_seq=%0d", $time, dump_seq_now);
          if (samples_since_last >= 0)
            $display("[%0t] SANITY samples since last dump = %0d (expect 16368)", $time, samples_since_last);

          axi_read32(REG_IE, r_ie);
          axi_read32(REG_QE, r_qe);
          axi_read32(REG_IP, r_ip);
          axi_read32(REG_QP, r_qp);
          axi_read32(REG_IL, r_il);
          axi_read32(REG_QL, r_ql);

          // sign-extend into integer
          ie_i = $signed(r_ie);
          qe_i = $signed(r_qe);
          ip_i = $signed(r_ip);
          qp_i = $signed(r_qp);
          il_i = $signed(r_il);
          ql_i = $signed(r_ql);

          ppow   = pow2(ip_i, qp_i);
          ie_pow = pow2(ie_i, qe_i);
          ip_pow = ppow;
          il_pow = pow2(il_i, ql_i);

          $display("[%0t] AXI DUMP dump_seq=%0d IE=%0d QE=%0d IP=%0d QP=%0d (Ppow=%0d)",
                   $time, dump_seq_now, ie_i, qe_i, ip_i, qp_i, ppow);

          // Direct (hier) snapshot from top (sample-domain)
          $display("[%0t] DIRECT (hier) IE=%0d QE=%0d IP=%0d QP=%0d IL=%0d QL=%0d",
                   $time, dut.i_early, dut.q_early, dut.i_prompt, dut.q_prompt, dut.i_late, dut.q_late);

          $fwrite(dumpfo, "%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d\n",
                  dump_seq_now, ie_i, qe_i, ip_i, qp_i, il_i, ql_i, ie_pow, ip_pow, il_pow);
          /*
          if (ie_pow > 40000000) begin
            code_incr = nco_word(CHIP_HZ_NOMINAL, FS_HZ);
            axi_write32(REG_CH0_CODE_INCR, code_incr);
            $display("[%0t] >>Signal was found<< code_incr=0x%08x", $time, code_incr);
          end
          */
        end
      end
    end
  endtask

  // Direct dump pulse monitor (samp_clk domain)
  always @(posedge samp_clk) begin
    if (!samp_rstn) begin
      last_dump_samp <= -1;
    end else begin
      if (dut.dump_pulse) begin
        if (last_dump_samp >= 0) begin
          $display("[%0t] DIRECT-DUMP dump_seq=%0d samples_since_last=%0d  IP=%0d QP=%0d",
                   $time, dut.dump_seq_samp, (samp - last_dump_samp), dut.i_prompt, dut.q_prompt);
        end else begin
          $display("[%0t] DIRECT-DUMP dump_seq=%0d (first) IP=%0d QP=%0d",
                   $time, dut.dump_seq_samp, dut.i_prompt, dut.q_prompt);
        end
        last_dump_samp <= samp;
      end
    end
  end

  // ---------------- Main initial ----------------
  reg  [7:0]  prn;
  reg  [31:0] carr_incr;
  reg  [31:0] code_incr;

  initial begin
    // init signals
    fe_val = 3'sd0;
    awaddr=0; awvalid=0; wdata=0; wstrb=0; wvalid=0; bready=0;
    araddr=0; arvalid=0; rready=0;

    $display("[%0t] TB start", $time);
    //$display("[%0t] FS_HZ=%f => TCLK_NS=%f", $time, FS_HZ, TCLK_NS);

    // Reset
    samp_rstn = 1'b0;
    axi_rstn  = 1'b0;

    repeat (20) @(posedge axi_clk);
    axi_rstn = 1'b1;
    $display("[%0t] Deassert axi_rstn", $time);

    // Program TIC regs (0.1s)
    $display("[%0t] Programming TIC regs via AXI", $time);
    axi_write32(REG_PROG_TIC_CYCLES, 32'd1636800);
    axi_write32(REG_CONTROL,         32'd1);

    // Program channel controls (NCOs etc) BEFORE enabling PRN.
    carr_incr = nco_word(CARR_WIPE_HZ, FS_HZ);
    code_incr = nco_word(CHIP_HZ, FS_HZ);
    axi_write32(REG_CH0_CARR_INCR, carr_incr);
    axi_write32(REG_CH0_CODE_INCR, code_incr);
    //axi_write32(REG_CH0_SLEW_HC,   32'd0); // No code slew
    $display("[%0t] carr_incr=0x%08x code_incr=0x%08x", $time, carr_incr, code_incr);

    // Open IF + dump files
    $display("[%0t] Opening IF file %s", $time, INFILE);
    fi = $fopen(INFILE, "r");
    if (fi == 0) begin
      $display("[%0t] ERROR: cannot open IF file", $time);
      $finish;
    end

    dumpfo = $fopen(OUTFILE, "w");
    if (dumpfo == 0) begin
      $display("[%0t] ERROR: cannot open output file %s", $time, OUTFILE);
      $finish;
    end
    $fwrite(dumpfo, "# dump_seq ie qe ip qp il ql ie_pow ip_pow il_pow\n");

    // Preload IF[1]
    rc = $fscanf(fi, "%d\n", fe_val);
    if (rc != 1) fe_val = 0;
    //$display("[%0t] Preload IF[1]=%0d into fe_val before releasing samp_rstn", $time, fe_val);

    // Release sampling reset on a clean edge.
    @(negedge samp_clk);
    samp_rstn = 1'b1;

    // Allow CDC of control regs into samp domain.
    repeat (8) @(posedge samp_clk);

    // Enable channel by writing PRN!=0.
    prn = PRN_ID[7:0];
    axi_write32(REG_CH0_PRN, prn);

    $display("[%0t] Deassert samp_rstn; channel enabled by PRN write", $time);

    last_dump_samp = -1;

    fork
      feed_if_thread();
      axi_poll_thread();
    join_any

    repeat (50000) @(posedge axi_clk);
    $display("[%0t] Done. Wrote gps_dump_axi.txt", $time);
    $finish;
  end

endmodule
