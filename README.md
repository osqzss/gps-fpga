# gps-fpga
Hardware-ready GPS L1 C/A single-channel correlator for Zynq/MicroBlaze.
Takes 2-bit sign/magnitude IF samples from an external RF front-end and produces early/prompt/late I/Q accumulations. A lightweight AXI4-Lite register map provides channel control (PRN, carrier/code NCO, dump timing) and readout of the latest correlation dumps.

## Quick Vivado instructions

### A. Creae project + add RTL

1. **Create Project**
2. Add sources:
    * `top_singlech_axi_hw.sv`
    * `gp2021_axi_wrapper.sv`
    * `gps_ca_correlator_channel.sv`

### B. Package you RTL as a custom IP

1. **Tools**  --> **Create and Package New IP**
2. "Package your current project"
3. In the IP packager:
    * Select top module: top_singlech_axi_hw
    * Make sure your AXI-lite interface ports are recognized as **AXI4-Lite Slave**
    * Expose non-AXI ports as external:
        * `fe_samp_clk_in`, `fe_sign`, `fe_mag`
4. Then **Re-package IP**

### C. Block Design

1. **Create Block Design**
2. Add:
    * MicroBlaze
    * AXI Interconnect
    * Clocking Wizard
    * Processor System Reset
    * Your packaged IP (e.g. `top_singlech_axi_hw_0`)
3. Connect:
    * MicroBlaze `M_AXI_DP` --> AXI Interconnect --> Your IP `S_AXI`
    * Clock/reset:
        * `axi_clk` for your IP from the same clock as MicroBlaze AXI clock
        * `axi_rstn` from Processor System Reset (active-low)
4. External ports:
    * Make `fe_samp_clk_in`, `fe_sign`, `fe_mag` external and match XDC port names

