#!/bin/sh

if [ -f "sim" ]; then
  rm sim
fi

# Run Icarus verilog:
iverilog -g2012 -Wall -o sim \
  ../test/tb_singlech_axi.sv \
  ../rtl/gps_ca_correlator_channel.v \
  ../rtl/gp2021_axi_wrapper.sv \
  ../rtl/top_singlech_axi_sim.sv

vvp sim

