#!/bin/sh

if [ -f "sim" ]; then
  rm sim
fi

# Run Icarus verilog:
iverilog -g2012 -Wall -o sim \
  ../test/tb_gps_ca_correlator_channel.v \
  ../rtl/gps_ca_correlator_channel.v

vvp sim
