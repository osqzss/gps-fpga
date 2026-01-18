# gps-fpga
Hardware-ready GPS L1 C/A single-channel correlator for Zynq/MicroBlaze.
Takes 2-bit sign/magnitude IF samples from an external RF front-end and produces early/prompt/late I/Q accumulations. A lightweight AXI4-Lite register map provides channel control (PRN, carrier/code NCO, dump timing) and readout of the latest correlation dumps.
