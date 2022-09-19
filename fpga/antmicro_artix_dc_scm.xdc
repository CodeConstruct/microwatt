################################################################################
# clkin, reset, uart pins...
################################################################################

set_property -dict { PACKAGE_PIN C18  IOSTANDARD LVCMOS33 } [get_ports { ext_clk }];

set_property -dict { PACKAGE_PIN R19 IOSTANDARD LVCMOS33 } [get_ports { uart_main_tx }];
set_property -dict { PACKAGE_PIN P19  IOSTANDARD LVCMOS33 } [get_ports { uart_main_rx }];

#set_property -dict { PACKAGE_PIN T20 IOSTANDARD LVCMOS33 } [get_ports { d11_led }];
#set_property -dict { PACKAGE_PIN U20 IOSTANDARD LVCMOS33 } [get_ports { d12_led }];
#set_property -dict { PACKAGE_PIN W20 IOSTANDARD LVCMOS33 } [get_ports { d13_led }];

################################################################################
# User LEDs
################################################################################
set_property -dict { PACKAGE_PIN T20 IOSTANDARD LVCMOS33 } [get_ports {usr_led[0]}];
set_property -dict { PACKAGE_PIN U20 IOSTANDARD LVCMOS33 } [get_ports {usr_led[1]}];
set_property -dict { PACKAGE_PIN W20 IOSTANDARD LVCMOS33 } [get_ports {usr_led[2]}];

################################################################################
# DC-SCM GPIOs from spec
################################################################################
set_property -dict { PACKAGE_PIN U2 IOSTANDARD LVCMOS33 } [get_ports { sys_pwrbtn_n }];
set_property -dict { PACKAGE_PIN V3 IOSTANDARD LVCMOS33 } [get_ports { sys_pwrok }];
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports { hpm_stby_en }];

################################################################################
# Platform specific GPIOs
################################################################################
set_property -dict { PACKAGE_PIN D19 IOSTANDARD LVCMOS33 } [get_ports { fsi_clk }];
set_property -dict { PACKAGE_PIN F18 IOSTANDARD LVCMOS33 } [get_ports { fsi_dat }];
set_property -dict { PACKAGE_PIN U1 IOSTANDARD LVCMOS33 } [get_ports { bmc_fsi_in_ena }];

################################################################################
# TPM J4 header, for GPIO rework and litescope
################################################################################
set_property -dict { PACKAGE_PIN D16 IOSTANDARD LVCMOS33 } [get_ports { spi0_cs_n }]; # 1
set_property -dict { PACKAGE_PIN A16 IOSTANDARD LVCMOS33 } [get_ports { spi0_mosi }]; # 2
set_property -dict { PACKAGE_PIN B16 IOSTANDARD LVCMOS33 } [get_ports { spi0_miso }]; # 3
set_property -dict { PACKAGE_PIN E16 IOSTANDARD LVCMOS33 } [get_ports { spi0_clk }]; #  4
set_property -dict { PACKAGE_PIN G17 IOSTANDARD LVCMOS33 } [get_ports { tpm_pirq }]; #  7
set_property -dict { PACKAGE_PIN V22 IOSTANDARD LVCMOS33 } [get_ports { tpm_rst }]; # 8
set_property -dict { PACKAGE_PIN Y21 IOSTANDARD LVCMOS33 } [get_ports { tpm_gpio }]; #  9
set_property -dict { PACKAGE_PIN G18 IOSTANDARD LVCMOS33 } [get_ports { tpm_pp }];   #  10

################################################################################
# Connects to edge connector A12
# Unused, A12 edge connector should be i2c5 instead.
################################################################################
# set_property -dict { PACKAGE_PIN F6 IOSTANDARD LVCMOS33 } [get_ports { pcie_bmc_clk_100_p }];

################################################################################
# I2C - as numbered on dc-scm schematic
################################################################################
# not exposed by interposer
set_property -dict { PACKAGE_PIN J16 IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[0]}];
set_property -dict { PACKAGE_PIN J15 IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[0]}];

set_property -dict { PACKAGE_PIN L15 IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[1]}];
set_property -dict { PACKAGE_PIN M15 IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[1]}];
set_property -dict { PACKAGE_PIN L14 IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[2]}];
set_property -dict { PACKAGE_PIN J14 IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[2]}];
set_property -dict { PACKAGE_PIN K14 IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[3]}];
set_property -dict { PACKAGE_PIN M13 IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[3]}];
set_property -dict { PACKAGE_PIN K13 IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[4]}];
set_property -dict { PACKAGE_PIN L13 IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[4]}];

# incorrect pins on dcscm. scl was to ground, has been disconnected
set_property -dict { PACKAGE_PIN N13 IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[5]}];
# i2c_sda5 on edge A12 now uses pcie_bmc_clk_100_p, see above
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[5]}];

set_property -dict { PACKAGE_PIN H13 IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[6]}];
set_property -dict { PACKAGE_PIN G13 IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[6]}];
set_property -dict { PACKAGE_PIN F13 IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[7]}];
set_property -dict { PACKAGE_PIN E13 IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[7]}];
set_property -dict { PACKAGE_PIN C13 IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[8]}];
set_property -dict { PACKAGE_PIN C14 IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[8]}];

# note that interposer swaps scl/sda, swapped back in vhdl
set_property -dict { PACKAGE_PIN T5  IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[9]}];
set_property -dict { PACKAGE_PIN T6  IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[9]}];
# note that interposer swaps scl/sda, swapped back in vhdl
set_property -dict { PACKAGE_PIN U6  IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[10]}];
set_property -dict { PACKAGE_PIN W6  IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[10]}];

set_property -dict { PACKAGE_PIN W7  IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[11]}];
set_property -dict { PACKAGE_PIN W9  IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[11]}];
set_property -dict { PACKAGE_PIN V8  IOSTANDARD LVCMOS33 } [get_ports {i2c_scl[12]}];
set_property -dict { PACKAGE_PIN V9  IOSTANDARD LVCMOS33 } [get_ports {i2c_sda[12]}];

################################################################################
# SPI Flash
################################################################################
# P22 DQ0
# R22 DQ1
# P21 DQ2
# R21 DQ3
# T19 CS_B

set_property -dict { PACKAGE_PIN T19 IOSTANDARD LVCMOS33 } [get_ports { spi_flash_cs_n }];
#set_property -dict { PACKAGE_PIN L12 IOSTANDARD LVCMOS33 } [get_ports { spi_flash_clk }];
set_property -dict { PACKAGE_PIN P22 IOSTANDARD LVCMOS33 } [get_ports { spi_flash_mosi }];
set_property -dict { PACKAGE_PIN R22 IOSTANDARD LVCMOS33 } [get_ports { spi_flash_miso }];
set_property -dict { PACKAGE_PIN P21 IOSTANDARD LVCMOS33 } [get_ports { spi_flash_wp_n }];
set_property -dict { PACKAGE_PIN R21 IOSTANDARD LVCMOS33 } [get_ports { spi_flash_hold_n }];

# Put registers into IOBs to improve timing
set_property IOB true [get_cells -hierarchical -filter {NAME =~*/spi_rxtx/*sck_1*}]
set_property IOB true [get_cells -hierarchical -filter {NAME =~*/spi_rxtx/input_delay_1.dat_i_l*}]

################################################################################
# DRAM (generated by LiteX)
################################################################################

# ddram:0.a
set_property LOC M2 [get_ports {ddram_a[0]}]
set_property SLEW FAST [get_ports {ddram_a[0]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[0]}]

# ddram:0.a
set_property LOC M5 [get_ports {ddram_a[1]}]
set_property SLEW FAST [get_ports {ddram_a[1]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[1]}]

# ddram:0.a
set_property LOC M3 [get_ports {ddram_a[2]}]
set_property SLEW FAST [get_ports {ddram_a[2]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[2]}]

# ddram:0.a
set_property LOC M1 [get_ports {ddram_a[3]}]
set_property SLEW FAST [get_ports {ddram_a[3]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[3]}]

# ddram:0.a
set_property LOC L6 [get_ports {ddram_a[4]}]
set_property SLEW FAST [get_ports {ddram_a[4]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[4]}]

# ddram:0.a
set_property LOC P1 [get_ports {ddram_a[5]}]
set_property SLEW FAST [get_ports {ddram_a[5]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[5]}]

# ddram:0.a
set_property LOC N3 [get_ports {ddram_a[6]}]
set_property SLEW FAST [get_ports {ddram_a[6]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[6]}]

# ddram:0.a
set_property LOC N2 [get_ports {ddram_a[7]}]
set_property SLEW FAST [get_ports {ddram_a[7]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[7]}]

# ddram:0.a
set_property LOC M6 [get_ports {ddram_a[8]}]
set_property SLEW FAST [get_ports {ddram_a[8]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[8]}]

# ddram:0.a
set_property LOC R1 [get_ports {ddram_a[9]}]
set_property SLEW FAST [get_ports {ddram_a[9]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[9]}]

# ddram:0.a
set_property LOC L5 [get_ports {ddram_a[10]}]
set_property SLEW FAST [get_ports {ddram_a[10]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[10]}]

# ddram:0.a
set_property LOC N5 [get_ports {ddram_a[11]}]
set_property SLEW FAST [get_ports {ddram_a[11]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[11]}]

# ddram:0.a
set_property LOC N4 [get_ports {ddram_a[12]}]
set_property SLEW FAST [get_ports {ddram_a[12]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[12]}]

# ddram:0.a
set_property LOC P2 [get_ports {ddram_a[13]}]
set_property SLEW FAST [get_ports {ddram_a[13]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[13]}]

# ddram:0.a
set_property LOC P6 [get_ports {ddram_a[14]}]
set_property SLEW FAST [get_ports {ddram_a[14]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_a[14]}]

# ddram:0.ba
set_property LOC L3 [get_ports {ddram_ba[0]}]
set_property SLEW FAST [get_ports {ddram_ba[0]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_ba[0]}]

# ddram:0.ba
set_property LOC K6 [get_ports {ddram_ba[1]}]
set_property SLEW FAST [get_ports {ddram_ba[1]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_ba[1]}]

# ddram:0.ba
set_property LOC L4 [get_ports {ddram_ba[2]}]
set_property SLEW FAST [get_ports {ddram_ba[2]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_ba[2]}]

# ddram:0.ras_n
set_property LOC J4 [get_ports {ddram_ras_n}]
set_property SLEW FAST [get_ports {ddram_ras_n}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_ras_n}]

# ddram:0.cas_n
set_property LOC K3 [get_ports {ddram_cas_n}]
set_property SLEW FAST [get_ports {ddram_cas_n}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_cas_n}]

# ddram:0.we_n
set_property LOC L1 [get_ports {ddram_we_n}]
set_property SLEW FAST [get_ports {ddram_we_n}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_we_n}]

# ddram:0.dm
set_property LOC G3 [get_ports {ddram_dm[0]}]
set_property SLEW FAST [get_ports {ddram_dm[0]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dm[0]}]

# ddram:0.dm
set_property LOC F1 [get_ports {ddram_dm[1]}]
set_property SLEW FAST [get_ports {ddram_dm[1]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dm[1]}]

# ddram:0.dq
set_property LOC G2 [get_ports {ddram_dq[0]}]
set_property SLEW FAST [get_ports {ddram_dq[0]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[0]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[0]}]

# ddram:0.dq
set_property LOC H4 [get_ports {ddram_dq[1]}]
set_property SLEW FAST [get_ports {ddram_dq[1]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[1]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[1]}]

# ddram:0.dq
set_property LOC H5 [get_ports {ddram_dq[2]}]
set_property SLEW FAST [get_ports {ddram_dq[2]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[2]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[2]}]

# ddram:0.dq
set_property LOC J1 [get_ports {ddram_dq[3]}]
set_property SLEW FAST [get_ports {ddram_dq[3]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[3]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[3]}]

# ddram:0.dq
set_property LOC K1 [get_ports {ddram_dq[4]}]
set_property SLEW FAST [get_ports {ddram_dq[4]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[4]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[4]}]

# ddram:0.dq
set_property LOC H3 [get_ports {ddram_dq[5]}]
set_property SLEW FAST [get_ports {ddram_dq[5]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[5]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[5]}]

# ddram:0.dq
set_property LOC H2 [get_ports {ddram_dq[6]}]
set_property SLEW FAST [get_ports {ddram_dq[6]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[6]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[6]}]

# ddram:0.dq
set_property LOC J5 [get_ports {ddram_dq[7]}]
set_property SLEW FAST [get_ports {ddram_dq[7]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[7]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[7]}]

# ddram:0.dq
set_property LOC E3 [get_ports {ddram_dq[8]}]
set_property SLEW FAST [get_ports {ddram_dq[8]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[8]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[8]}]

# ddram:0.dq
set_property LOC B2 [get_ports {ddram_dq[9]}]
set_property SLEW FAST [get_ports {ddram_dq[9]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[9]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[9]}]

# ddram:0.dq
set_property LOC F3 [get_ports {ddram_dq[10]}]
set_property SLEW FAST [get_ports {ddram_dq[10]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[10]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[10]}]

# ddram:0.dq
set_property LOC D2 [get_ports {ddram_dq[11]}]
set_property SLEW FAST [get_ports {ddram_dq[11]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[11]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[11]}]

# ddram:0.dq
set_property LOC C2 [get_ports {ddram_dq[12]}]
set_property SLEW FAST [get_ports {ddram_dq[12]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[12]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[12]}]

# ddram:0.dq
set_property LOC A1 [get_ports {ddram_dq[13]}]
set_property SLEW FAST [get_ports {ddram_dq[13]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[13]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[13]}]

# ddram:0.dq
set_property LOC E2 [get_ports {ddram_dq[14]}]
set_property SLEW FAST [get_ports {ddram_dq[14]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[14]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[14]}]

# ddram:0.dq
set_property LOC B1 [get_ports {ddram_dq[15]}]
set_property SLEW FAST [get_ports {ddram_dq[15]}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_dq[15]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dq[15]}]

# ddram:0.dqs_p
set_property LOC K2 [get_ports {ddram_dqs_p[0]}]
set_property SLEW FAST [get_ports {ddram_dqs_p[0]}]
set_property IOSTANDARD DIFF_SSTL135 [get_ports {ddram_dqs_p[0]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dqs_p[0]}]

# ddram:0.dqs_p
set_property LOC E1 [get_ports {ddram_dqs_p[1]}]
set_property SLEW FAST [get_ports {ddram_dqs_p[1]}]
set_property IOSTANDARD DIFF_SSTL135 [get_ports {ddram_dqs_p[1]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dqs_p[1]}]

# ddram:0.dqs_n
set_property LOC J2 [get_ports {ddram_dqs_n[0]}]
set_property SLEW FAST [get_ports {ddram_dqs_n[0]}]
set_property IOSTANDARD DIFF_SSTL135 [get_ports {ddram_dqs_n[0]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dqs_n[0]}]

# ddram:0.dqs_n
set_property LOC D1 [get_ports {ddram_dqs_n[1]}]
set_property SLEW FAST [get_ports {ddram_dqs_n[1]}]
set_property IOSTANDARD DIFF_SSTL135 [get_ports {ddram_dqs_n[1]}]
set_property IN_TERM UNTUNED_SPLIT_40 [get_ports {ddram_dqs_n[1]}]

# ddram:0.clk_p
set_property LOC P5 [get_ports {ddram_clk_p}]
set_property SLEW FAST [get_ports {ddram_clk_p}]
set_property IOSTANDARD DIFF_SSTL135 [get_ports {ddram_clk_p}]

# ddram:0.clk_n
set_property LOC P4 [get_ports {ddram_clk_n}]
set_property SLEW FAST [get_ports {ddram_clk_n}]
set_property IOSTANDARD DIFF_SSTL135 [get_ports {ddram_clk_n}]

# ddram:0.cke
set_property LOC J6 [get_ports {ddram_cke}]
set_property SLEW FAST [get_ports {ddram_cke}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_cke}]

# ddram:0.odt
set_property LOC K4 [get_ports {ddram_odt}]
set_property SLEW FAST [get_ports {ddram_odt}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_odt}]

# ddram:0.reset_n
set_property LOC G1 [get_ports {ddram_reset_n}]
set_property SLEW FAST [get_ports {ddram_reset_n}]
set_property IOSTANDARD SSTL135 [get_ports {ddram_reset_n}]

################################################################################
# Ethernet (generated by LiteX)
################################################################################

# eth_clocks:0.tx
set_property LOC J19 [get_ports {eth_clocks_tx}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_clocks_tx}]

# eth_clocks:0.rx
set_property LOC K19 [get_ports {eth_clocks_rx}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_clocks_rx}]

# eth:0.rst_n
set_property LOC N18 [get_ports {eth_rst_n}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_rst_n}]

# eth:0.int_n
set_property LOC N20 [get_ports {eth_int_n}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_int_n}]

# eth:0.mdio
set_property LOC M21 [get_ports {eth_mdio}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_mdio}]

# eth:0.mdc
set_property LOC N22 [get_ports {eth_mdc}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_mdc}]

# eth:0.rx_ctl
set_property LOC M22 [get_ports {eth_rx_ctl}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_rx_ctl}]

# eth:0.rx_data
set_property LOC L20 [get_ports {eth_rx_data[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_rx_data[0]}]

# eth:0.rx_data
set_property LOC L21 [get_ports {eth_rx_data[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_rx_data[1]}]

# eth:0.rx_data
set_property LOC K21 [get_ports {eth_rx_data[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_rx_data[2]}]

# eth:0.rx_data
set_property LOC K22 [get_ports {eth_rx_data[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_rx_data[3]}]

# eth:0.tx_ctl
set_property LOC J22 [get_ports {eth_tx_ctl}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_tx_ctl}]

# eth:0.tx_data
set_property LOC G20 [get_ports {eth_tx_data[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_tx_data[0]}]

# eth:0.tx_data
set_property LOC H20 [get_ports {eth_tx_data[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_tx_data[1]}]

# eth:0.tx_data
set_property LOC H22 [get_ports {eth_tx_data[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_tx_data[2]}]

# eth:0.tx_data
set_property LOC J21 [get_ports {eth_tx_data[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_tx_data[3]}]

################################################################################
# eMMC
################################################################################

# Board has pullup resistors
# litesdcard only uses 4 data bits
set_property -dict { PACKAGE_PIN P17 IOSTANDARD LVCMOS33 SLEW FAST } [get_ports { sdcard_data[0] }];
set_property -dict { PACKAGE_PIN W17 IOSTANDARD LVCMOS33 SLEW FAST } [get_ports { sdcard_data[1] }];
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 SLEW FAST } [get_ports { sdcard_data[2] }];
set_property -dict { PACKAGE_PIN V18 IOSTANDARD LVCMOS33 SLEW FAST } [get_ports { sdcard_data[3] }];
set_property -dict { PACKAGE_PIN U18 IOSTANDARD LVCMOS33 SLEW FAST } [get_ports { sdcard_data[4] }];
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33 SLEW FAST } [get_ports { sdcard_data[5] }];
set_property -dict { PACKAGE_PIN T18 IOSTANDARD LVCMOS33 SLEW FAST } [get_ports { sdcard_data[6] }];
set_property -dict { PACKAGE_PIN R17 IOSTANDARD LVCMOS33 SLEW FAST } [get_ports { sdcard_data[7] }];

set_property -dict { PACKAGE_PIN Y19 IOSTANDARD LVCMOS33 SLEW FAST } [get_ports { sdcard_cmd }];
set_property -dict { PACKAGE_PIN Y18 IOSTANDARD LVCMOS33 SLEW FAST } [get_ports { sdcard_clk }];
set_property -dict { PACKAGE_PIN V19 IOSTANDARD LVCMOS33 SLEW FAST } [get_ports { sdcard_rstn }];


################################################################################
# Design constraints and bitsteam attributes
################################################################################

#Internal VREF
set_property INTERNAL_VREF 0.675 [get_iobanks 34]
set_property INTERNAL_VREF 0.675 [get_iobanks 35]

set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]

#set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets main_ethphy_eth_rx_clk_ibuf]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets {has_liteeth.liteeth/main_maccore_ethphy_eth_rx_clk_ibuf}]

################################################################################
# Clock constraints
################################################################################

create_clock -name sys_clk_pin -period 10.00 [get_ports { ext_clk }];

create_clock -name eth_clocks_rx -period 8.0 [get_ports { eth_clocks_rx }]

create_clock -name eth_clocks_tx -period 8.0 [get_ports { eth_clocks_tx }]


set_clock_groups -asynchronous -group [get_clocks sys_clk_pin -include_generated_clocks] -group [get_clocks eth_clocks_rx -include_generated_clocks]

set_clock_groups -asynchronous -group [get_clocks sys_clk_pin -include_generated_clocks] -group [get_clocks eth_clocks_tx -include_generated_clocks]



################################################################################
# False path constraints (from LiteX as they relate to LiteDRAM and LiteEth)
################################################################################


set_false_path -quiet -through [get_nets -hierarchical -filter {mr_ff == TRUE}]

set_false_path -quiet -to [get_pins -filter {REF_PIN_NAME == PRE} -of_objects [get_cells -hierarchical -filter {ars_ff1 == TRUE || ars_ff2 == TRUE}]]

set_max_delay 2 -quiet -from [get_pins -filter {REF_PIN_NAME == C} -of_objects [get_cells -hierarchical -filter {ars_ff1 == TRUE}]] -to [get_pins -filter {REF_PIN_NAME == D} -of_objects [get_cells -hierarchical -filter {ars_ff2 == TRUE}]]
