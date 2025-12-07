set_property -dict { PACKAGE_PIN E3    IOSTANDARD LVCMOS33 } [get_ports {clk}]; # CLK100MHZ
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports {clk}];

set_property -dict { PACKAGE_PIN M17   IOSTANDARD LVCMOS33 } [get_ports {rst}]; 

set_property -dict { PACKAGE_PIN C4    IOSTANDARD LVCMOS33 } [get_ports {uart_rx}]; # UART_TXD_IN (to FPGA)
set_property -dict { PACKAGE_PIN D4    IOSTANDARD LVCMOS33 } [get_ports {uart_tx}]; # UART_RXD_OUT (from FPGA)

set_property -dict { PACKAGE_PIN N15   IOSTANDARD LVCMOS33 } [get_ports {PWM_red}];
set_property -dict { PACKAGE_PIN M16   IOSTANDARD LVCMOS33 } [get_ports {PWM_green}];
set_property -dict { PACKAGE_PIN R12   IOSTANDARD LVCMOS33 } [get_ports {PWM_blue}];

set_property -dict { PACKAGE_PIN H17   IOSTANDARD LVCMOS33 } [get_ports {done}];