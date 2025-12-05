set_property -dict { PACKAGE_PIN E3    IOSTANDARD LVCMOS33 } [get_ports {clk}]; # CLK100MHZ
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports {clk}];

set_property -dict { PACKAGE_PIN M17   IOSTANDARD LVCMOS33 } [get_ports {rst}]; 

## USB-UART (on-board FTDI/USB-UART interface)
## Note: the template names in the board files call these UART_TXD_IN / UART_RXD_OUT
## For our top-level: `uart_rx` receives data into the FPGA (connect to UART_TXD_IN)
## and `uart_tx` transmits data from FPGA (connect to UART_RXD_OUT).
set_property -dict { PACKAGE_PIN C4    IOSTANDARD LVCMOS33 } [get_ports {uart_rx}]; # UART_TXD_IN (to FPGA)
set_property -dict { PACKAGE_PIN D4    IOSTANDARD LVCMOS33 } [get_ports {uart_tx}]; # UART_RXD_OUT (from FPGA)

## RGB LED (user-controllable LEDs near the center of the board)
## LED16: R= N15, G= M16, B= R12 (verify on your board schematic if unsure)
set_property -dict { PACKAGE_PIN N15   IOSTANDARD LVCMOS33 } [get_ports {PWM_red}];
set_property -dict { PACKAGE_PIN M16   IOSTANDARD LVCMOS33 } [get_ports {PWM_green}];
set_property -dict { PACKAGE_PIN R12   IOSTANDARD LVCMOS33 } [get_ports {PWM_blue}];

## Status LED (use LED0)
set_property -dict { PACKAGE_PIN H17   IOSTANDARD LVCMOS33 } [get_ports {done}];
set_property -dict { PACKAGE_PIN K15   IOSTANDARD LVCMOS33 } [get_ports {test_light}];
set_property -dict { PACKAGE_PIN J13   IOSTANDARD LVCMOS33 } [get_ports {test_light1}];
set_property -dict { PACKAGE_PIN N14   IOSTANDARD LVCMOS33 } [get_ports {test_light2}];
set_property -dict { PACKAGE_PIN R18   IOSTANDARD LVCMOS33 } [get_ports {test_light3}];
set_property -dict { PACKAGE_PIN V17   IOSTANDARD LVCMOS33 } [get_ports {test_light4}]; # rx_done
set_property -dict { PACKAGE_PIN V11   IOSTANDARD LVCMOS33 } [get_ports {test_uart_rx}]; # uart_rx
set_property -dict { PACKAGE_PIN U17   IOSTANDARD LVCMOS33 } [get_ports {test_reciever}]; # reciever state == IDLE
set_property -dict { PACKAGE_PIN U16   IOSTANDARD LVCMOS33 } [get_ports {test_reciever1}]; # reciever state == START
set_property -dict { PACKAGE_PIN V16   IOSTANDARD LVCMOS33 } [get_ports {test_reciever2}]; # reciever state == DATA
set_property -dict { PACKAGE_PIN T15   IOSTANDARD LVCMOS33 } [get_ports {test_reciever3}]; # reciever state == STOP