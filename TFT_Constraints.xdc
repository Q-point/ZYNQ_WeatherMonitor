

set_property IOSTANDARD LVCMOS33 [get_ports {spi_rtl_ss_i[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_rtl_ss_i[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports spi_rtl_io0_i]
set_property IOSTANDARD LVCMOS33 [get_ports spi_rtl_io0_t]
set_property IOSTANDARD LVCMOS33 [get_ports spi_rtl_io1_o]
set_property IOSTANDARD LVCMOS33 [get_ports spi_rtl_io1_t]
set_property IOSTANDARD LVCMOS33 [get_ports spi_rtl_sck_i]
set_property IOSTANDARD LVCMOS33 [get_ports spi_rtl_sck_t]
set_property IOSTANDARD LVCMOS33 [get_ports spi_rtl_ss_t]

set_property PACKAGE_PIN N7 [get_ports MISO]
set_property PACKAGE_PIN R10 [get_ports MOSI]
set_property PACKAGE_PIN P10 [get_ports SCK]

set_property PACKAGE_PIN G15 [get_ports iic_rtl_scl_io]
set_property PACKAGE_PIN F15 [get_ports iic_rtl_sda_io]
set_property DRIVE 12 [get_ports iic_rtl_scl_io]
set_property SLEW SLOW [get_ports iic_rtl_scl_io]

