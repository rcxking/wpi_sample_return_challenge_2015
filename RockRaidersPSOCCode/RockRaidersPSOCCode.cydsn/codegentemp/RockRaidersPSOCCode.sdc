# THIS FILE IS AUTOMATICALLY GENERATED
# Project: C:\Users\malinaT430s\Documents\NASA\wpi_sample_return_challenge_2015\RockRaidersPSOCCode\RockRaidersPSOCCode.cydsn\RockRaidersPSOCCode.cyprj
# Date: Wed, 10 Jun 2015 17:59:14 GMT
#set_units -time ns
create_clock -name {PCComms_SCBCLK(FFB)} -period 708.33333333333326 -waveform {0 354.166666666667} [list [get_pins {ClockBlock/ff_div_2}]]
create_clock -name {MotorComms_SCBCLK(FFB)} -period 8666.6666666666661 -waveform {0 4333.33333333333} [list [get_pins {ClockBlock/ff_div_3}]]
create_clock -name {CyRouted1} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/dsi_in_0}]]
create_clock -name {CyILO} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/ilo}]]
create_clock -name {CyLFCLK} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/lfclk}]]
create_clock -name {CyIMO} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/imo}]]
create_clock -name {CyHFCLK} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/hfclk}]]
create_clock -name {CySYSCLK} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/sysclk}]]
create_generated_clock -name {PCComms_SCBCLK} -source [get_pins {ClockBlock/hfclk}] -edges {1 17 35} -nominal_period 708.33333333333326 [list]
create_generated_clock -name {MotorComms_SCBCLK} -source [get_pins {ClockBlock/hfclk}] -edges {1 209 417} -nominal_period 8666.6666666666661 [list]


# Component constraints for C:\Users\malinaT430s\Documents\NASA\wpi_sample_return_challenge_2015\RockRaidersPSOCCode\RockRaidersPSOCCode.cydsn\TopDesign\TopDesign.cysch
# Project: C:\Users\malinaT430s\Documents\NASA\wpi_sample_return_challenge_2015\RockRaidersPSOCCode\RockRaidersPSOCCode.cydsn\RockRaidersPSOCCode.cyprj
# Date: Wed, 10 Jun 2015 17:59:12 GMT
