// ======================================================================
// QuadDecoder2.v generated from TopDesign.cysch
// 06/01/2015 at 18:15
// This file is auto generated. ANY EDITS YOU MAKE MAY BE LOST WHEN THIS FILE IS REGENERATED!!!
// ======================================================================

/* -- WARNING: The following section of defines are deprecated and will be removed in a future release -- */
`define CYDEV_CHIP_DIE_LEOPARD 1
`define CYDEV_CHIP_REV_LEOPARD_PRODUCTION 3
`define CYDEV_CHIP_REV_LEOPARD_ES3 3
`define CYDEV_CHIP_REV_LEOPARD_ES2 1
`define CYDEV_CHIP_REV_LEOPARD_ES1 0
`define CYDEV_CHIP_DIE_PSOC4A 2
`define CYDEV_CHIP_REV_PSOC4A_PRODUCTION 17
`define CYDEV_CHIP_REV_PSOC4A_ES0 17
`define CYDEV_CHIP_DIE_PSOC5LP 3
`define CYDEV_CHIP_REV_PSOC5LP_PRODUCTION 0
`define CYDEV_CHIP_REV_PSOC5LP_ES0 0
`define CYDEV_CHIP_DIE_PANTHER 4
`define CYDEV_CHIP_REV_PANTHER_PRODUCTION 1
`define CYDEV_CHIP_REV_PANTHER_ES1 1
`define CYDEV_CHIP_REV_PANTHER_ES0 0
`define CYDEV_CHIP_DIE_EXPECT 2
`define CYDEV_CHIP_REV_EXPECT 17
`define CYDEV_CHIP_DIE_ACTUAL 2
/* -- WARNING: The previous section of defines are deprecated and will be removed in a future release -- */
`define CYDEV_CHIP_FAMILY_UNKNOWN 0
`define CYDEV_CHIP_MEMBER_UNKNOWN 0
`define CYDEV_CHIP_FAMILY_PSOC3 1
`define CYDEV_CHIP_MEMBER_3A 1
`define CYDEV_CHIP_REVISION_3A_PRODUCTION 3
`define CYDEV_CHIP_REVISION_3A_ES3 3
`define CYDEV_CHIP_REVISION_3A_ES2 1
`define CYDEV_CHIP_REVISION_3A_ES1 0
`define CYDEV_CHIP_FAMILY_PSOC4 2
`define CYDEV_CHIP_MEMBER_4D 2
`define CYDEV_CHIP_REVISION_4D_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4A 3
`define CYDEV_CHIP_REVISION_4A_PRODUCTION 17
`define CYDEV_CHIP_REVISION_4A_ES0 17
`define CYDEV_CHIP_MEMBER_4F 4
`define CYDEV_CHIP_REVISION_4F_PRODUCTION 0
`define CYDEV_CHIP_FAMILY_PSOC5 3
`define CYDEV_CHIP_MEMBER_5B 5
`define CYDEV_CHIP_REVISION_5B_PRODUCTION 0
`define CYDEV_CHIP_REVISION_5B_ES0 0
`define CYDEV_CHIP_MEMBER_5A 6
`define CYDEV_CHIP_REVISION_5A_PRODUCTION 1
`define CYDEV_CHIP_REVISION_5A_ES1 1
`define CYDEV_CHIP_REVISION_5A_ES0 0
`define CYDEV_CHIP_FAMILY_USED 2
`define CYDEV_CHIP_MEMBER_USED 3
`define CYDEV_CHIP_REVISION_USED 17
// Component: and_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\and_v1_0"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\and_v1_0\and_v1_0.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\and_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\and_v1_0\and_v1_0.v"
`endif

// Component: cydff_v1_30
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\cydff_v1_30"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\cydff_v1_30\cydff_v1_30.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\cydff_v1_30"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\cydff_v1_30\cydff_v1_30.v"
`endif

// Component: not_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\not_v1_0"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\not_v1_0\not_v1_0.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\not_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\not_v1_0\not_v1_0.v"
`endif

// Component: or_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\or_v1_0"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\or_v1_0\or_v1_0.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\or_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\or_v1_0\or_v1_0.v"
`endif

// Component: cy_constant_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\cy_constant_v1_0"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\cy_constant_v1_0\cy_constant_v1_0.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\cy_constant_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\cy_constant_v1_0\cy_constant_v1_0.v"
`endif

// Component: cy_virtualmux_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\cy_virtualmux_v1_0"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\cy_virtualmux_v1_0\cy_virtualmux_v1_0.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\cy_virtualmux_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\cy_virtualmux_v1_0\cy_virtualmux_v1_0.v"
`endif

// Component: ZeroTerminal
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal\ZeroTerminal.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal\ZeroTerminal.v"
`endif

// Component: B_Counter_v2_40
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cycomponentlibrary\CyComponentLibrary.cylib\B_Counter_v2_40"
`include "$CYPRESS_DIR\..\psoc\content\cycomponentlibrary\CyComponentLibrary.cylib\B_Counter_v2_40\B_Counter_v2_40.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cycomponentlibrary\CyComponentLibrary.cylib\B_Counter_v2_40"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\3.1\PSoC Creator\psoc\content\cycomponentlibrary\CyComponentLibrary.cylib\B_Counter_v2_40\B_Counter_v2_40.v"
`endif

// Counter_v2_40(CaptureMode=0, CaptureModeSoftware=0, ClockMode=3, CompareMode=1, CompareModeSoftware=0, CompareStatusEdgeSense=true, CompareValue=128, CONTROL3=0, ControlRegRemoved=0, CtlModeReplacementString=SyncCtl, CyGetRegReplacementString=CY_GET_REG8, CySetRegReplacementString=CY_SET_REG8, EnableMode=0, FF16=false, FF8=false, FixedFunction=false, FixedFunctionUsed=0, InitCounterValue=254, InterruptOnCapture=false, InterruptOnCompare=false, InterruptOnOverUnderFlow=false, InterruptOnTC=false, Period=254, RegDefReplacementString=reg8, RegSizeReplacementString=uint8, ReloadOnCapture=false, ReloadOnCompare=false, ReloadOnOverUnder=true, ReloadOnReset=false, Resolution=8, RstStatusReplacementString=sSTSReg_rstSts, RunMode=0, UDB16=false, UDB24=false, UDB32=false, UDB8=true, UDBControlReg=true, UseInterrupt=true, VerilogSectionReplacementString=sC8, CY_COMPONENT_NAME=Counter_v2_40, CY_CONTROL_FILE=<:default:>, CY_DATASHEET_FILE=<:default:>, CY_FITTER_NAME=Counter_2, CY_INSTANCE_SHORT_NAME=Counter_2, CY_MAJOR_VERSION=2, CY_MINOR_VERSION=40, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=PSoC Creator  3.1 SP1, INSTANCE_NAME=Counter_2, )
module Counter_v2_40_0 (
    clock,
    comp,
    tc,
    reset,
    interrupt,
    enable,
    capture,
    upCnt,
    downCnt,
    up_ndown,
    count);
    input       clock;
    output      comp;
    output      tc;
    input       reset;
    output      interrupt;
    input       enable;
    input       capture;
    input       upCnt;
    input       downCnt;
    input       up_ndown;
    input       count;

    parameter CaptureMode = 0;
    parameter ClockMode = 3;
    parameter CompareMode = 1;
    parameter CompareStatusEdgeSense = 1;
    parameter EnableMode = 0;
    parameter ReloadOnCapture = 0;
    parameter ReloadOnCompare = 0;
    parameter ReloadOnOverUnder = 1;
    parameter ReloadOnReset = 0;
    parameter Resolution = 8;
    parameter RunMode = 0;
    parameter UseInterrupt = 1;

          wire  Net_95;
          wire  Net_89;

	// VirtualMux_1 (cy_virtualmux_v1_0)
	assign Net_89 = Net_95;

    ZeroTerminal ZeroTerminal_2 (
        .z(Net_95));

    B_Counter_v2_40 CounterUDB (
        .reset(reset),
        .tc_out(tc),
        .cmp_out(comp),
        .clock(clock),
        .irq_out(interrupt),
        .up_ndown(Net_89),
        .upcnt(upCnt),
        .dwncnt(downCnt),
        .enable(enable),
        .capture(capture),
        .count(count));
    defparam CounterUDB.CaptureMode = 0;
    defparam CounterUDB.ClockMode = 3;
    defparam CounterUDB.CompareMode = 1;
    defparam CounterUDB.CompareStatusEdgeSense = 1;
    defparam CounterUDB.EnableMode = 0;
    defparam CounterUDB.ReloadOnCapture = 0;
    defparam CounterUDB.ReloadOnCompare = 0;
    defparam CounterUDB.ReloadOnOverUnder = 1;
    defparam CounterUDB.ReloadOnReset = 0;
    defparam CounterUDB.Resolution = 8;
    defparam CounterUDB.RunMode = 0;
    defparam CounterUDB.UseInterrupt = 1;



endmodule

// Counter_v2_40(CaptureMode=0, CaptureModeSoftware=0, ClockMode=3, CompareMode=1, CompareModeSoftware=0, CompareStatusEdgeSense=true, CompareValue=128, CONTROL3=0, ControlRegRemoved=0, CtlModeReplacementString=SyncCtl, CyGetRegReplacementString=CY_GET_REG8, CySetRegReplacementString=CY_SET_REG8, EnableMode=0, FF16=false, FF8=false, FixedFunction=false, FixedFunctionUsed=0, InitCounterValue=254, InterruptOnCapture=false, InterruptOnCompare=false, InterruptOnOverUnderFlow=false, InterruptOnTC=false, Period=254, RegDefReplacementString=reg8, RegSizeReplacementString=uint8, ReloadOnCapture=false, ReloadOnCompare=false, ReloadOnOverUnder=true, ReloadOnReset=false, Resolution=8, RstStatusReplacementString=sSTSReg_rstSts, RunMode=0, UDB16=false, UDB24=false, UDB32=false, UDB8=true, UDBControlReg=true, UseInterrupt=true, VerilogSectionReplacementString=sC8, CY_COMPONENT_NAME=Counter_v2_40, CY_CONTROL_FILE=<:default:>, CY_DATASHEET_FILE=<:default:>, CY_FITTER_NAME=Counter_3, CY_INSTANCE_SHORT_NAME=Counter_3, CY_MAJOR_VERSION=2, CY_MINOR_VERSION=40, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=PSoC Creator  3.1 SP1, INSTANCE_NAME=Counter_3, )
module Counter_v2_40_1 (
    clock,
    comp,
    tc,
    reset,
    interrupt,
    enable,
    capture,
    upCnt,
    downCnt,
    up_ndown,
    count);
    input       clock;
    output      comp;
    output      tc;
    input       reset;
    output      interrupt;
    input       enable;
    input       capture;
    input       upCnt;
    input       downCnt;
    input       up_ndown;
    input       count;

    parameter CaptureMode = 0;
    parameter ClockMode = 3;
    parameter CompareMode = 1;
    parameter CompareStatusEdgeSense = 1;
    parameter EnableMode = 0;
    parameter ReloadOnCapture = 0;
    parameter ReloadOnCompare = 0;
    parameter ReloadOnOverUnder = 1;
    parameter ReloadOnReset = 0;
    parameter Resolution = 8;
    parameter RunMode = 0;
    parameter UseInterrupt = 1;

          wire  Net_95;
          wire  Net_89;

	// VirtualMux_1 (cy_virtualmux_v1_0)
	assign Net_89 = Net_95;

    ZeroTerminal ZeroTerminal_2 (
        .z(Net_95));

    B_Counter_v2_40 CounterUDB (
        .reset(reset),
        .tc_out(tc),
        .cmp_out(comp),
        .clock(clock),
        .irq_out(interrupt),
        .up_ndown(Net_89),
        .upcnt(upCnt),
        .dwncnt(downCnt),
        .enable(enable),
        .capture(capture),
        .count(count));
    defparam CounterUDB.CaptureMode = 0;
    defparam CounterUDB.ClockMode = 3;
    defparam CounterUDB.CompareMode = 1;
    defparam CounterUDB.CompareStatusEdgeSense = 1;
    defparam CounterUDB.EnableMode = 0;
    defparam CounterUDB.ReloadOnCapture = 0;
    defparam CounterUDB.ReloadOnCompare = 0;
    defparam CounterUDB.ReloadOnOverUnder = 1;
    defparam CounterUDB.ReloadOnReset = 0;
    defparam CounterUDB.Resolution = 8;
    defparam CounterUDB.RunMode = 0;
    defparam CounterUDB.UseInterrupt = 1;



endmodule

// Counter_v2_40(CaptureMode=0, CaptureModeSoftware=0, ClockMode=3, CompareMode=1, CompareModeSoftware=0, CompareStatusEdgeSense=true, CompareValue=128, CONTROL3=0, ControlRegRemoved=0, CtlModeReplacementString=SyncCtl, CyGetRegReplacementString=CY_GET_REG8, CySetRegReplacementString=CY_SET_REG8, EnableMode=0, FF16=false, FF8=false, FixedFunction=false, FixedFunctionUsed=0, InitCounterValue=254, InterruptOnCapture=false, InterruptOnCompare=false, InterruptOnOverUnderFlow=false, InterruptOnTC=false, Period=254, RegDefReplacementString=reg8, RegSizeReplacementString=uint8, ReloadOnCapture=false, ReloadOnCompare=false, ReloadOnOverUnder=true, ReloadOnReset=false, Resolution=8, RstStatusReplacementString=sSTSReg_rstSts, RunMode=0, UDB16=false, UDB24=false, UDB32=false, UDB8=true, UDBControlReg=true, UseInterrupt=true, VerilogSectionReplacementString=sC8, CY_COMPONENT_NAME=Counter_v2_40, CY_CONTROL_FILE=<:default:>, CY_DATASHEET_FILE=<:default:>, CY_FITTER_NAME=Counter_4, CY_INSTANCE_SHORT_NAME=Counter_4, CY_MAJOR_VERSION=2, CY_MINOR_VERSION=40, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=PSoC Creator  3.1 SP1, INSTANCE_NAME=Counter_4, )
module Counter_v2_40_2 (
    clock,
    comp,
    tc,
    reset,
    interrupt,
    enable,
    capture,
    upCnt,
    downCnt,
    up_ndown,
    count);
    input       clock;
    output      comp;
    output      tc;
    input       reset;
    output      interrupt;
    input       enable;
    input       capture;
    input       upCnt;
    input       downCnt;
    input       up_ndown;
    input       count;

    parameter CaptureMode = 0;
    parameter ClockMode = 3;
    parameter CompareMode = 1;
    parameter CompareStatusEdgeSense = 1;
    parameter EnableMode = 0;
    parameter ReloadOnCapture = 0;
    parameter ReloadOnCompare = 0;
    parameter ReloadOnOverUnder = 1;
    parameter ReloadOnReset = 0;
    parameter Resolution = 8;
    parameter RunMode = 0;
    parameter UseInterrupt = 1;

          wire  Net_95;
          wire  Net_89;

	// VirtualMux_1 (cy_virtualmux_v1_0)
	assign Net_89 = Net_95;

    ZeroTerminal ZeroTerminal_2 (
        .z(Net_95));

    B_Counter_v2_40 CounterUDB (
        .reset(reset),
        .tc_out(tc),
        .cmp_out(comp),
        .clock(clock),
        .irq_out(interrupt),
        .up_ndown(Net_89),
        .upcnt(upCnt),
        .dwncnt(downCnt),
        .enable(enable),
        .capture(capture),
        .count(count));
    defparam CounterUDB.CaptureMode = 0;
    defparam CounterUDB.ClockMode = 3;
    defparam CounterUDB.CompareMode = 1;
    defparam CounterUDB.CompareStatusEdgeSense = 1;
    defparam CounterUDB.EnableMode = 0;
    defparam CounterUDB.ReloadOnCapture = 0;
    defparam CounterUDB.ReloadOnCompare = 0;
    defparam CounterUDB.ReloadOnOverUnder = 1;
    defparam CounterUDB.ReloadOnReset = 0;
    defparam CounterUDB.Resolution = 8;
    defparam CounterUDB.RunMode = 0;
    defparam CounterUDB.UseInterrupt = 1;



endmodule

// Counter_v2_40(CaptureMode=0, CaptureModeSoftware=0, ClockMode=3, CompareMode=1, CompareModeSoftware=0, CompareStatusEdgeSense=true, CompareValue=128, CONTROL3=0, ControlRegRemoved=0, CtlModeReplacementString=SyncCtl, CyGetRegReplacementString=CY_GET_REG8, CySetRegReplacementString=CY_SET_REG8, EnableMode=0, FF16=false, FF8=false, FixedFunction=false, FixedFunctionUsed=0, InitCounterValue=254, InterruptOnCapture=false, InterruptOnCompare=false, InterruptOnOverUnderFlow=false, InterruptOnTC=false, Period=254, RegDefReplacementString=reg8, RegSizeReplacementString=uint8, ReloadOnCapture=false, ReloadOnCompare=false, ReloadOnOverUnder=true, ReloadOnReset=false, Resolution=8, RstStatusReplacementString=sSTSReg_rstSts, RunMode=0, UDB16=false, UDB24=false, UDB32=false, UDB8=true, UDBControlReg=true, UseInterrupt=true, VerilogSectionReplacementString=sC8, CY_COMPONENT_NAME=Counter_v2_40, CY_CONTROL_FILE=<:default:>, CY_DATASHEET_FILE=<:default:>, CY_FITTER_NAME=Counter_1, CY_INSTANCE_SHORT_NAME=Counter_1, CY_MAJOR_VERSION=2, CY_MINOR_VERSION=40, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=PSoC Creator  3.1 SP1, INSTANCE_NAME=Counter_1, )
module Counter_v2_40_3 (
    clock,
    comp,
    tc,
    reset,
    interrupt,
    enable,
    capture,
    upCnt,
    downCnt,
    up_ndown,
    count);
    input       clock;
    output      comp;
    output      tc;
    input       reset;
    output      interrupt;
    input       enable;
    input       capture;
    input       upCnt;
    input       downCnt;
    input       up_ndown;
    input       count;

    parameter CaptureMode = 0;
    parameter ClockMode = 3;
    parameter CompareMode = 1;
    parameter CompareStatusEdgeSense = 1;
    parameter EnableMode = 0;
    parameter ReloadOnCapture = 0;
    parameter ReloadOnCompare = 0;
    parameter ReloadOnOverUnder = 1;
    parameter ReloadOnReset = 0;
    parameter Resolution = 8;
    parameter RunMode = 0;
    parameter UseInterrupt = 1;

          wire  Net_95;
          wire  Net_89;

	// VirtualMux_1 (cy_virtualmux_v1_0)
	assign Net_89 = Net_95;

    ZeroTerminal ZeroTerminal_2 (
        .z(Net_95));

    B_Counter_v2_40 CounterUDB (
        .reset(reset),
        .tc_out(tc),
        .cmp_out(comp),
        .clock(clock),
        .irq_out(interrupt),
        .up_ndown(Net_89),
        .upcnt(upCnt),
        .dwncnt(downCnt),
        .enable(enable),
        .capture(capture),
        .count(count));
    defparam CounterUDB.CaptureMode = 0;
    defparam CounterUDB.ClockMode = 3;
    defparam CounterUDB.CompareMode = 1;
    defparam CounterUDB.CompareStatusEdgeSense = 1;
    defparam CounterUDB.EnableMode = 0;
    defparam CounterUDB.ReloadOnCapture = 0;
    defparam CounterUDB.ReloadOnCompare = 0;
    defparam CounterUDB.ReloadOnOverUnder = 1;
    defparam CounterUDB.ReloadOnReset = 0;
    defparam CounterUDB.Resolution = 8;
    defparam CounterUDB.RunMode = 0;
    defparam CounterUDB.UseInterrupt = 1;



endmodule

// top
module top ;

          wire  Net_321;
          wire  Net_322;
          wire  Net_319;
          wire  Net_318;
          wire  Net_317;
          wire  Net_316;
          wire  Net_315;
          wire  Net_313;
          wire  Net_312;
          wire  Net_311;
          wire  Net_397;
          wire  Net_401;
          wire  Net_426;
          wire  Net_425;
          wire  Net_424;
          wire  Net_423;
          wire  Net_422;
          wire  Net_421;
          wire  Net_420;
          wire  Net_419;
          wire  Net_418;
          wire  Net_417;
          wire  Net_363;
          wire  Net_367;
          wire  Net_391;
          wire  CLK;
          wire  Net_390;
          wire  Net_389;
          wire  Net_388;
          wire  Net_387;
          wire  Net_386;
          wire  Net_385;
          wire  Net_384;
          wire  Net_383;
          wire  Net_328;
          wire  Net_332;
          wire  Net_357;
          wire  Net_356;
          wire  Net_355;
          wire  Net_354;
          wire  Net_353;
          wire  Net_352;
          wire  Net_351;
          wire  Net_350;
          wire  Net_349;
          wire  Net_348;
          wire  Net_392;
          wire  Net_267;
          wire  Net_225;
          wire  Net_221;
          wire  Net_410;
          wire  Net_411;
          wire  Net_415;
          wire  Net_412;
          wire  Net_413;
          wire  Net_416;
          wire  Net_414;
          wire  Net_406;
          wire  Net_407;
          wire  Net_408;
          wire  Net_409;
          wire  Net_404;
          wire  Net_400;
          wire  Net_402;
          wire  Net_405;
          wire  Net_398;
          wire  Net_403;
          wire  Net_396;
          wire  Net_399;
          wire  Net_394;
          wire  Net_395;
          wire  Net_393;
          wire  Net_376;
          wire  Net_377;
          wire  Net_381;
          wire  Net_378;
          wire  Net_379;
          wire  Net_382;
          wire  Net_380;
          wire  Net_372;
          wire  Net_373;
          wire  Net_374;
          wire  Net_375;
          wire  Net_370;
          wire  Net_366;
          wire  Net_368;
          wire  Net_371;
          wire  Net_364;
          wire  Net_369;
          wire  Net_362;
          wire  Net_365;
          wire  Net_360;
          wire  Net_361;
          wire  Net_359;
          wire  Net_341;
          wire  Net_342;
          wire  Net_346;
          wire  Net_343;
          wire  Net_344;
          wire  Net_347;
          wire  Net_345;
          wire  Net_337;
          wire  Net_338;
          wire  Net_339;
          wire  Net_340;
          wire  Net_335;
          wire  Net_331;
          wire  Net_333;
          wire  Net_336;
          wire  Net_329;
          wire  Net_334;
          wire  Net_327;
          wire  Net_330;
          wire  Net_325;
          wire  Net_326;
          wire  Net_324;
          wire  Net_112;
          wire  Net_169;
          wire  Net_121;
          wire  Net_186;
          wire  Net_182;
          wire  Net_184;
          wire  Net_115;
          wire  Net_180;
          wire  Net_185;
          wire  Net_142;
          wire  Net_183;
          wire  Net_200;
          wire  Net_201;
          wire  Net_202;
          wire  Net_203;
          wire  Net_195;
          wire  Net_196;
          wire  Net_207;
          wire  Net_222;
          wire  Net_205;
          wire  Net_224;
          wire  Net_223;


    assign Net_142 = Net_121 & Net_169;


    assign Net_184 = Net_182 & Net_221;


    assign Net_180 = Net_115 & Net_112;


    assign Net_186 = Net_183 & Net_225;

    // -- DFF Start --
    reg  cydff_4;
    always @(posedge CLK)
    begin
        cydff_4 <= Net_182;
    end
    assign Net_169 = cydff_4;
    // -- DFF End --

    // -- DFF Start --
    reg  cydff_3;
    always @(posedge CLK)
    begin
        cydff_3 <= Net_183;
    end
    assign Net_112 = cydff_3;
    // -- DFF End --

	wire [0:0] tmpOE__phiB_In_net;
	wire [0:0] tmpIO_0__phiB_In_net;
	wire [0:0] tmpINTERRUPT_0__phiB_In_net;
	electrical [0:0] tmpSIOVREF__phiB_In_net;

	cy_psoc3_pins_v1_10
		#(.id("ddb046c7-5d70-4f8c-96af-15c8484ad7e9"),
		  .drive_mode(3'b001),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b0),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("I"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b00),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiB_In
		 (.oe(tmpOE__phiB_In_net),
		  .y({1'b0}),
		  .fb({Net_182}),
		  .io({tmpIO_0__phiB_In_net[0:0]}),
		  .siovref(tmpSIOVREF__phiB_In_net),
		  .interrupt({tmpINTERRUPT_0__phiB_In_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiB_In_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__phiA_In_net;
	wire [0:0] tmpIO_0__phiA_In_net;
	wire [0:0] tmpINTERRUPT_0__phiA_In_net;
	electrical [0:0] tmpSIOVREF__phiA_In_net;

	cy_psoc3_pins_v1_10
		#(.id("58320c45-4f42-4c98-ab2a-d3b27fe001ec"),
		  .drive_mode(3'b001),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b0),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("I"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b00),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiA_In
		 (.oe(tmpOE__phiA_In_net),
		  .y({1'b0}),
		  .fb({Net_183}),
		  .io({tmpIO_0__phiA_In_net[0:0]}),
		  .siovref(tmpSIOVREF__phiA_In_net),
		  .interrupt({tmpINTERRUPT_0__phiA_In_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiA_In_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__phiA_Out_net;
	wire [0:0] tmpFB_0__phiA_Out_net;
	wire [0:0] tmpIO_0__phiA_Out_net;
	wire [0:0] tmpINTERRUPT_0__phiA_Out_net;
	electrical [0:0] tmpSIOVREF__phiA_Out_net;

	cy_psoc3_pins_v1_10
		#(.id("02319258-735c-483f-8f44-13e3e634d238"),
		  .drive_mode(3'b110),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b1),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b10),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiA_Out
		 (.oe(tmpOE__phiA_Out_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__phiA_Out_net[0:0]}),
		  .io({tmpIO_0__phiA_Out_net[0:0]}),
		  .siovref(tmpSIOVREF__phiA_Out_net),
		  .interrupt({tmpINTERRUPT_0__phiA_Out_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiA_Out_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};


	cy_clock_v1_0
		#(.id("810eaf3a-9ffe-4682-8bae-1bb8e4611974"),
		  .source_clock_id(""),
		  .divisor(0),
		  .period("83333333.3333333"),
		  .is_direct(0),
		  .is_digital(0))
		Clock_1
		 (.clock_out(CLK));



    assign Net_225 = ~Net_112;


    assign Net_115 = ~Net_183;


    assign Net_185 = ~Net_182;


    assign Net_121 = ~Net_182;


    assign Net_221 = ~Net_169;


    assign Net_203 = Net_186 & Net_185;


    assign Net_202 = Net_184 & Net_183;


    assign Net_201 = Net_180 & Net_182;


    assign Net_200 = Net_142 & Net_115;


    assign Net_205 = Net_186 & Net_182;


    assign Net_207 = Net_184 & Net_115;


    assign Net_196 = Net_180 & Net_185;


    assign Net_195 = Net_142 & Net_183;


    assign Net_223 = Net_202 | Net_201 | Net_200 | Net_203;


    assign Net_222 = Net_207 | Net_196 | Net_195 | Net_205;


    assign Net_224 = Net_223 | Net_222;

	wire [0:0] tmpOE__phiA_In_1_net;
	wire [0:0] tmpIO_0__phiA_In_1_net;
	wire [0:0] tmpINTERRUPT_0__phiA_In_1_net;
	electrical [0:0] tmpSIOVREF__phiA_In_1_net;

	cy_psoc3_pins_v1_10
		#(.id("d8bdcee4-5506-4190-9ef6-29b494165e9e"),
		  .drive_mode(3'b001),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b0),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("I"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b00),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiA_In_1
		 (.oe(tmpOE__phiA_In_1_net),
		  .y({1'b0}),
		  .fb({Net_324}),
		  .io({tmpIO_0__phiA_In_1_net[0:0]}),
		  .siovref(tmpSIOVREF__phiA_In_1_net),
		  .interrupt({tmpINTERRUPT_0__phiA_In_1_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiA_In_1_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__phiA_Out_1_net;
	wire [0:0] tmpFB_0__phiA_Out_1_net;
	wire [0:0] tmpIO_0__phiA_Out_1_net;
	wire [0:0] tmpINTERRUPT_0__phiA_Out_1_net;
	electrical [0:0] tmpSIOVREF__phiA_Out_1_net;

	cy_psoc3_pins_v1_10
		#(.id("588a543d-0935-41f8-9d0c-62522c1969c8"),
		  .drive_mode(3'b110),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b1),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b10),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiA_Out_1
		 (.oe(tmpOE__phiA_Out_1_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__phiA_Out_1_net[0:0]}),
		  .io({tmpIO_0__phiA_Out_1_net[0:0]}),
		  .siovref(tmpSIOVREF__phiA_Out_1_net),
		  .interrupt({tmpINTERRUPT_0__phiA_Out_1_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiA_Out_1_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

    assign Net_392 = 1'h1;

    Counter_v2_40_0 Counter_2 (
        .reset(Net_348),
        .tc(Net_349),
        .comp(Net_350),
        .clock(Net_347),
        .interrupt(Net_351),
        .enable(1'b0),
        .capture(1'b0),
        .upCnt(1'b0),
        .downCnt(1'b0),
        .up_ndown(1'b1),
        .count(Net_357));
    defparam Counter_2.CaptureMode = 0;
    defparam Counter_2.ClockMode = 3;
    defparam Counter_2.CompareMode = 1;
    defparam Counter_2.CompareStatusEdgeSense = 1;
    defparam Counter_2.EnableMode = 0;
    defparam Counter_2.ReloadOnCapture = 0;
    defparam Counter_2.ReloadOnCompare = 0;
    defparam Counter_2.ReloadOnOverUnder = 1;
    defparam Counter_2.ReloadOnReset = 0;
    defparam Counter_2.Resolution = 8;
    defparam Counter_2.RunMode = 0;
    defparam Counter_2.UseInterrupt = 1;


    assign Net_347 = Net_345 | Net_346;


    assign Net_346 = Net_342 | Net_343 | Net_344 | Net_341;


    assign Net_345 = Net_338 | Net_339 | Net_340 | Net_337;


    assign Net_344 = Net_335 & Net_324;


    assign Net_343 = Net_331 & Net_336;


    assign Net_342 = Net_333 & Net_330;


    assign Net_341 = Net_329 & Net_325;


    assign Net_340 = Net_335 & Net_330;


    assign Net_339 = Net_331 & Net_325;


    assign Net_338 = Net_333 & Net_324;


    assign Net_337 = Net_329 & Net_336;


    assign Net_332 = ~Net_327;


    assign Net_334 = ~Net_325;


    assign Net_336 = ~Net_325;


    assign Net_330 = ~Net_324;


    assign Net_328 = ~Net_326;


    assign Net_335 = Net_334 & Net_327;


    assign Net_333 = Net_325 & Net_332;


    assign Net_331 = Net_330 & Net_326;


    assign Net_329 = Net_324 & Net_328;

    // -- DFF Start --
    reg  cydff_2;
    always @(posedge CLK)
    begin
        cydff_2 <= Net_325;
    end
    assign Net_327 = cydff_2;
    // -- DFF End --

    // -- DFF Start --
    reg  cydff_1;
    always @(posedge CLK)
    begin
        cydff_1 <= Net_324;
    end
    assign Net_326 = cydff_1;
    // -- DFF End --

	wire [0:0] tmpOE__phiB_In_1_net;
	wire [0:0] tmpIO_0__phiB_In_1_net;
	wire [0:0] tmpINTERRUPT_0__phiB_In_1_net;
	electrical [0:0] tmpSIOVREF__phiB_In_1_net;

	cy_psoc3_pins_v1_10
		#(.id("5c56d68e-cafa-4ba9-953b-1a904b5c360c"),
		  .drive_mode(3'b001),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b0),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("I"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b00),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiB_In_1
		 (.oe(tmpOE__phiB_In_1_net),
		  .y({1'b0}),
		  .fb({Net_325}),
		  .io({tmpIO_0__phiB_In_1_net[0:0]}),
		  .siovref(tmpSIOVREF__phiB_In_1_net),
		  .interrupt({tmpINTERRUPT_0__phiB_In_1_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiB_In_1_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

    assign Net_383 = 1'h0;

    Counter_v2_40_1 Counter_3 (
        .reset(Net_383),
        .tc(Net_384),
        .comp(Net_385),
        .clock(Net_382),
        .interrupt(Net_386),
        .enable(1'b0),
        .capture(1'b0),
        .upCnt(1'b0),
        .downCnt(1'b0),
        .up_ndown(1'b1),
        .count(Net_392));
    defparam Counter_3.CaptureMode = 0;
    defparam Counter_3.ClockMode = 3;
    defparam Counter_3.CompareMode = 1;
    defparam Counter_3.CompareStatusEdgeSense = 1;
    defparam Counter_3.EnableMode = 0;
    defparam Counter_3.ReloadOnCapture = 0;
    defparam Counter_3.ReloadOnCompare = 0;
    defparam Counter_3.ReloadOnOverUnder = 1;
    defparam Counter_3.ReloadOnReset = 0;
    defparam Counter_3.Resolution = 8;
    defparam Counter_3.RunMode = 0;
    defparam Counter_3.UseInterrupt = 1;


    assign Net_382 = Net_380 | Net_381;


    assign Net_381 = Net_377 | Net_378 | Net_379 | Net_376;


    assign Net_380 = Net_373 | Net_374 | Net_375 | Net_372;


    assign Net_379 = Net_370 & Net_359;


    assign Net_378 = Net_366 & Net_371;


    assign Net_377 = Net_368 & Net_365;


    assign Net_376 = Net_364 & Net_360;


    assign Net_375 = Net_370 & Net_365;


    assign Net_374 = Net_366 & Net_360;


    assign Net_373 = Net_368 & Net_359;


    assign Net_372 = Net_364 & Net_371;


    assign Net_367 = ~Net_362;


    assign Net_369 = ~Net_360;


    assign Net_371 = ~Net_360;


    assign Net_365 = ~Net_359;


    assign Net_363 = ~Net_361;


    assign Net_370 = Net_369 & Net_362;


    assign Net_368 = Net_360 & Net_367;


    assign Net_366 = Net_365 & Net_361;


    assign Net_364 = Net_359 & Net_363;

    // -- DFF Start --
    reg  cydff_6;
    always @(posedge CLK)
    begin
        cydff_6 <= Net_360;
    end
    assign Net_362 = cydff_6;
    // -- DFF End --

    // -- DFF Start --
    reg  cydff_5;
    always @(posedge CLK)
    begin
        cydff_5 <= Net_359;
    end
    assign Net_361 = cydff_5;
    // -- DFF End --

	wire [0:0] tmpOE__phiB_In_2_net;
	wire [0:0] tmpIO_0__phiB_In_2_net;
	wire [0:0] tmpINTERRUPT_0__phiB_In_2_net;
	electrical [0:0] tmpSIOVREF__phiB_In_2_net;

	cy_psoc3_pins_v1_10
		#(.id("fd3b3ec8-d371-4fe2-b772-f8f3aa56e669"),
		  .drive_mode(3'b001),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b0),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("I"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b00),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiB_In_2
		 (.oe(tmpOE__phiB_In_2_net),
		  .y({1'b0}),
		  .fb({Net_360}),
		  .io({tmpIO_0__phiB_In_2_net[0:0]}),
		  .siovref(tmpSIOVREF__phiB_In_2_net),
		  .interrupt({tmpINTERRUPT_0__phiB_In_2_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiB_In_2_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__phiA_In_2_net;
	wire [0:0] tmpIO_0__phiA_In_2_net;
	wire [0:0] tmpINTERRUPT_0__phiA_In_2_net;
	electrical [0:0] tmpSIOVREF__phiA_In_2_net;

	cy_psoc3_pins_v1_10
		#(.id("122adcdb-e3ab-4b31-b1ed-8736823839fc"),
		  .drive_mode(3'b001),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b0),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("I"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b00),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiA_In_2
		 (.oe(tmpOE__phiA_In_2_net),
		  .y({1'b0}),
		  .fb({Net_359}),
		  .io({tmpIO_0__phiA_In_2_net[0:0]}),
		  .siovref(tmpSIOVREF__phiA_In_2_net),
		  .interrupt({tmpINTERRUPT_0__phiA_In_2_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiA_In_2_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__phiA_Out_2_net;
	wire [0:0] tmpFB_0__phiA_Out_2_net;
	wire [0:0] tmpIO_0__phiA_Out_2_net;
	wire [0:0] tmpINTERRUPT_0__phiA_Out_2_net;
	electrical [0:0] tmpSIOVREF__phiA_Out_2_net;

	cy_psoc3_pins_v1_10
		#(.id("ee25189a-d909-405e-9598-a654ad181942"),
		  .drive_mode(3'b110),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b1),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b10),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiA_Out_2
		 (.oe(tmpOE__phiA_Out_2_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__phiA_Out_2_net[0:0]}),
		  .io({tmpIO_0__phiA_Out_2_net[0:0]}),
		  .siovref(tmpSIOVREF__phiA_Out_2_net),
		  .interrupt({tmpINTERRUPT_0__phiA_Out_2_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiA_Out_2_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

    assign Net_417 = 1'h0;

    Counter_v2_40_2 Counter_4 (
        .reset(Net_417),
        .tc(Net_418),
        .comp(Net_419),
        .clock(Net_416),
        .interrupt(Net_420),
        .enable(1'b0),
        .capture(1'b0),
        .upCnt(1'b0),
        .downCnt(1'b0),
        .up_ndown(1'b1),
        .count(Net_426));
    defparam Counter_4.CaptureMode = 0;
    defparam Counter_4.ClockMode = 3;
    defparam Counter_4.CompareMode = 1;
    defparam Counter_4.CompareStatusEdgeSense = 1;
    defparam Counter_4.EnableMode = 0;
    defparam Counter_4.ReloadOnCapture = 0;
    defparam Counter_4.ReloadOnCompare = 0;
    defparam Counter_4.ReloadOnOverUnder = 1;
    defparam Counter_4.ReloadOnReset = 0;
    defparam Counter_4.Resolution = 8;
    defparam Counter_4.RunMode = 0;
    defparam Counter_4.UseInterrupt = 1;


    assign Net_416 = Net_414 | Net_415;


    assign Net_415 = Net_411 | Net_412 | Net_413 | Net_410;


    assign Net_414 = Net_407 | Net_408 | Net_409 | Net_406;


    assign Net_413 = Net_404 & Net_393;


    assign Net_412 = Net_400 & Net_405;


    assign Net_411 = Net_402 & Net_399;


    assign Net_410 = Net_398 & Net_394;


    assign Net_409 = Net_404 & Net_399;


    assign Net_408 = Net_400 & Net_394;


    assign Net_407 = Net_402 & Net_393;


    assign Net_406 = Net_398 & Net_405;


    assign Net_401 = ~Net_396;


    assign Net_403 = ~Net_394;


    assign Net_405 = ~Net_394;


    assign Net_399 = ~Net_393;


    assign Net_397 = ~Net_395;


    assign Net_404 = Net_403 & Net_396;


    assign Net_402 = Net_394 & Net_401;


    assign Net_400 = Net_399 & Net_395;


    assign Net_398 = Net_393 & Net_397;

    // -- DFF Start --
    reg  cydff_8;
    always @(posedge CLK)
    begin
        cydff_8 <= Net_394;
    end
    assign Net_396 = cydff_8;
    // -- DFF End --

    // -- DFF Start --
    reg  cydff_7;
    always @(posedge CLK)
    begin
        cydff_7 <= Net_393;
    end
    assign Net_395 = cydff_7;
    // -- DFF End --

	wire [0:0] tmpOE__phiB_In_3_net;
	wire [0:0] tmpIO_0__phiB_In_3_net;
	wire [0:0] tmpINTERRUPT_0__phiB_In_3_net;
	electrical [0:0] tmpSIOVREF__phiB_In_3_net;

	cy_psoc3_pins_v1_10
		#(.id("56cf5928-5e12-49d7-b648-e04def27de43"),
		  .drive_mode(3'b001),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b0),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("I"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b00),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiB_In_3
		 (.oe(tmpOE__phiB_In_3_net),
		  .y({1'b0}),
		  .fb({Net_394}),
		  .io({tmpIO_0__phiB_In_3_net[0:0]}),
		  .siovref(tmpSIOVREF__phiB_In_3_net),
		  .interrupt({tmpINTERRUPT_0__phiB_In_3_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiB_In_3_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__phiA_In_3_net;
	wire [0:0] tmpIO_0__phiA_In_3_net;
	wire [0:0] tmpINTERRUPT_0__phiA_In_3_net;
	electrical [0:0] tmpSIOVREF__phiA_In_3_net;

	cy_psoc3_pins_v1_10
		#(.id("d27c8f4e-c3b8-4f2e-b816-eced056e25de"),
		  .drive_mode(3'b001),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b0),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("I"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b00),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiA_In_3
		 (.oe(tmpOE__phiA_In_3_net),
		  .y({1'b0}),
		  .fb({Net_393}),
		  .io({tmpIO_0__phiA_In_3_net[0:0]}),
		  .siovref(tmpSIOVREF__phiA_In_3_net),
		  .interrupt({tmpINTERRUPT_0__phiA_In_3_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiA_In_3_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__phiA_Out_3_net;
	wire [0:0] tmpFB_0__phiA_Out_3_net;
	wire [0:0] tmpIO_0__phiA_Out_3_net;
	wire [0:0] tmpINTERRUPT_0__phiA_Out_3_net;
	electrical [0:0] tmpSIOVREF__phiA_Out_3_net;

	cy_psoc3_pins_v1_10
		#(.id("5551810f-263d-4b73-b7e6-ffd18b91bfbf"),
		  .drive_mode(3'b110),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_clk_en(0),
		  .input_sync(1'b1),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b0),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b10),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		phiA_Out_3
		 (.oe(tmpOE__phiA_Out_3_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__phiA_Out_3_net[0:0]}),
		  .io({tmpIO_0__phiA_Out_3_net[0:0]}),
		  .siovref(tmpSIOVREF__phiA_Out_3_net),
		  .interrupt({tmpINTERRUPT_0__phiA_Out_3_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__phiA_Out_3_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

    Counter_v2_40_3 Counter_1 (
        .reset(Net_311),
        .tc(Net_312),
        .comp(Net_313),
        .clock(Net_224),
        .interrupt(Net_315),
        .enable(1'b0),
        .capture(1'b0),
        .upCnt(1'b0),
        .downCnt(1'b0),
        .up_ndown(1'b1),
        .count(Net_321));
    defparam Counter_1.CaptureMode = 0;
    defparam Counter_1.ClockMode = 3;
    defparam Counter_1.CompareMode = 1;
    defparam Counter_1.CompareStatusEdgeSense = 1;
    defparam Counter_1.EnableMode = 0;
    defparam Counter_1.ReloadOnCapture = 0;
    defparam Counter_1.ReloadOnCompare = 0;
    defparam Counter_1.ReloadOnOverUnder = 1;
    defparam Counter_1.ReloadOnReset = 0;
    defparam Counter_1.Resolution = 8;
    defparam Counter_1.RunMode = 0;
    defparam Counter_1.UseInterrupt = 1;

    assign Net_311 = 1'h0;

    assign Net_321 = 1'h1;

    assign Net_348 = 1'h0;

    assign Net_357 = 1'h1;

    assign Net_426 = 1'h1;



endmodule

