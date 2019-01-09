`define INITIAL_PC 32'hbfc00000
`define EXCEPTION_PC 32'hbfc00380

// Instruction decoding

`define GET_RS(x)       x[25:21]
`define GET_RT(x)       x[20:16]
`define GET_RD(x)       x[15:11]
`define GET_SA(x)       x[10:6]
`define GET_IMM(x)      x[15:0]
`define GET_INDEX(x)    x[25:0]

// Control signal indexes

`define I_ALU_ADD   0
`define I_ALU_SUB   1
`define I_ALU_AND   2
`define I_ALU_OR    3
`define I_ALU_XOR   4
`define I_ALU_NOR   5
`define I_ALU_SLT   6
`define I_ALU_SLTU  7
`define I_ALU_SLL   8
`define I_ALU_SRL   9
`define I_ALU_SRA   10

`define I_MFHI      11
`define I_MTHI      12
`define I_MFLO      13
`define I_MTLO      14
`define I_LUI       15
`define I_ERET      16
`define I_MFC0      17
`define I_MTC0      18
`define I_LB        19
`define I_LH        20
`define I_LWL       21
`define I_LW        22
`define I_LBU       23
`define I_LHU       24
`define I_LWR       25
`define I_SB        26
`define I_SH        27
`define I_SWL       28
`define I_SW        29
`define I_SWR       30

`define I_MEM_R     31
`define I_MEM_W     32
`define I_RS_R      33
`define I_RT_R      34
`define I_RT_WEX    35
`define I_RT_WWB    36
`define I_RD_W      37
`define I_R31_W     38
`define I_IMM_SX    39
`define I_ALU_A_SA  40
`define I_ALU_B_IMM 41
`define I_LINK      42
`define I_DO_MUL    43
`define I_DO_DIV    44
`define I_MD_SIGN   45
`define I_EXC_OF    46

`define I_MAX       47

// EXCCODE

`define EXC_INT     5'h00
`define EXC_ADEL    5'h04
`define EXC_ADES    5'h05
`define EXC_SYS     5'h08
`define EXC_BP      5'h09
`define EXC_RI      5'h0a
`define EXC_OV      5'h0c
`define EXC_ERET    5'h1f // only for indicating ERET operation, reserved in MIPS specification
