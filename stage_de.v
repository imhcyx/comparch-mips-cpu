`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/09/21 09:47:11
// Design Name: 
// Module Name: stage_de
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`include "common.vh"

module decoder #(
    parameter integer bits = 4
)
(
    input [bits-1:0] in,
    output [(1<<bits)-1:0] out
);

  generate
    genvar i;
    for (i=0; i<(1<<bits); i=i+1) begin
      assign out[i] = in == i;
    end
  endgenerate

endmodule

module stage_de(

    // Common interfaces
    input               clk,
    input               resetn,
    input               valid_in,
    output              valid_out,
    input               stall_in,
    output              stall_out,
    input [31:0]        pc_in,

    // Exception interfaces
    input               exc_in,
    output              exc_out,
    output [4:0]        exccode_out,
    output              bd_out,

    // Special interfaces
    input [31:0]        instr_in,
    output [`I_MAX-1:0] ctrl_out,
    output [4:0]        rf_raddr1,
    output [4:0]        rf_raddr2,
    input [31:0]        rf_rdata1,
    input [31:0]        rf_rdata2,
    output [31:0]       rdata1_out,
    output [31:0]       rdata2_out,
    output              pc_update,
    output [31:0]       pc_new,
    input               ex_fwd,
    input               ex_valid,
    input [4:0]         ex_addr,
    input [31:0]        ex_data,
    input               mem_fwd,
    input               mem_valid,
    input [4:0]         mem_addr,
    input [31:0]        mem_data,
    input               wb_fwd,
    input               wb_valid,
    input [4:0]         wb_addr,
    input [31:0]        wb_data,
    input               ok_to_branch

);

  wire valid;

  wire [`I_MAX-1:0] ctrl_sig;

  wire [63:0] op_d, func_d;
  wire [31:0] rs_d, rt_d, rd_d, sa_d;

  decoder #(.bits(6))
  dec_op (.in(instr_in[31:26]), .out(op_d)), dec_func (.in(instr_in[5:0]), .out(func_d));

  decoder #(.bits(5))
  dec_rs (.in(instr_in[25:21]), .out(rs_d)), dec_rt (.in(instr_in[20:16]), .out(rt_d)),
  dec_rd (.in(instr_in[15:11]), .out(rd_d)), dec_sa (.in(instr_in[10:6]), .out(sa_d));

  wire op_sll       = op_d[0] && rs_d[0] && func_d[0];
  wire op_srl       = op_d[0] && rs_d[0] && func_d[2];
  wire op_sra       = op_d[0] && rs_d[0] && func_d[3];
  wire op_sllv      = op_d[0] && sa_d[0] && func_d[4];
  wire op_srlv      = op_d[0] && sa_d[0] && func_d[6];
  wire op_srav      = op_d[0] && sa_d[0] && func_d[7];
  wire op_jr        = op_d[0] && rt_d[0] && rd_d[0] && sa_d[0] && func_d[8];
  wire op_jalr      = op_d[0] && rt_d[0] && sa_d[0] && func_d[9];
  wire op_syscall   = op_d[0] && func_d[12];
  wire op_break     = op_d[0] && func_d[13];
  wire op_mfhi      = op_d[0] && rs_d[0] && rt_d[0] && sa_d[0] && func_d[16];
  wire op_mthi      = op_d[0] && rt_d[0] && rd_d[0] && sa_d[0] && func_d[17];
  wire op_mflo      = op_d[0] && rs_d[0] && rt_d[0] && sa_d[0] && func_d[18];
  wire op_mtlo      = op_d[0] && rt_d[0] && rd_d[0] && sa_d[0] && func_d[19];
  wire op_mult      = op_d[0] && rd_d[0] && sa_d[0] && func_d[24];
  wire op_multu     = op_d[0] && rd_d[0] && sa_d[0] && func_d[25];
  wire op_div       = op_d[0] && rd_d[0] && sa_d[0] && func_d[26];
  wire op_divu      = op_d[0] && rd_d[0] && sa_d[0] && func_d[27];
  wire op_add       = op_d[0] && sa_d[0] && func_d[32];
  wire op_addu      = op_d[0] && sa_d[0] && func_d[33];
  wire op_sub       = op_d[0] && sa_d[0] && func_d[34];
  wire op_subu      = op_d[0] && sa_d[0] && func_d[35];
  wire op_and       = op_d[0] && sa_d[0] && func_d[36];
  wire op_or        = op_d[0] && sa_d[0] && func_d[37];
  wire op_xor       = op_d[0] && sa_d[0] && func_d[38];
  wire op_nor       = op_d[0] && sa_d[0] && func_d[39];
  wire op_slt       = op_d[0] && sa_d[0] && func_d[42];
  wire op_sltu      = op_d[0] && sa_d[0] && func_d[43];
  wire op_bltz      = op_d[1] && rt_d[0];
  wire op_bgez      = op_d[1] && rt_d[1];
  wire op_bltzal    = op_d[1] && rt_d[16];
  wire op_bgezal    = op_d[1] && rt_d[17];
  wire op_j         = op_d[2];
  wire op_jal       = op_d[3];
  wire op_beq       = op_d[4];
  wire op_bne       = op_d[5];
  wire op_blez      = op_d[6] && rt_d[0];
  wire op_bgtz      = op_d[7] && rt_d[0];
  wire op_addi      = op_d[8];
  wire op_addiu     = op_d[9];
  wire op_slti      = op_d[10];
  wire op_sltiu     = op_d[11];
  wire op_andi      = op_d[12];
  wire op_ori       = op_d[13];
  wire op_xori      = op_d[14];
  wire op_lui       = op_d[15];
  wire op_eret      = op_d[16] && rs_d[16] && rt_d[0] && rd_d[0] && sa_d[0] && func_d[24];
  wire op_mfc0      = op_d[16] && rs_d[0] && sa_d[0] && instr_in[5:3] == 3'b000;
  wire op_mtc0      = op_d[16] && rs_d[4] && sa_d[0] && instr_in[5:3] == 3'b000;
  wire op_lb        = op_d[32];
  wire op_lh        = op_d[33];
  wire op_lwl       = op_d[34];
  wire op_lw        = op_d[35];
  wire op_lbu       = op_d[36];
  wire op_lhu       = op_d[37];
  wire op_lwr       = op_d[38];
  wire op_sb        = op_d[40];
  wire op_sh        = op_d[41];
  wire op_swl       = op_d[42];
  wire op_sw        = op_d[43];
  wire op_swr       = op_d[46];

  wire reserved     = !(op_sll||op_srl||op_sra||op_sllv||op_srlv||op_srav||op_jr||op_jalr||op_syscall||op_break||
                         op_mfhi||op_mthi||op_mflo||op_mtlo||op_mult||op_multu||op_div||op_divu||
                         op_add||op_addu||op_sub||op_subu||op_and||op_or||op_xor||op_nor||op_slt||op_sltu||
                         op_bltz||op_bgez||op_bltzal||op_bgezal||op_j||op_jal||op_beq||op_bne||op_blez||op_bgtz||
                         op_addi||op_addiu||op_slti||op_sltiu||op_andi||op_ori||op_xori||op_lui||
                         op_eret||op_mfc0||op_mtc0||
                         op_lb||op_lh||op_lwl||op_lw||op_lbu||op_lhu||op_lwr||op_sb||op_sh||op_swl||op_sw||op_swr);

  assign ctrl_sig[`I_MFHI]      = op_mfhi;
  assign ctrl_sig[`I_MTHI]      = op_mthi;
  assign ctrl_sig[`I_MFLO]      = op_mflo;
  assign ctrl_sig[`I_MTLO]      = op_mtlo;
  assign ctrl_sig[`I_LUI]       = op_lui;
  assign ctrl_sig[`I_ERET]      = op_eret;
  assign ctrl_sig[`I_MFC0]      = op_mfc0;
  assign ctrl_sig[`I_MTC0]      = op_mtc0;
  assign ctrl_sig[`I_LB]        = op_lb;
  assign ctrl_sig[`I_LH]        = op_lh;
  assign ctrl_sig[`I_LWL]       = op_lwl;
  assign ctrl_sig[`I_LW]        = op_lw;
  assign ctrl_sig[`I_LBU]       = op_lbu;
  assign ctrl_sig[`I_LHU]       = op_lhu;
  assign ctrl_sig[`I_LWR]       = op_lwr;
  assign ctrl_sig[`I_SB]        = op_sb;
  assign ctrl_sig[`I_SH]        = op_sh;
  assign ctrl_sig[`I_SWL]       = op_swl;
  assign ctrl_sig[`I_SW]        = op_sw;
  assign ctrl_sig[`I_SWR]       = op_swr;

  // load instruction
  assign ctrl_sig[`I_MEM_R]     = op_lb||op_lh||op_lwl||op_lw||op_lbu||op_lhu||op_lwr;
  // store instruction
  assign ctrl_sig[`I_MEM_W]     = op_sb||op_sh||op_swl||op_sw||op_swr;
  // alu operation
  assign ctrl_sig[`I_ALU_ADD]   = op_add||op_addu||op_addi||op_addiu||ctrl_sig[`I_MEM_R]||ctrl_sig[`I_MEM_W];
  assign ctrl_sig[`I_ALU_SUB]   = op_sub||op_subu||op_beq||op_bne;
  assign ctrl_sig[`I_ALU_AND]   = op_and||op_andi;
  assign ctrl_sig[`I_ALU_OR]    = op_or||op_ori;
  assign ctrl_sig[`I_ALU_XOR]   = op_xor||op_xori;
  assign ctrl_sig[`I_ALU_NOR]   = op_nor;
  assign ctrl_sig[`I_ALU_SLT]   = op_slt||op_slti;
  assign ctrl_sig[`I_ALU_SLTU]  = op_sltu||op_sltiu;
  assign ctrl_sig[`I_ALU_SLL]   = op_sll||op_sllv;
  assign ctrl_sig[`I_ALU_SRL]   = op_srl||op_srlv;
  assign ctrl_sig[`I_ALU_SRA]   = op_sra||op_srav;
  // read [rs]
  assign ctrl_sig[`I_RS_R]      = op_sllv||op_srlv||op_srav||op_jr||op_jalr||op_mthi||op_mtlo||op_mult||op_multu||op_div||op_divu||
                                  op_add||op_addu||op_sub||op_subu||op_and||op_or||op_xor||op_nor||op_slt||op_sltu||
                                  op_bltz||op_bgez||op_bltzal||op_bgezal||op_beq||op_bne||op_blez||op_bgtz||
                                  op_addi||op_addiu||op_slti||op_sltiu||op_andi||op_ori||op_xori||
                                  ctrl_sig[`I_MEM_R]||ctrl_sig[`I_MEM_W];
  // read [rt]
  assign ctrl_sig[`I_RT_R]      = op_sll||op_srl||op_sra||op_sllv||op_srlv||op_srav||op_mult||op_multu||op_div||op_divu||
                                  op_add||op_addu||op_sub||op_subu||op_and||op_or||op_xor||op_nor||op_slt||op_sltu||
                                  op_beq||op_bne||op_mtc0||op_lwl||op_lwr||ctrl_sig[`I_MEM_W];
  // write [rt] in ex stage
  assign ctrl_sig[`I_RT_WEX]    = op_addi||op_addiu||op_slti||op_sltiu||op_andi||op_ori||op_xori||op_lui||op_mfc0;
  // write [rt] in wb stage
  assign ctrl_sig[`I_RT_WWB]    = ctrl_sig[`I_MEM_R];
  // write [rd] in ex stage
  assign ctrl_sig[`I_RD_W]      = op_sll||op_srl||op_sra||op_sllv||op_srlv||op_srav||op_jr||op_jalr||op_mfhi||op_mflo||
                                  op_add||op_addu||op_sub||op_subu||op_and||op_or||op_xor||op_nor||op_slt||op_sltu;
  // write [31] in ex stage
  assign ctrl_sig[`I_R31_W]     = op_bltzal||op_bgezal||op_jal;
  // imm is sign-extended
  assign ctrl_sig[`I_IMM_SX]    = !(op_andi||op_ori||op_xori);
  // alu operand a is sa
  assign ctrl_sig[`I_ALU_A_SA]  = op_sll||op_srl||op_sra;
  // alu operand b is imm
  assign ctrl_sig[`I_ALU_B_IMM] = op_addi||op_addiu||op_slti||op_sltiu||op_andi||op_ori||op_xori||ctrl_sig[`I_MEM_R]||ctrl_sig[`I_MEM_W];
  assign ctrl_sig[`I_LINK]      = op_jal||op_jalr||op_bgezal||op_bltzal;
  assign ctrl_sig[`I_DO_MUL]    = op_mult||op_multu;
  assign ctrl_sig[`I_DO_DIV]    = op_div||op_divu;
  assign ctrl_sig[`I_MD_SIGN]   = op_mult||op_div;
  assign ctrl_sig[`I_EXC_OF]    = op_add || op_sub || op_addi;

  //////////////////// exception out ////////////////////
  assign exc_out = valid_in && !exc_in && (op_syscall || op_break || op_eret || reserved);
  assign exccode_out = {5{op_syscall}} & `EXC_SYS
                     | {5{op_break}} & `EXC_BP
                     | {5{op_eret}} & `EXC_ERET
                     | {5{reserved}} & `EXC_RI;

  assign rf_raddr1 = `GET_RS(instr_in);
  assign rf_raddr2 = `GET_RT(instr_in);

  // data forwarding
  assign rdata1_out = (rf_raddr1 == ex_addr && ex_fwd) ? ex_data
                           : (rf_raddr1 == mem_addr && mem_fwd) ? mem_data
                           : (rf_raddr1 == wb_addr && wb_fwd) ? wb_data
                           : rf_rdata1;
  assign rdata2_out = (rf_raddr2 == ex_addr && ex_fwd) ? ex_data
                           : (rf_raddr2 == mem_addr && mem_fwd) ? mem_data
                           : (rf_raddr2 == wb_addr && wb_fwd) ? wb_data
                           : rf_rdata2;

  // stall until writeback for instructions which require rf data in de stage
  wire data_hazard = op_bne||op_beq||op_bgez||op_bgezal||op_blez||op_bgtz||op_bltz||op_bltzal||op_jr||op_jalr;
  wire fwd_stall = ex_fwd && (!ex_valid || data_hazard) && (rf_raddr1 == ex_addr || rf_raddr2 == ex_addr)
                 || mem_fwd && (!mem_valid || data_hazard) && (rf_raddr1 == mem_addr || rf_raddr2 == mem_addr)
                 || wb_fwd && (!wb_valid || data_hazard) && (rf_raddr1 == wb_addr || rf_raddr2 == wb_addr);

  wire br_inst = op_bne||op_beq||op_bgez||op_bgezal||op_blez||op_bgtz||op_bltz||op_bltzal||op_j||op_jr||op_jal||op_jalr;

  wire this_stall = valid_in && !exc_in && (fwd_stall || br_inst && !ok_to_branch);
  assign valid = valid_in && !exc_in && !this_stall;

  // branch delay slot
  reg prev_branch; // if previous instruction is branch/jump
  always @(posedge clk) begin
    if (!resetn)
      prev_branch <= 1'b0;
    else if (valid && !stall_in)
      prev_branch <= br_inst;
  end

  // Branch test
  wire branch_taken = (op_bne && (rf_rdata1 != rf_rdata2)) ||
                      (op_beq && (rf_rdata1 == rf_rdata2)) ||
                      ((op_bgez||op_bgezal) && !rf_rdata1[31]) ||
                      (op_blez && (rf_rdata1[31] || rf_rdata1 == 32'd0)) ||
                      (op_bgtz && !(rf_rdata1[31] || rf_rdata1 == 32'd0)) ||
                      ((op_bltz||op_bltzal) && rf_rdata1[31]);

  assign pc_update = valid && !stall_in && (op_j||op_jr||op_jal||op_jalr||branch_taken);

  wire [15:0] imm = `GET_IMM(instr_in);
  wire [31:0] pc_ninstr = pc_in + 32'd4;
  wire [31:0] pc_branch = pc_ninstr + {{14{imm[15]}}, imm, 2'd0};
  wire [31:0] pc_jump = {pc_ninstr[31:28], `GET_INDEX(instr_in), 2'd0};
  assign pc_new = {32{branch_taken}} & pc_branch
                | {32{op_jr||op_jalr}} & rf_rdata1
                | {32{op_j||op_jal}} & pc_jump;

  assign valid_out = valid_in && !this_stall;
  assign stall_out = this_stall || valid_in && stall_in;
  assign ctrl_out = ctrl_sig;
  assign bd_out = prev_branch;

endmodule
