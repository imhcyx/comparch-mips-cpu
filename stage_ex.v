`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/09/22 09:26:01
// Design Name: 
// Module Name: stage_ex
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

module stage_ex(

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

    output              cp0_write,
    output [4:0]        cp0_reg,
    output [2:0]        cp0_sel,
    output [31:0]       cp0_wdata,
    input [31:0]        cp0_rdata,

    // Special interfaces
    input [31:0]        instr_in,
    input [`I_MAX-1:0]  ctrl_in,
    input [31:0]        rdata1_in,
    input [31:0]        rdata2_in,
    output [63:0]       mul_res,
    output [31:0]       div_s,
    output [31:0]       div_r,
    input               hilo_lock,
    input [31:0]        hi,
    input [31:0]        lo,
    output              wb,
    output              wb_valid,
    output [4:0]        wb_addr,
    output [31:0]       wb_data

);

  wire valid;
  wire [`I_MAX-1:0] ctrl_sig = ctrl_in;
  
  // imm extension
  wire [15:0] imm = `GET_IMM(instr_in);
  wire [31:0] imm_sx = {{16{imm[15]}}, imm};
  wire [31:0] imm_zx = {16'd0, imm};
  wire [31:0] imm_32 = ctrl_sig[`I_IMM_SX] ? imm_sx : imm_zx;

  // ALU operation
  wire [10:0] alu_op = ctrl_sig[10:0];

  // ALU module
  wire [31:0] alu_a, alu_b, alu_res_wire;
  wire alu_cf, alu_of, alu_zf;
  alu alu_instance(
    .A(alu_a),
    .B(alu_b),
    .ALUop(alu_op),
    .CarryOut(alu_cf),
    .Overflow(alu_of),
    .Zero(alu_zf),
    .Result(alu_res_wire)
  );

  wire ex_overflow = ctrl_sig[`I_EXC_OF] && alu_of;

  //////////////////// exception out ////////////////////
  assign exc_out = valid_in && !exc_in && ex_overflow;
  assign exccode_out = {5{ex_overflow}} & `EXC_OV;

  // select operand sources
  assign alu_a = ctrl_sig[`I_ALU_A_SA] ? {27'd0, `GET_SA(instr_in)} : rdata1_in;
  assign alu_b = ctrl_sig[`I_ALU_B_IMM] ? imm_32 : rdata2_in;

  // multiplication
  mul u_mul(
    .mul_clk(clk),
    .resetn(resetn),
    .mul_signed(ctrl_sig[`I_MD_SIGN]),
    .x(rdata1_in),
    .y(rdata2_in),
    .result(mul_res)
  );

  // division
  wire div_complete;
  div u_div(
    .div_clk(clk),
    .resetn(resetn),
    .div(ctrl_sig[`I_DO_DIV] && valid_in && !exc_in),
    .div_signed(ctrl_sig[`I_MD_SIGN]),
    .x(rdata1_in),
    .y(rdata2_in),
    .s(div_s),
    .r(div_r),
    .complete(div_complete)
  );

  // cp0 operation
  assign cp0_write = valid && ctrl_sig[`I_MTC0];
  assign cp0_reg = `GET_RD(instr_in);
  assign cp0_sel = instr_in[2:0];
  assign cp0_wdata = rdata2_in;

  assign wb_data = {32{ctrl_sig[`I_MFHI]}} & hi
                          | {32{ctrl_sig[`I_MFLO]}} & lo
                          | {32{ctrl_sig[`I_LUI]}} & {`GET_IMM(instr_in), 16'd0}
                          | {32{ctrl_sig[`I_LINK]}} & (pc_in + 32'd8)
                          | {32{ctrl_sig[`I_MFC0]}} & cp0_rdata
                          | {32{|alu_op}} & alu_res_wire;

  assign wb = valid_in && !exc_in && (ctrl_sig[`I_RT_WEX] || ctrl_sig[`I_RT_WWB] || ctrl_sig[`I_RD_W] || ctrl_sig[`I_R31_W]);
  assign wb_valid = valid_in && !exc_in && !stall_out && (ctrl_sig[`I_RT_WEX] || ctrl_sig[`I_RD_W] || ctrl_sig[`I_R31_W]);
  assign wb_addr =
    {5{valid_in && !exc_in && (ctrl_sig[`I_RT_WEX]||ctrl_sig[`I_RT_WWB])}} & `GET_RT(instr_in) |
    {5{valid_in && !exc_in && ctrl_sig[`I_RD_W]}} & `GET_RD(instr_in) |
    {5{valid_in && !exc_in && ctrl_sig[`I_R31_W]}} & 5'd31;

  wire this_stall = valid_in && !exc_in &&
                   ((ctrl_sig[`I_MFHI]||ctrl_sig[`I_MFLO]) && hilo_lock || ctrl_sig[`I_DO_DIV] && !div_complete);
  assign valid = valid_in && !exc_in && !this_stall;

  assign valid_out = valid_in && !this_stall;
  assign stall_out = this_stall || valid_in && stall_in;

endmodule
