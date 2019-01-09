`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/09/22 10:13:46
// Design Name: 
// Module Name: stage_mem
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

module stage_mem(

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
    input               exc_stall,

    // Special interfaces
    input [31:0]        instr_in,
    input [`I_MAX-1:0]  ctrl_in,
    input [31:0]        rdata1_in,
    input [31:0]        rdata2_in,
    input [31:0]        result_in,
    output              data_req,
    output              data_wr,
    output [3:0]        data_wstrb,
    output [31:0]       data_addr,
    output [31:0]       data_wdata,
    input               data_addr_ok,
    input [63:0]        mul_res,
    input [31:0]        div_s,
    input [31:0]        div_r,
    output              hilo_lock,
    output reg [31:0]   hi,
    output reg [31:0]   lo,
    output              wb,
    output              wb_valid,
    output [4:0]        wb_addr,
    output [31:0]       wb_data
);

  wire valid;

  wire [`I_MAX-1:0] ctrl_sig = ctrl_in;

  wire [31:0] mem_addr_aligned = result_in & 32'hfffffffc;
  wire [1:0] mem_byte_offset = result_in[1:0];
  wire [1:0] mem_byte_offsetn = ~mem_byte_offset;

  wire mem_adel = ctrl_sig[`I_LW] && result_in[1:0] != 2'd0
               || (ctrl_sig[`I_LH] || ctrl_sig[`I_LHU]) && result_in[0] != 1'd0;
  wire mem_ades = ctrl_sig[`I_SW] && result_in[1:0] != 2'd0
               || ctrl_sig[`I_SH] && result_in[0] != 1'd0;

  //////////////////// exception out ////////////////////
  assign exc_out = valid_in && !exc_in && (mem_adel || mem_ades);
  assign exccode_out = {5{mem_adel}} & `EXC_ADEL
                     | {5{mem_ades}} & `EXC_ADES;

  wire mem_read = ctrl_sig[`I_MEM_R] && !mem_adel;
  wire mem_write = ctrl_sig[`I_MEM_W] && !mem_ades;

  assign data_req = resetn && valid_in && !exc_in && !stall_in && (mem_read || mem_write);
  assign data_wr = mem_write;

  // mem write mask
  assign data_wstrb =
    {4{ctrl_sig[`I_SW]}} & 4'b1111 |
    {4{ctrl_sig[`I_SH]}} & (4'b0011 << mem_byte_offset) |
    {4{ctrl_sig[`I_SB]}} & (4'b0001 << mem_byte_offset) |
    {4{ctrl_sig[`I_SWL]}} & (4'b1111 >> mem_byte_offsetn) |
    {4{ctrl_sig[`I_SWR]}} & (4'b1111 << mem_byte_offset);

  // mem write data
  assign data_wdata =
    {32{ctrl_sig[`I_SW]}} & rdata2_in |
    {32{ctrl_sig[`I_SH]}} & {rdata2_in[15:0], rdata2_in[15:0]} |
    {32{ctrl_sig[`I_SB]}} & {rdata2_in[7:0], rdata2_in[7:0], rdata2_in[7:0], rdata2_in[7:0]} |
    {32{ctrl_sig[`I_SWL]}} & (rdata2_in >> (8 * mem_byte_offsetn)) |
    {32{ctrl_sig[`I_SWR]}} & (rdata2_in << (8 * mem_byte_offset));

  assign data_addr = mem_addr_aligned;

  wire wait_data = data_req && !data_addr_ok;

  // process mul & div

  assign hilo_lock = valid && ctrl_sig[`I_DO_MUL]||ctrl_sig[`I_DO_DIV]||ctrl_sig[`I_MTHI]||ctrl_sig[`I_MTLO];
  always @(posedge clk) begin
    if (valid && ctrl_sig[`I_DO_MUL]) begin
      hi <= mul_res[63:32];
      lo <= mul_res[31:0];
    end
    else if (valid && ctrl_sig[`I_DO_DIV]) begin
      hi <= div_r;
      lo <= div_s;
    end
    else begin
      if (valid && ctrl_sig[`I_MTHI]) hi <= rdata1_in;
      if (valid && ctrl_sig[`I_MTLO]) lo <= rdata1_in;
    end
  end

  assign wb = valid_in && !exc_in && (ctrl_sig[`I_RT_WEX] || ctrl_sig[`I_RT_WWB] || ctrl_sig[`I_RD_W] || ctrl_sig[`I_R31_W]);
  assign wb_valid = valid_in && !exc_in && !stall_out && (ctrl_sig[`I_RT_WEX] || ctrl_sig[`I_RD_W] || ctrl_sig[`I_R31_W]);
  assign wb_addr =
    {5{valid_in && !exc_in && (ctrl_sig[`I_RT_WEX]||ctrl_sig[`I_RT_WWB])}} & `GET_RT(instr_in) |
    {5{valid_in && !exc_in && ctrl_sig[`I_RD_W]}} & `GET_RD(instr_in) |
    {5{valid_in && !exc_in && ctrl_sig[`I_R31_W]}} & 5'd31;
  assign wb_data = result_in;

  wire this_stall = valid_in && !exc_in && wait_data || exc_stall;
  assign valid = valid_in && !exc_in && !this_stall;

  assign valid_out = valid_in && !this_stall;
  assign stall_out = this_stall || valid_in && stall_in;

endmodule
