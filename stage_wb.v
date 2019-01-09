`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/09/22 13:58:33
// Design Name: 
// Module Name: stage_wb
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

module stage_wb(

    // Common interfaces
    input               clk,
    input               resetn,
    input               valid_in,
    output              valid_out,
    input               stall_in,
    output              stall_out,
    input [31:0]        pc_in,

    // Special interfaces
    input [31:0]        instr_in,
    input [`I_MAX-1:0]  ctrl_in,
    input [31:0]        rdata2_in,
    input [31:0]        result_in,
    input [31:0]        data_rdata,
    input               data_data_ok,
    output              wb,
    output              wb_valid,
    output [4:0]        wb_addr,
    output [31:0]       wb_data

);

  wire valid;

  wire [`I_MAX-1:0] ctrl_sig = ctrl_in;

  reg [31:0] data;
  reg data_loaded;
  always @(posedge clk) if (data_data_ok) data <= data_rdata;
  always @(posedge clk) data_loaded <= data_data_ok || data_loaded && stall_in;

  // process length & extension for read
  wire [1:0] mem_byte_offset = result_in[1:0];
  wire [1:0] mem_byte_offsetn = ~mem_byte_offset;
  wire [7:0] mem_rdata_b = data >> (8 * mem_byte_offset);
  wire [15:0] mem_rdata_h = data >> (8 * mem_byte_offset);
  wire [31:0] mem_rdata_b_sx, mem_rdata_b_zx, mem_rdata_h_sx, mem_rdata_h_zx;
  assign mem_rdata_b_sx = {{24{mem_rdata_b[7]}}, mem_rdata_b};
  assign mem_rdata_b_zx = {24'd0, mem_rdata_b};
  assign mem_rdata_h_sx = {{16{mem_rdata_h[15]}}, mem_rdata_h};
  assign mem_rdata_h_zx = {16'd0, mem_rdata_h};

  wire [31:0] mem_rdata_b_res =
    {32{ctrl_sig[`I_LB]}} & mem_rdata_b_sx |
    {32{ctrl_sig[`I_LBU]}} & mem_rdata_b_zx;

  wire [31:0] mem_rdata_h_res =
    {32{ctrl_sig[`I_LH]}} & mem_rdata_h_sx |
    {32{ctrl_sig[`I_LHU]}} & mem_rdata_h_zx;

  // mem read mask
  wire [31:0] mem_rmask =
    {32{ctrl_sig[`I_LW]||ctrl_sig[`I_LH]||ctrl_sig[`I_LHU]||ctrl_sig[`I_LB]||ctrl_sig[`I_LBU]}} & 32'hffffffff |
    {32{ctrl_sig[`I_LWL]}} & (32'hffffffff << (8 * mem_byte_offsetn)) |
    {32{ctrl_sig[`I_LWR]}} & (32'hffffffff >> (8 * mem_byte_offset));
  // mem read data
  wire [31:0] memdata = rdata2_in & ~mem_rmask |
    {32{ctrl_sig[`I_LW]}} & data |
    {32{ctrl_sig[`I_LH]||ctrl_sig[`I_LHU]}} & mem_rdata_h_res |
    {32{ctrl_sig[`I_LB]||ctrl_sig[`I_LBU]}} & mem_rdata_b_res |
    {32{ctrl_sig[`I_LWL]}} & (data << (8 * mem_byte_offsetn)) |
    {32{ctrl_sig[`I_LWR]}} & (data >> (8 * mem_byte_offset));

  wire wait_data = (ctrl_sig[`I_MEM_R] || ctrl_sig[`I_MEM_W]) && !data_loaded; 

  assign wb = valid_in && (ctrl_sig[`I_RT_WEX] || ctrl_sig[`I_RT_WWB] || ctrl_sig[`I_RD_W] || ctrl_sig[`I_R31_W]);
  assign wb_valid = valid_in && !stall_out && (ctrl_sig[`I_RT_WEX] || ctrl_sig[`I_RT_WWB] || ctrl_sig[`I_RD_W] || ctrl_sig[`I_R31_W]);
  assign wb_addr =
    {5{valid_in && (ctrl_sig[`I_RT_WEX]||ctrl_sig[`I_RT_WWB])}} & `GET_RT(instr_in) |
    {5{valid_in && ctrl_sig[`I_RD_W]}} & `GET_RD(instr_in) |
    {5{valid_in && ctrl_sig[`I_R31_W]}} & 5'd31;
  assign wb_data = ctrl_sig[`I_MEM_R] ? memdata : result_in;

  wire this_stall = valid_in && wait_data;
  assign valid = valid_in && !this_stall;

  assign valid_out = valid_in && !this_stall;
  assign stall_out = this_stall || valid && stall_in;

endmodule
