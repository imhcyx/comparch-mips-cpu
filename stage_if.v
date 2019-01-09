`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/09/20 10:41:07
// Design Name: 
// Module Name: stage_if
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

module stage_if(

    // Common interfaces
    input               clk,
    input               resetn,
    input               valid_in,
    output              valid_out,
    input               stall_in,
    output              stall_out,
    input [31:0]        pc_in,
    output [31:0]       pc_out,

    // Exception interfaces
    output              exc_out,
    output [4:0]        exccode_out,
    input               intr,

    // Special interfaces
    output              inst_req,
    output [31:0]       inst_addr,
    input [31:0]        inst_rdata,
    input               inst_addr_ok,
    input               inst_data_ok,
    output [31:0]       instruction,
    output              status

);

  reg stat, stat_next; // 0=req 1=wait
  reg [31:0] instr;
  reg instr_fetched;

  wire if_adel = pc_in[1:0] != 2'd0 && stat == 1'b0;
  //////////////////// exception out ////////////////////
  assign exc_out = valid_in && (intr && instr_fetched || if_adel);
  assign exccode_out = intr ? `EXC_INT : `EXC_ADEL;

  always @(posedge clk) begin
    if (!resetn)    stat <= 1'b0;
    else            stat <= stat_next;
  end

  always @(*) begin
    if (stat == 1'b0)   stat_next = inst_addr_ok ? 1'b1 : 1'b0;
    else                stat_next = instr_fetched && !stall_in ? 1'b0 : 1'b1;
  end

  assign status = stat;

  assign inst_req = resetn && valid_in && stat == 1'b0 && !if_adel;
  assign inst_addr = pc_in;

  reg [31:0] pc_save;
  always @(posedge clk) if (inst_addr_ok) pc_save <= pc_in;
  assign pc_out = if_adel ? pc_in : pc_save;

  always @(posedge clk) begin
    if (if_adel) instr <= 32'd0;
    else if (inst_data_ok) instr <= inst_rdata;
  end
  always @(posedge clk) instr_fetched <= inst_data_ok || instr_fetched && stall_in;
  assign instruction = instr;

  wire this_stall = valid_in && !instr_fetched && !if_adel;

  assign valid_out = valid_in && !this_stall;
  assign stall_out = this_stall || valid_in && stall_in; // unused

endmodule
