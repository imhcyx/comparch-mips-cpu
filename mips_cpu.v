`timescale 10ns / 1ns

`include "common.vh"

module mips_cpu(
    input            clk,
    input            resetn,            //low active

    input [5:0] int,

    (*mark_debug = "true"*)output inst_req,
    (*mark_debug = "true"*)output [31:0] inst_addr,
    (*mark_debug = "true"*)input [31:0] inst_rdata,
    (*mark_debug = "true"*)input inst_addr_ok,
    (*mark_debug = "true"*)input inst_data_ok,
    (*mark_debug = "true"*)output data_req,
    (*mark_debug = "true"*)output data_wr,
    (*mark_debug = "true"*)output [3:0] data_wstrb,
    (*mark_debug = "true"*)output [31:0] data_addr,
    (*mark_debug = "true"*)output [31:0] data_wdata,
    (*mark_debug = "true"*)input [31:0] data_rdata,
    (*mark_debug = "true"*)input data_addr_ok,
    (*mark_debug = "true"*)input data_data_ok,

    //debug interface
    output  [31:0]   debug_wb_pc,
    output  [3 :0]   debug_wb_rf_wen,
    output  [4 :0]   debug_wb_rf_wnum,
    output  [31:0]   debug_wb_rf_wdata
);

  // Hardware interrupt
  wire [5:0] intr;

  // CP0 registers

  reg [7:0] sr_im;
  reg sr_exl, sr_ie;
  reg [5:0] cause_ip_7_2;
  reg [1:0] cause_ip_1_0;
  reg cause_bd, cause_ti;
  reg [4:0] cause_exccode;

  wire [7:0] cause_ip = {cause_ip_7_2, cause_ip_1_0};

  reg [31:0] cp0_badvaddr;  // 8    0
  reg [31:0] cp0_count;     // 9    0
  reg [31:0] cp0_compare;   // 11   0
  reg [31:0] cp0_epc;       // 14   0
  wire [31:0] cp0_sr = {9'd0, 1'b1, 6'd0, sr_im, 6'd0, sr_exl, sr_ie};  // 12   0
  wire [31:0] cp0_cause = {cause_bd, cause_ti, 14'd0, cause_ip, 1'd0, cause_exccode, 2'd0}; // 13   0
  
  // software read/write
  wire cp0_write;
  wire [4:0] cp0_reg;
  wire [2:0] cp0_sel;
  wire [31:0] cp0_wdata;
  wire [31:0] cp0_rdata;
  assign cp0_rdata = {32{cp0_reg == 5'd8 && cp0_sel == 3'd0}} & cp0_badvaddr
                   | {32{cp0_reg == 5'd9 && cp0_sel == 3'd0}} & cp0_count
                   | {32{cp0_reg == 5'd11 && cp0_sel == 3'd0}} & cp0_compare
                   | {32{cp0_reg == 5'd12 && cp0_sel == 3'd0}} & cp0_sr
                   | {32{cp0_reg == 5'd13 && cp0_sel == 3'd0}} & cp0_cause
                   | {32{cp0_reg == 5'd14 && cp0_sel == 3'd0}} & cp0_epc
                   ;
  
  // hardware write
  wire hw_exc;
  wire hw_excbd;
  wire [4:0] hw_exccode;
  wire [31:0] hw_epc, hw_badvaddr;
  wire hw_eret;
  
  always @(posedge clk) begin
    if (!resetn) sr_im <= 8'd0;
    else if (cp0_write && cp0_reg == 5'd12 && cp0_sel == 5'd0) sr_im <= cp0_wdata[15:8];
  end

  always @(posedge clk) begin
    if (!resetn) sr_exl <= 1'd0;
    else if (hw_exc) sr_exl <= !hw_eret;
    else if (cp0_write && cp0_reg == 5'd12 && cp0_sel == 5'd0) sr_exl = cp0_wdata[1];
  end

  always @(posedge clk) begin
    if (!resetn) sr_ie <= 1'd0;
    else if (cp0_write && cp0_reg == 5'd12 && cp0_sel == 5'd0) sr_ie <= cp0_wdata[0];
  end

  always @(posedge clk) begin
    if (!resetn) cause_bd <= 1'd0;
    else if (hw_exc && !sr_exl) cause_bd <= hw_excbd;
  end

  always @(posedge clk) begin
    if (!resetn) cause_ti <= 1'd0;
  end

  always @(posedge clk) begin
    if (!resetn) cause_ip_7_2 <= 6'd0;
    else cause_ip_7_2 <= intr;
  end

  always @(posedge clk) begin
    if (!resetn) cause_ip_1_0 <= 2'd0;
    else if (cp0_write && cp0_reg == 5'd13 && cp0_sel == 5'd0) cause_ip_1_0 <= cp0_wdata[9:8];
  end

  always @(posedge clk) begin
    if (!resetn) cause_exccode <= 5'd0;
    else if (hw_exc) cause_exccode <= hw_exccode;
  end

  always @(posedge clk) begin
    if (!resetn) cp0_badvaddr <= 32'd0;
    else if (hw_exc && (hw_exccode == `EXC_ADEL || hw_exccode == `EXC_ADES)) cp0_badvaddr <= hw_badvaddr;
    else if (cp0_write && cp0_reg == 5'd8 && cp0_sel == 3'd0) cp0_badvaddr <= cp0_wdata;
  end

  always @(posedge clk) begin
    if (!resetn) cp0_epc <= 32'd0;
    else if (hw_exc && !hw_eret && !sr_exl) cp0_epc <= hw_epc;
    else if (cp0_write && cp0_reg == 5'd14 && cp0_sel == 3'd0) cp0_epc <= cp0_wdata;
  end

  reg tick;
  always @(posedge clk) begin
    if (!resetn) tick <= 1'b0;
    else tick <= ~tick;
  end

  always @(posedge clk) begin
    if (!resetn) cp0_count <= 32'd0;
    else if (cp0_write && cp0_reg == 5'd9 && cp0_sel == 3'd0) cp0_count <= cp0_wdata;
    else if (tick == 1'b0) cp0_count <= cp0_count + 32'd1;
  end

  always @(posedge clk) begin
    if (!resetn) cp0_compare <= 32'd0;
    else if (cp0_write && cp0_reg == 5'd11 && cp0_sel == 3'd0) cp0_compare <= cp0_wdata;
  end

  reg clk_intr;
  always @(posedge clk) begin
    if (!resetn) clk_intr <= 1'b0;
    else if (cp0_write && cp0_reg == 5'd11 && cp0_sel == 3'd0) clk_intr <= 1'b0;
    else if (cp0_count == cp0_compare) clk_intr <= 1'b1;
  end

  assign intr[0] = int[0];
  assign intr[1] = int[1];
  assign intr[2] = int[2];
  assign intr[3] = int[3];
  assign intr[4] = int[4];
  assign intr[5] = clk_intr;

 ///////////////////////// Pipeline /////////////////////////

  wire de_pc_update;
  wire [31:0] de_pc_new;

  wire if_valid, de_valid, ex_valid, mem_valid, wb_valid;
  wire if_stall, de_stall, ex_stall, mem_stall, wb_stall;
  wire if_exc, de_exc, ex_exc, mem_exc;
  wire [4:0] if_exccode, de_exccode, ex_exccode, mem_exccode;
  (*mark_debug = "true"*)reg if_de_valid, de_ex_valid, ex_mem_valid, mem_wb_valid;
  (*mark_debug = "true"*)reg [31:0] if_de_pc, de_ex_pc, ex_mem_pc, mem_wb_pc;
  (*mark_debug = "true"*)reg if_de_exc, de_ex_exc, ex_mem_exc;
  reg [4:0] if_de_exccode, de_ex_exccode, ex_mem_exccode;
  reg de_ex_bd, ex_mem_bd;

  /*** IF ***/

  reg [31:0] pc;
  wire [31:0] if_pc, if_pc_out;

  assign if_pc = hw_eret ? cp0_epc
               : hw_exc ? `EXCEPTION_PC
               : de_pc_update ? de_pc_new
               : pc;

  always @(posedge clk) begin
    if (!resetn) pc <= `INITIAL_PC;
    else if (hw_exc || de_pc_update) pc <= if_pc;
    else if (inst_addr_ok) pc <= if_pc + 32'd4;
  end

  wire if_stat;
  wire [31:0] if_instr;

/*
  reg if_valid_in;
  always @(posedge clk) begin
    if (!resetn || hw_exc && if_stat == 1'b0) if_valid_in <= 1'b1;
    else if (if_exc || de_exc || ex_exc || mem_exc) if_valid_in <= 1'b0;
  end
*/

  stage_if u_stage_if(
    .clk(clk),
    .resetn(resetn),
    .valid_in(!(if_de_exc || de_ex_exc || ex_mem_exc || de_exc || ex_exc || mem_exc)),
    .valid_out(if_valid),
    .stall_in(de_stall),
    .stall_out(if_stall),
    .pc_in(if_pc),
    .pc_out(if_pc_out),
    .exc_out(if_exc),
    .exccode_out(if_exccode),
    .intr(sr_ie && !sr_exl && |(cause_ip & sr_im)),
    .inst_req(inst_req),
    .inst_addr(inst_addr),
    .inst_rdata(inst_rdata),
    .inst_addr_ok(inst_addr_ok),
    .inst_data_ok(inst_data_ok),
    .instruction(if_instr),
    .status(if_stat)
  );

  /** IF/DE **/

  reg [31:0] if_de_instr;

  always @(posedge clk) begin
    if (!resetn) begin
      if_de_valid <= 1'b0;
      if_de_exc <= 1'b0;
      if_de_exccode <= 5'd0;
    end
    else if (!de_stall) begin
      if_de_valid <= if_valid;
      if_de_exc <= !hw_exc && if_exc;
      if_de_exccode <= if_exccode;
    end
  end

  always @(posedge clk) begin
    if (!de_stall) begin
      if_de_pc <= if_pc_out;
      if_de_instr <= if_instr;
    end
  end

  /*** DE ***/

  // Register file
  wire [4:0] rf_raddr1, rf_raddr2;
  wire [31:0] rf_rdata1, rf_rdata2;
  wire fwd_ex, fwd_mem, fwd_wb;
  wire fwd_ex_valid, fwd_mem_valid, fwd_wb_valid;
  wire [4:0] fwd_ex_addr, fwd_mem_addr, fwd_wb_addr;
  wire [31:0] fwd_ex_data, fwd_mem_data, fwd_wb_data;
  reg_file rf_instance(
    .clk(clk),
    .waddr(fwd_wb_addr),
    .raddr1(rf_raddr1),
    .raddr2(rf_raddr2),
    .wen(fwd_wb_valid),
    .wdata(fwd_wb_data),
    .rdata1(rf_rdata1),
    .rdata2(rf_rdata2)
  );

  wire [`I_MAX-1:0] de_ctrl;
  wire [31:0] de_rdata1, de_rdata2;
  wire de_bd;

  stage_de u_stage_de(
    .clk(clk),
    .resetn(resetn),
    .valid_in(if_de_valid && !(ex_exc || mem_exc)),
    .valid_out(de_valid),
    .stall_in(ex_stall),
    .stall_out(de_stall),
    .pc_in(if_de_pc),
    .exc_in(if_de_exc),
    .exc_out(de_exc),
    .exccode_out(de_exccode),
    .bd_out(de_bd),
    .instr_in(if_de_instr),
    .ctrl_out(de_ctrl),
    .rf_raddr1(rf_raddr1),
    .rf_raddr2(rf_raddr2),
    .rf_rdata1(rf_rdata1),
    .rf_rdata2(rf_rdata2),
    .rdata1_out(de_rdata1),
    .rdata2_out(de_rdata2),
    .pc_update(de_pc_update),
    .pc_new(de_pc_new),
    .ex_fwd(fwd_ex),
    .ex_valid(fwd_ex_valid),
    .ex_addr(fwd_ex_addr),
    .ex_data(fwd_ex_data),
    .mem_fwd(fwd_mem),
    .mem_valid(fwd_mem_valid),
    .mem_addr(fwd_mem_addr),
    .mem_data(fwd_mem_data),
    .wb_fwd(fwd_wb),
    .wb_valid(fwd_wb_valid),
    .wb_addr(fwd_wb_addr),
    .wb_data(fwd_wb_data),
    .ok_to_branch(if_stat == 1'b1)
  );

  /** DE/EX **/

  reg [31:0] de_ex_instr;
  reg [`I_MAX-1:0] de_ex_ctrl;
  reg [31:0] de_ex_rdata1, de_ex_rdata2;

  always @(posedge clk) begin
    if (!resetn) begin
      de_ex_valid <= 1'b0;
      de_ex_exc <= 1'b0;
      de_ex_exccode <= 5'd0;
    end
    else if (!ex_stall) begin
      de_ex_valid <= de_valid;
      de_ex_exc <= !hw_exc && (if_de_exc || de_exc);
      de_ex_exccode <= de_exc ? de_exccode : if_de_exccode;
    end
  end

  always @(posedge clk) begin
    if (!ex_stall) begin
      de_ex_pc <= if_de_pc;
      de_ex_ctrl <= de_ctrl;
      de_ex_instr <= if_de_instr;
      de_ex_rdata1 <= de_rdata1;
      de_ex_rdata2 <= de_rdata2;
      de_ex_bd <= de_bd;
    end
  end

  /*** EX ***/

  wire [63:0] mul_res;
  wire [31:0] div_s;
  wire [31:0] div_r;
  wire hilo_lock;
  wire [31:0] hi;
  wire [31:0] lo;

  stage_ex u_stage_ex(
    .clk(clk),
    .resetn(resetn),
    .valid_in(de_ex_valid && !mem_exc),
    .valid_out(ex_valid),
    .stall_in(mem_stall),
    .stall_out(ex_stall),
    .pc_in(de_ex_pc),
    .exc_in(de_ex_exc),
    .exc_out(ex_exc),
    .exccode_out(ex_exccode),
    .cp0_write(cp0_write),
    .cp0_reg(cp0_reg),
    .cp0_sel(cp0_sel),
    .cp0_wdata(cp0_wdata),
    .cp0_rdata(cp0_rdata),
    .instr_in(de_ex_instr),
    .ctrl_in(de_ex_ctrl),
    .rdata1_in(de_ex_rdata1),
    .rdata2_in(de_ex_rdata2),
    .mul_res(mul_res),
    .div_s(div_s),
    .div_r(div_r),
    .hilo_lock(hilo_lock),
    .hi(hi),
    .lo(lo),
    .wb(fwd_ex),
    .wb_valid(fwd_ex_valid),
    .wb_addr(fwd_ex_addr),
    .wb_data(fwd_ex_data)
  );

  /** EX/MEM **/

  reg [31:0] ex_mem_instr;
  reg [`I_MAX-1:0] ex_mem_ctrl;
  reg [31:0] ex_mem_rdata1, ex_mem_rdata2, ex_mem_result;

  always @(posedge clk) begin
    if (!resetn) begin
      ex_mem_valid <= 1'b0;
      ex_mem_exc <= 1'b0;
      ex_mem_exccode <= 5'd0;
    end
    else if (!mem_stall) begin
      ex_mem_valid <= ex_valid;
      ex_mem_exc <= !hw_exc && (de_ex_exc || ex_exc);
      ex_mem_exccode <= ex_exc ? ex_exccode : de_ex_exccode;
    end
  end

  always @(posedge clk) begin
    if (!mem_stall) begin
      ex_mem_pc <= de_ex_pc;
      ex_mem_ctrl <= de_ex_ctrl;
      ex_mem_instr <= de_ex_instr;
      ex_mem_rdata1 <= de_ex_rdata1;
      ex_mem_rdata2 <= de_ex_rdata2;
      ex_mem_result <= fwd_ex_data;
      ex_mem_bd <= de_ex_bd;
    end
  end

  /*** MEM ***/

  stage_mem u_stage_mem(
    .clk(clk),
    .resetn(resetn),
    .valid_in(ex_mem_valid),
    .valid_out(mem_valid),
    .stall_in(wb_stall),
    .stall_out(mem_stall),
    .pc_in(ex_mem_pc),
    .exc_in(ex_mem_exc),
    .exc_out(mem_exc),
    .exccode_out(mem_exccode),
    .exc_stall(hw_exc && if_stat == 1'b1),
    .instr_in(ex_mem_instr),
    .ctrl_in(ex_mem_ctrl),
    .rdata1_in(ex_mem_rdata1),
    .rdata2_in(ex_mem_rdata2),
    .result_in(ex_mem_result),
    .data_req(data_req),
    .data_wr(data_wr),
    .data_wstrb(data_wstrb),
    .data_addr(data_addr),
    .data_wdata(data_wdata),
    .data_addr_ok(data_addr_ok),
    .mul_res(mul_res),
    .div_s(div_s),
    .div_r(div_r),
    .hilo_lock(hilo_lock),
    .hi(hi),
    .lo(lo),
    .wb(fwd_mem),
    .wb_valid(fwd_mem_valid),
    .wb_addr(fwd_mem_addr),
    .wb_data(fwd_mem_data)
  );

  /** MEM/WB **/

  reg [31:0] mem_wb_instr;
  reg [`I_MAX-1:0] mem_wb_ctrl;
  reg [31:0] mem_wb_rdata2, mem_wb_result;

  always @(posedge clk) begin
    if (!resetn) begin
      mem_wb_valid <= 1'b0;
    end
    else if (!wb_stall) begin
      mem_wb_valid <= mem_valid && !hw_exc;
    end
  end

  always @(posedge clk) begin
    if (!wb_stall) begin
      mem_wb_pc <= ex_mem_pc;
      mem_wb_ctrl <= ex_mem_ctrl;
      mem_wb_instr <= ex_mem_instr;
      mem_wb_result <= ex_mem_result;
      mem_wb_rdata2 <= ex_mem_rdata2;
    end
  end

  /*** Exception ***/

  assign hw_exc = ex_mem_exc || mem_exc;
  assign hw_excbd = ex_mem_bd;
  assign hw_exccode = mem_exc ? mem_exccode : ex_mem_exccode;
  assign hw_epc = ex_mem_bd ? (ex_mem_pc - 32'd4) : ex_mem_pc;
  assign hw_badvaddr = (mem_exccode == `EXC_ADEL || mem_exccode == `EXC_ADES) ? ex_mem_result : ex_mem_pc;
  assign hw_eret = ex_mem_exccode == `EXC_ERET;

  /*** WB ***/

  stage_wb u_stage_wb(
    .clk(clk),
    .resetn(resetn),
    .valid_in(mem_wb_valid),
    .valid_out(wb_valid),
    .stall_in(1'b0),
    .stall_out(wb_stall),
    .pc_in(mem_wb_pc),
    .instr_in(mem_wb_instr),
    .ctrl_in(mem_wb_ctrl),
    .rdata2_in(mem_wb_rdata2),
    .result_in(mem_wb_result),
    .data_rdata(data_rdata),
    .data_data_ok(data_data_ok),
    .wb(fwd_wb),
    .wb_valid(fwd_wb_valid),
    .wb_addr(fwd_wb_addr),
    .wb_data(fwd_wb_data)
  );

  // mem_pc is the pc of stage_wb when reg_file write is performed
  assign debug_wb_pc = mem_wb_pc;
  assign debug_wb_rf_wen = {4{fwd_wb_valid}};
  assign debug_wb_rf_wnum = fwd_wb_addr;
  assign debug_wb_rf_wdata = fwd_wb_data;

endmodule

