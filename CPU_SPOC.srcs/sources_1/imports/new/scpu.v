`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/04/02 16:28:45
// Design Name: 
// Module Name: scpu
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



module SCPU(
    input wire MIO_ready,
    input wire [31:0] Data_in,
    input wire clk,
    input wire [31:0] inst_in,
    input wire rst,
    input wire out_io,
    output wire [31:0] ALU_out,
    output wire [31:0] Data_out,
    output wire [31:0] PC_out,
    output wire [1:0] MemRW,
    output wire [1023:0] Reg_value,
    output wire [31:0] mepc
    );

    wire [2:0] ImmSel;
    wire [3:0] ALU_Control;
    wire ALUSrc_B;
    wire [1:0] MemtoReg;
    wire [1:0] Jump;
    wire Branch;
    wire BranchN;
    wire RegWrite;
    wire restore;
    // wire MemRW;
    wire CPU_MIO;
    wire mret,ecall,ill_instr,INT;
    reg [1:0] check_int = 2'b0;
    always@(posedge clk)begin
        check_int <= {check_int[0],out_io};
    end
    assign INT = (check_int == 2'b01);
    SCPU_ctrl_more ctrl_unit (.inst_field(inst_in),.MIO_ready(MIO_ready),.restore(restore)
    ,.ImmSel(ImmSel),.ALUSrc_B(ALUSrc_B),.MemtoReg(MemtoReg),.Jump(Jump),.Branch(Branch),.BranchN(BranchN),.RegWrite(RegWrite),
    .MemRW(MemRW),.ALU_Control(ALU_Control),.CPU_MIO(CPU_MIO),.mret(mret),.ecall(ecall),.ill_instr(ill_instr));
    Data_path_more datapath_unit (.clk(clk),.rst(rst),.inst_field(inst_in),.ALUSrc_B(ALUSrc_B),.MemtoReg(MemtoReg),.Jump(Jump),.Branch(Branch),
    .BranchN(BranchN),.RegWrite(RegWrite),.Data_in(Data_in),.ALU_Control(ALU_Control),.ImmSel(ImmSel),.ALU_out(ALU_out),
    .Data_out(Data_out),.PC_out(PC_out),.Reg_value(Reg_value),.ecall(ecall),.ill_instr(ill_instr),
    .mret(mret),.INT(INT),.mepc(mepc),.state(restore));

endmodule

