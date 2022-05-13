`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/04/11 16:00:22
// Design Name: 
// Module Name: sim_ctrolunit
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


module sim_ctrolunit;
reg [31:0] inst_field;
reg MIO_ready;
wire [2:0] ImmSel;
wire ALUSrc_B;
wire [1:0] MemtoReg;
wire [1:0] Jump;
wire Branch;
wire BranchN;
wire RegWrite;
wire MemRW;
wire [3:0] ALU_Control;
wire CPU_MIO;
SCPU_ctrl_more test0(.inst_field(inst_field),.restore(1'b0),.ImmSel(ImmSel),.ALUSrc_B(ALUSrc_B),.MemtoReg(MemtoReg),
.Jump(Jump),.Branch(Branch),.BranchN(BranchN),.RegWrite(RegWrite),.MemRW(MemRW),.ALU_Control(ALU_Control)); 

initial begin
inst_field = 0;
#40;
// add 
inst_field[6:0] = 7'b0110011;
inst_field[14:12] = 3'b000;
inst_field[30] = 0;
#20 
// sltu
inst_field[6:0] = 7'b0110011;
inst_field[14:12] = 3'b011;
inst_field[30] = 0;
// sra
inst_field[6:0] = 7'b0110011;
inst_field[14:12] = 3'b101;
inst_field[30] = 1;
#20
// sub
inst_field[6:0] = 7'b0110011;
inst_field[14:12] = 3'b000;
inst_field[30] = 1;
#20;
// addi
inst_field[6:0] = 7'b0010011;
inst_field[14:12] = 3'b000;
inst_field[30] = 0;
// slli
inst_field[6:0] = 7'b0010011;
inst_field[14:12] = 3'b001;
inst_field[30] = 0;
// jalr
inst_field[6:0]= 7'b1100111;
inst_field[14:12] = 3'b000;
#20;
// lw
inst_field[6:0] = 7'b0000011;
inst_field[14:12] = 3'b010;
#20;
// sw
inst_field[6:0] = 7'b0100011;
inst_field[14:12] = 3'b010;
#20
// beq
inst_field[6:0] = 7'b1100011;
inst_field[14:12] = 3'b000;
#20;
// bne
inst_field[6:0] = 7'b1100011;
inst_field[14:12] = 3'b001;
#20;
// lui
inst_field[6:0] = 7'b0110111;
#20;
// jal
inst_field[6:0] = 7'b1101111;
#20; 
end


endmodule