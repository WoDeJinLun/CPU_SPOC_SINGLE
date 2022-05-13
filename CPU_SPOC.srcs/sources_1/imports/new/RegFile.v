`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/03/28 16:35:08
// Design Name: 
// Module Name: RegFile
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

module RegFile(
    input wire clk,rst,RegWrite,restore,
    input wire [4:0] Rs1_addr,Rs2_addr,Wt_addr,
    input wire [31:0] Wt_data,
    input wire [1023:0] restore_data,
    output  wire [31:0] Rs1_data,Rs2_data,
    output wire [1023:0] Reg_value
    );

    reg [31:0] register[1:31];
    integer i;
    assign Rs1_data = (Rs1_addr==0)?0:register[Rs1_addr];
    assign Rs2_data = (Rs2_addr==0)?0:register[Rs2_addr];
    
    always@(posedge clk or posedge rst)begin
        if(rst==1)begin
            for(i=1;i<32;i=i+1)begin
                register[i] <= 0;
            end
        end else if(restore==1)begin
        register[1]<=restore_data[991:960];
        register[2]<=restore_data[959:928];
        register[3]<=restore_data[927:896];
        register[4]<=restore_data[895:864];
        register[5]<=restore_data[863:832];
        register[6]<=restore_data[831:800];
        register[7]<=restore_data[799:768];
        register[8]<=restore_data[767:736];
        register[9]<=restore_data[735:704];
        register[10]<=restore_data[703:672];
        register[11]<=restore_data[671:640];
        register[12]<=restore_data[639:608];
        register[13]<=restore_data[607:576];
        register[14]<=restore_data[575:544];
        register[15]<=restore_data[543:512];
        register[16]<=restore_data[511:480];
        register[17]<=restore_data[479:448];
        register[18]<=restore_data[447:416];
        register[19]<=restore_data[415:384];
        register[20]<=restore_data[383:352];
        register[21]<=restore_data[351:320];
        register[22]<=restore_data[319:288];
        register[23]<=restore_data[287:256];
        register[24]<=restore_data[255:224];
        register[25]<=restore_data[223:192];
        register[26]<=restore_data[191:160];
        register[27]<=restore_data[159:128];
        register[28]<=restore_data[127:96];
        register[29]<=restore_data[95:64];
        register[30]<=restore_data[63:32];
        register[31]<=restore_data[31:0];
        end
        else if((Wt_addr!=0)&&(RegWrite==1))begin
            register[Wt_addr]=Wt_data;        
        end
    end
    
    // wire to vga
    assign Reg_value[1023:992] = 32'b0;
    assign Reg_value[991:960] = register[1];
    assign Reg_value[959:928] = register[2];
    assign Reg_value[927:896] = register[3];
    assign Reg_value[895:864] = register[4];
    assign Reg_value[863:832] = register[5];
    assign Reg_value[831:800] = register[6];
    assign Reg_value[799:768] = register[7];
    assign Reg_value[767:736] = register[8];
    assign Reg_value[735:704] = register[9];
    assign Reg_value[703:672] = register[10];
    assign Reg_value[671:640] = register[11];
    assign Reg_value[639:608] = register[12];
    assign Reg_value[607:576] = register[13];
    assign Reg_value[575:544] = register[14];
    assign Reg_value[543:512] = register[15];
    assign Reg_value[511:480] = register[16];
    assign Reg_value[479:448] = register[17];
    assign Reg_value[447:416] = register[18];
    assign Reg_value[415:384] = register[19];
    assign Reg_value[383:352] = register[20];
    assign Reg_value[351:320] = register[21];
    assign Reg_value[319:288] = register[22];
    assign Reg_value[287:256] = register[23];
    assign Reg_value[255:224] = register[24];
    assign Reg_value[223:192] = register[25];
    assign Reg_value[191:160] = register[26];
    assign Reg_value[159:128] = register[27];
    assign Reg_value[127:96] = register[28];
    assign Reg_value[95:64] = register[29];
    assign Reg_value[63:32] = register[30];
    assign Reg_value[31:0] = register[31];

endmodule
