`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/05/12 23:47:50
// Design Name: 
// Module Name: Mux1023to32
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


module Mux1023to32(
input wire [1023:0] in,
input wire [4:0] sel,
output reg [31:0] out
    );
    
always @* begin
case (sel)
5'd31:out<=in[31:0];
5'd30:out<=in[63:32];
5'd29:out<=in[95:64];
5'd28:out<=in[127:96];
5'd27:out<=in[159:128];
5'd26:out<=in[191:160];
5'd25:out<=in[223:192];
5'd24:out<=in[255:224];
5'd23:out<=in[287:256];
5'd22:out<=in[319:288];
5'd21:out<=in[351:320];
5'd20:out<=in[383:352];
5'd19:out<=in[415:384];
5'd18:out<=in[447:416];
5'd17:out<=in[479:448];
5'd16:out<=in[511:480];
5'd15:out<=in[543:512];
5'd14:out<=in[575:544];
5'd13:out<=in[607:576];
5'd12:out<=in[639:608];
5'd11:out<=in[671:640];
5'd10:out<=in[703:672];
5'd9:out<=in[735:704];
5'd8:out<=in[767:736];
5'd7:out<=in[799:768];
5'd6:out<=in[831:800];
5'd5:out<=in[863:832];
5'd4:out<=in[895:864];
5'd3:out<=in[927:896];
5'd2:out<=in[959:928];
5'd1:out<=in[991:960];
5'd0:out<=in[1023:992];

endcase
end
endmodule
