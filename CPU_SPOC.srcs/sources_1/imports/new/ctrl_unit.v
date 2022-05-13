`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/03/28 19:12:43
// Design Name: 
// Module Name: ctrl
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

module SCPU_ctrl_more( 
input [31:0] inst_field,
//input[6:0]OPcode, //OPcode
//input[2:0]Fun3, //Function
//input Fun7, //Function
input MIO_ready, //CPU Wait
input wire restore,
output reg ecall,
output reg ill_instr,
output reg mret,
output reg [2:0]ImmSel,
output reg ALUSrc_B,
output reg [1:0]MemtoReg,
output reg [1:0]Jump,
output reg Branch,
output reg BranchN,
output reg RegWrite,
output reg MemRW,
output reg [3:0]ALU_Control,
output reg CPU_MIO
); 
parameter _add = 4'b0010 , _sub = 4'b0110, _sll = 4'b1110, _slt = 4'b0111 , _sltu = 4'b1001, _xor = 4'b1100,
_srl = 4'b1101, _sra = 4'b1111, _or = 4'b0001, _and = 4'b0000, _I = 3'd0,_S = 3'd1, _B = 3'd2, _J = 3'd3, _U = 3'd4,
_Reg = 0,_Imm = 1,_Read = 0,_Write = 1,_ALU = 0,_Mem = 1,_PC = 2, _Imm_mem = 3;

wire [6:0] OPcode = inst_field[6:0];
wire [2:0] Fun3 = inst_field[14:12];
wire Fun7 = inst_field[30];

always @* begin
    if(restore==1)begin
        ecall <=0;
        ill_instr <= 0;
        mret <= 0;
        MemRW <= _Read;
    end
    else begin
    case(OPcode) 
    // r type signed 
       7'b0110011:begin
            ecall = 0;
            ill_instr = 0;
            mret = 0;
            Branch = 0;
            BranchN = 0;
            Jump = 0;
            ALUSrc_B = 0;
            MemRW = 0;
            RegWrite = 1;
            MemtoReg = 2'b0;
            case({Fun3,Fun7})
                4'b0000:ALU_Control = 4'b0010;
                4'b0001:ALU_Control = 4'b0110;
                4'b0010:ALU_Control = 4'b1110;
                4'b0100:ALU_Control = 4'b0111;
                4'b0110:ALU_Control = 4'b1001;
                4'b1000:ALU_Control = 4'b1100;
                4'b1010:ALU_Control = 4'b1101;
                4'b1011:ALU_Control = 4'b1111;
                4'b1100:ALU_Control = 4'b0001;
                4'b1110:ALU_Control = 4'b0000;
                default:ill_instr = 1;
            endcase
        end
        // r type mret
       7'b1110011:begin
            ill_instr = 0;
//            ecall = 1;
//            mret = 0;
            ecall = (inst_field[21]==1'b0)?1:0;
            mret = (inst_field[21]==1'b1)?1:0;
       end
        // i type  lw
       7'b0000011:begin
            ecall = 0;
            ill_instr = (Fun3==3'b010)?0:1;
             mret = 0;
            Branch = 0;
            BranchN = 0;
            Jump = 0;
            ImmSel = 0;
            ALUSrc_B = 1;
            ALU_Control =_add;
            MemRW = _Read;
            RegWrite = 1;
            MemtoReg = 1;
         end
        // i type addi slti ... imm
        7'b0010011:begin
             ecall = 0;
             ill_instr = 0;
             mret = 0;
            Branch = 0;
            BranchN = 0;
            Jump = 0;
            ImmSel = 0;
            ALUSrc_B = 1;
            MemRW = _Read;
            RegWrite = 1;
            MemtoReg = _ALU;
            case(Fun3)
                3'b000:ALU_Control = _add;
                3'b001:ALU_Control = _sll;
                3'b010:ALU_Control = _slt;
                3'b011:ALU_Control = _sltu;
                3'b100:ALU_Control = _xor;
                3'b101:ALU_Control = _srl | ({2'b0,Fun7,1'b0});
                3'b110:ALU_Control = _or;
                3'b111:ALU_Control = _and; 
            endcase
        end
        // i type jalr
        7'b1100111:begin
            ecall = 0;
            ill_instr = 0;
            mret = 0;
            Jump = 2;
            ImmSel = _I;
            ALUSrc_B = _Imm;
            ALU_Control = _add;
            RegWrite = 1;
            MemtoReg = _PC;
        end
        // itype ecall mret ebreak
        7'b1110011:begin
            ill_instr = 0;
//            ecall = 1;
//            mret = 0;
            ecall = (inst_field[21]==1'b0)?1:0;
            mret = (inst_field[21]==1'b1)?1:0;
        end
         // s type sw
       7'b0100011:begin
            ecall = 0;
            ill_instr = (Fun3==3'b010)?0:1;
             mret = 0;
           Branch = 0;
           BranchN = 0;
           Jump = 0;
           ImmSel = _S;
           ALUSrc_B = _Imm;
           ALU_Control = _add;
           MemRW = _Write;
           RegWrite = 0; 
        end
           // sb type 
       7'b1100011:begin
            ecall = 0;
             mret = 0;
           case(Fun3)
                3'b000:begin // beq
                    ill_instr = 0;
                    Branch = 1;
                    Jump = 0;
                    ImmSel = _B;
                    ALUSrc_B = _Reg;
                    ALU_Control = _sub;
                    RegWrite = 0;
                    MemRW = _Read;
                end
                3'b001:begin // bne
                    ill_instr = 0;
                    BranchN = 1;
                    Jump = 0;
                    ImmSel = _B;
                    ALUSrc_B = _Reg;
                    ALU_Control = _sub;
                    RegWrite = 0;    
                    MemRW = _Read;
                end
                default:begin
                    ill_instr = 1;
                    BranchN = 0;
                    Jump = 0;
                    ImmSel = 0;
                    ALUSrc_B = 0;
                    ALU_Control = 0;
                    RegWrite = 0;
                    MemRW = _Read;
                end
           endcase
       end  
        // uj type jal
       7'b1101111:begin
                    ecall = 0;
                    ill_instr = 0;
                    mret = 0;
                    Jump = 1;
                    ImmSel = _J;
                    RegWrite = 1;
                    MemtoReg = _PC;
               end
        // u type
        7'b0110111:begin
            ecall = 0;
            ill_instr = 0;
            mret = 0;
            Branch = 0;
            BranchN = 0;
            Jump = 0;
            ImmSel = _U;
            RegWrite = 1;
            MemtoReg = _Imm_mem;
            MemRW = _Read;
        end
        default:begin
            ecall = 0;
            ill_instr = 1;
            mret = 0;
            Branch = 0;
            BranchN = 0;
            Jump = 0;
            ImmSel = 0;
            RegWrite = 0;
            MemtoReg = _Imm_mem;
            MemRW = _Read;
        end 
    endcase
    end
end




endmodule
