# Lab04 单周期CPU（含指令拓展）

***

##### 姓名：张鑫

##### 学号：3200102809

## 1 实验目标

***

+ 设计单周期CPU数据通路
+ 设计单周期CPU控制单元
+ 将数据通路和控制单元组成SCPU模块，置于实验二的CPU测试环境中测试
+ 在原有基础上进行指令拓展

## 2 实验原理

***

### 2.1 顶层模块

![image-20220512222254782](C:\Users\86173\AppData\Roaming\Typora\typora-user-images\image-20220512222254782.png)

顶层模块原理图，SCPU主要由数据通路(Datapath)和控制单元(Controller)组成。

其中,datapath进行了CPU的取指令(IF),计算(EX),访存(MEM),写回(RB)。

事实上，CPU指令执行的硬件绝大多数位于Datapath中，而Controller则进行译码过程(ID),将信号传入Datapath, 使得Datapath中并行存在的逻辑电路能够组合正确的执行指令的功能。

另外，单周期CPU在MEM阶段访问的RAM与本实验的SCPU是分离的，本实验中只需要将MemRW信号传出，然后将RAM的读取数据传入SCPU即可。SCPU也不包含ROM，也就是说，取指令阶段，SCPU只传出PC值，由ROM读取返回指令值。

本实验中认为单周期可以完成RAM的寻址访存，故而MIO_ready信号在本实验中不需要利用，事实上，本实验的RAM大小只有1024$\times$32 bit, 内存深度小，寻址快，当内存深度增大时，可能会取决于内存深度花费数个时钟周期完成寻址访存，因而MIO_ready信号进行CPU和内存的同步是重要的，事实上，为了CPU等待内存访问所额外花费的大量时钟周期，完整的CPU需要有缓存(Cache)设计，缓存是CPU能够快速访问的较小的存储器(SRAM)，此处不展开。

### 2.2 Datapath

Datapath是SCPU的核心组成部分，其原理图如下：

![image-20220512224046064](C:\Users\86173\AppData\Roaming\Typora\typora-user-images\image-20220512224046064.png)

上图为带指令拓展的原理图。

其中:

+ PC: 32bit 寄存器，在时钟上升沿更新，存储当前指令在ROM中的地址
+ register file: 32 $\times$ 32 bit寄存器组，在实验一中实现
+ Mux: 在数字逻辑中实现，略
+ ALU: 算术运算单元，支持加减与或，异或，比较，算术\逻辑移位运算
+ ROM,RAM不属于SCPU内模块，前者包含指令，后者为内存(数据)
+ ImmGen: 根据五种指令生成拓展立即数

### 2.3 Controller

事实上控制器就是译码器，参照datapath中的硬件逻辑，controller需要对应的生成PC更新选择信号，立即数选择信号，branch，branchN信号，jump信号，register file写入信号，alu输入源选择信号，alu控制信号，mem读写信号，register file写入源选择信号。

由于一条指令的信号首先取决于其opcode，其次取决于其function code，为了方便代码编写，本实验采用二级解码方式，其优点是可读性强，占用硬件资源较少，可拓展性强，缺点是增加了电路延时，若CPU时钟周期变快，可能无法正确解码，因而本实验更应当使用一级解码。

控制信号表:

![image-20220512230528732](C:\Users\86173\AppData\Roaming\Typora\typora-user-images\image-20220512230528732.png)

## 3 实验过程

***

### 3.1 代码实现

开发过程使用VSCODE编辑, 利用VSCODE的自动补全功能较少无谓的debug时间。

本次实验直接按照指令拓展标准完成。

顶层模块连接如下(int信号为4-4CPU中断时使用)：

代码见附录。

Datapath实现

除了register file,mux使用了实验一的模块外，其他模块均重新设计。

为了拓展指令，重新设计了4位宽控制信号的alu以支持更多算术操作，为方便起见，alu使用代码设计

其余部分按照原理图连接。（其中包含4-4CPU中断代码）

Immgen: 代码见附录
immgen需要三位宽的信号，支持五种拓展，对应五种指令类型。

CPU controller:
按照控制信号表二级译码,代码见附录。

### 3.2 仿真

由于Datapath结构较为简单，大多数模块为现成的可调用模块，故而Datapath未进行仿真，只对controller进行了仿真。

分别依次测试了R,I,S,SB,U,UJ六种类型的指令。

仿真代码如下：

```verilog
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
```

仿真结果如图，认为基本符合预期，下一步进行物理验证

![image-20220513143717676](C:\Users\86173\AppData\Roaming\Typora\typora-user-images\image-20220513143717676.png)

### 3.3 物理验证

图一，物理验证效果。

为了方便调试和测试，我将rigister file中的寄存器值连入VGA中，故而可以通过查看VGA显示界面的寄存器值判断CPU是否工作正常。

使用4-3拓展指令提供的测试指令，转换成COE导入ROM后运行如图所示，单步运行结果与注释一致

![image-20220513140901060](C:\Users\86173\AppData\Roaming\Typora\typora-user-images\image-20220513140901060.png)

测试指令如下：

```verilog
PC Machine Code Basic Code Original Code Result
0x0 0x00007293 andi x5 x0 0 main:andi x5, x0, 0x0 # x5 = 0
0x4 0x00007313 andi x6 x0 0 andi x6, x0, 0x0 # x6 = 0
0x8 0x88888137 lui x2 559240 lui x2, 0x88888 # x2 = 0x88888000
0xc 0x00832183 lw x3 8(x6) lw x3, 0x8(x6) # x3 = 0x80000000
0x10 0x0032A223 sw x3 4(x5) sw x3, 0x4(x5) # mem（1）= 0x80000000
0x14 0x00402083 lw x1 4(x0) lw x1, 0x4(x0) # x1 = 0x80000000
0x18 0x01C02383 lw x7 28(x0) nochange:lw x7, 0x1c(x0) #x7 = 0x80000000
0x1c 0x00338863 beq x7 x3 16 beq x7, x3, cmd_add
0x20 0x555550B7 lui x1 349525 lui x1,0x55555 # x1 = 0x55555000
0x24 0x0070A0B3 slt x1 x1 x7 slt x1,x1,x7 # x1 = 0x1;
0x28 0xFE0098E3 bne x1 x0 -16 bne x1,x0,nochange
0x2c 0x007282B3 add x5 x5 x7 cmd_add:add x5, x5, x7 # x5 = 0x80000000
0x30 0x00230333 add x6 x6 x2 add x6, x6, x2 # x6 = 0x88888000
0x34 0x00531463 bne x6 x5 8 bne x6, x5, cmd_sub
0x38 0x40000033 sub x0 x0 x0 sub x0,x0,x0
0x3c 0x40530433 sub x8 x6 x5 cmd_sub: sub x8,x6,x5 # x8 = 0x08888000
0x40 0x405304B3 sub x9 x6 x5 sub x9,x6,x5 #x9 = 0x08888000
0x44 0x0080006F jal x0 8 jal x0, cmd_and
0x48 0x00007033 and x0 x0 x0 and x0,x0,x0
0x4c 0x0072F533 and x10 x5 x7 cmd_and: and x10,x5,x7 #x10 = 0x80000000
0x50 0x00157593 andi x11 x10 1 andi x11,x10,0x1 # x11 = 0x0
0x54 0x00B51463 bne x10 x11 8 bne x10,x11 cmd_or
0x58 0x00006033 or x0 x0 x0 or x0,x0,x0
0x5c 0x00A5E5B3 or x11 x11 x10 cmd_or:or x11,x11,x10 # x11 = 0x80000000
0x60 0x0015E513 ori x10 x11 1 ori x10,x11,0x1 # x10 = 0x80000001
0x64 0x00558463 beq x11 x5 8 beq x11,x5,cmd_xor
0x68 0x00004033 xor x0 x0 x0 xor x0,x0,x0
0x6c 0x00A5C633 xor x12 x11 x10 cmd_xor: xor x12,x11,x10 # x12 = 0x00000001
0x70 0x00164613 xori x12 x12 1 xori x12,x12,0x1 # x12 = 0x00000000
0x74 0x00B61463 bne x12 x11 8 bne x12,x11,cmd_srl
0x78 0x00000013 addi x0 x0 0 addi x0,x0,0x0
0x7c 0x0012D293 srli x5 x5 1 cmd_srl: srli x5,x5,0x1 # x5 = 0x40000000
0x80 0x00060463 beq x12 x0 8 beq x12,x0,cmd_sll
0x84 0x40000033 sub x0 x0 x0 sub x0,x0,x0
0x88 0x00129293 slli x5 x5 1 cmd_sll: slli x5,x5,0x1 # x5 = 0x80000000
0x8c 0x00B28463 beq x5 x11 8 beq x5,x11,cmd_slt
0x90 0x00000013 addi x0 x0 0 addi x0,x0,0x0
0x94 0x001026B3 slt x13 x0 x1 cmd_slt: slt x13,x0,x1 # x13 = 0x0
0x98 0x00503733 sltu x14 x0 x5 sltu x14,x0,x5 # x14 = 0x1
0x9c 0xF65FF06F jal x0 -156 jal x0,main
```



## 4 实验感想

本实验为本学期计组课程第一个大型实验，给了四周的时间，可以说时间十分充分了。

单周期CPU的原理在理论课上已经理解了，实验设计上也就十分顺畅，除了译码器为了编写方便采取了有缺陷的设计之外，SCPU设计上没有什么瓶颈。

说说我都在什么地方踩坑了吧。

1. VGA . 由于实验二中的CPU测试环境中的VGA连线不完全，实际上只显示了ALU_RES，PC和INST_FIELD的值。

   于是我就自行修改vga部分的verilog代码，将register file的三十二个寄存器值引入VGA，造成了物理验证时无法显示，VGA一片黑，经排查，是VGA模块中的vga显示所用到的字体等文件的路径十分严格

   + 不能有中文
   + 必须为绝对路径
   + 路径中必须使用//

   修改后，VGA终于正常显示。

2. ImmGen. 注意，ImmGen在SB,UJ型指令中立即数是低位补充一个0的，如果忘记，会出现PC不对齐的错误情况。

这次实验做完还是蛮有成就感的，毕竟是第一次自己写出一个CPU，尽管它的性能不值一提。



## 5 附录

SCPU：

```verilog
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
    // wire MemRW;
    wire CPU_MIO;
    wire mret,ecall,ill_instr,INT;
    reg [1:0] check_int = 2'b0;
    always@(posedge clk)begin
        check_int <= {check_int[0],out_io};
    end
    assign INT = (check_int == 2'b01);
    SCPU_ctrl_more ctrl_unit (.inst_field(inst_in),.MIO_ready(MIO_ready)
    ,.ImmSel(ImmSel),.ALUSrc_B(ALUSrc_B),.MemtoReg(MemtoReg),.Jump(Jump),.Branch(Branch),.BranchN(BranchN),.RegWrite(RegWrite),
    .MemRW(MemRW),.ALU_Control(ALU_Control),.CPU_MIO(CPU_MIO),.mret(mret),.ecall(ecall),.ill_instr(ill_instr));
    Data_path_more datapath_unit (.clk(clk),.rst(rst),.inst_field(inst_in),.ALUSrc_B(ALUSrc_B),.MemtoReg(MemtoReg),.Jump(Jump),.Branch(Branch),
    .BranchN(BranchN),.RegWrite(RegWrite),.Data_in(Data_in),.ALU_Control(ALU_Control),.ImmSel(ImmSel),.ALU_out(ALU_out),
    .Data_out(Data_out),.PC_out(PC_out),.Reg_value(Reg_value),.ecall(ecall),.ill_instr(ill_instr),
    .mret(mret),.INT(INT),.mepc(mepc));

endmodule
```



ALU:
```verilog
module ALU(
    input wire [3:0] S,
    input wire [31:0] A,
    input wire [31:0] B,
    output reg [31:0] C,
    output wire ZERO
    );
    integer i;
    assign ZERO = ~(|C);
    always@(*)begin
        case(S)
            4'b0000:C<=A&B; // and
            4'b0001:C<=A|B;// sub
            4'b1111:C <= ($signed(A)) >>> B[4:0]; // sra
            4'b1001:C <= ($unsigned(A)) < ($unsigned(B)); // sltu
            4'b1110:C <= A << B[4:0]; // sll
            4'b0010:C<=A+B; // add
            4'b1100:C<=A^B; // xor
            4'b0100:C<=~(A|B); // nor
            4'b1101:C <= A >> B[4:0]; // srl
            4'b0110:C<=A-B; // sub
            4'b0111:C<=($signed(A)) < ($signed(B)); //slt
        endcase
    end

endmodule
```
Datapath:

```verilog
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/03/28 16:22:57
// Design Name: 
// Module Name: DataPath
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

module Data_path_more
( 
    input clk, //寄存器时钟
    input rst, //寄存器复位
    input[31:0]inst_field, //指令数据域[31:7]    
    input ALUSrc_B, //ALU端口B输入选择
    input [1:0]MemtoReg, //Regs写入数据源控制
    input [1:0]Jump, //J指令
    input Branch, //Beq指令
    input BranchN, //Bne指令
    input RegWrite, //寄存器写信号
    input[31:0]Data_in, //存储器输入
    input[3:0]ALU_Control, //ALU操作控制
    input[2:0]ImmSel, //ImmGen操作控制
    input INT, // 中断信号
    input ecall, // 软中断信号
    input mret, // 中断返回信号
    input ill_instr, // 非法指令信号
    output[31:0]ALU_out, //ALU运算输出
    output[31:0]Data_out, //CPU数据输出
    output[31:0]PC_out, //PC指针输出
    output wire [1023:0] Reg_value,
    output reg [31:0] mepc,
    output reg state
);
    parameter _supervisor = 1'b1, _user = 1'b0;
    parameter _normal_ = 1'b0,_restore_ = 1'b1;
//    reg state;
    // register file output line
    wire [31:0] rs1_data,rs2_data;
    // data to be write 
    wire [31:0] register_in;
    // 32 * 32 register file
    reg [1023:0] dummy_stack;
    RegFile cpu_regfile (.clk(clk),.rst(rst),.restore(state),.restore_data(dummy_stack),.RegWrite(RegWrite),.Rs1_addr(inst_field[19:15]),
    .Rs2_addr(inst_field[24:20]),.Wt_addr(inst_field[11:7]),
    .Wt_data(state?restore_data:register_in),.Rs1_data(rs1_data),
    .Rs2_data(rs2_data),.Reg_value(Reg_value));
    // imm signed extension
    wire [31:0] imm;
    immGen cpu_imm (.ImmSel(ImmSel),.inst_field(inst_field),.Imm_out(imm));
    // imm or rigister data 2 to one mux
    wire [31:0] alu_op2 = (ALUSrc_B) ? (imm) : (rs2_data);
    // ALU
    wire zero;
    ALU cpu_alu (.S(ALU_Control),.A(rs1_data),.B(alu_op2),.C(ALU_out),.ZERO(zero));
    // pc
    reg [31:0] pc;
    assign PC_out = pc;
    wire [31:0] pc_in;
    wire [31:0] pc_increase = pc + 32'h4;
    // jal
    wire [31:0] pc_offset = pc + imm;
    // beq,bne
    wire [31:0] pc_check_beq = ((Branch&zero)|(BranchN&(~zero))) ? (pc_offset) : pc_increase;
    // jump-mux
    Mux_2to1_32bit sel_jmp (.s(Jump),.in0(pc_check_beq),.in1(pc_offset),
    .in2(ALU_out),.in3(pc_check_beq),.out(pc_in));
    // pc update
    // hundle interrupt
    wire [3:0] exc = {INT,ecall,mret,ill_instr};
//    reg mode = _user;
//    RV_Int cpu_pc (
//        .clk(clk),.reset(rst),.INT(INT),.ecall(ecall),.mepc(mepc),
//        .mret(mret),.ill_instr(ill_instr),.pc_next(pc_in),.pc(pc_int)
//    );
    
    always@(posedge clk or posedge rst)begin
        if(rst==1'b1)begin
            pc <= 0;
            mepc <= 0;
            dummy_stack <= 0;
            state <= _normal_;
        end
        else begin
        if(state==_normal_)begin
        case(exc)
        4'b0001:begin 
            dummy_stack <= Reg_value;
            mepc <= pc + 32'h4 ;
            pc <= 32'h4;
            
        end
        4'b0010:begin
            state <= _restore_;
            pc <= mepc;
        end
        4'b0100:begin
            dummy_stack <= Reg_value;
            mepc <= pc + 32'h4 ;
            pc <= 32'h8;
        end
        4'b1000:begin
            dummy_stack <= Reg_value;
            mepc <= pc;
            pc <= 32'hC;
        end
        default:begin
            pc <= pc_in;
        end
        endcase
    end else begin
        // restore
        state <= _normal_;
    end
    end
    end
    // control write data to the register file
    Mux_2to1_32bit sel_reg_in (.s(MemtoReg),.in0(ALU_out),.in1(Data_in),
    .in2(pc_increase),.in3(imm),.out(register_in));
    // data to be write to memory
    assign Data_out = rs2_data;
endmodule

```

ImmGen:

```verilog
module immGen(
    input wire [2:0] ImmSel,
    input wire [31:0] inst_field,
    output reg [31:0]   Imm_out
    );
    
    always @* begin
        case(ImmSel)
            // i
            3'b000:Imm_out = {{20{inst_field[31]}},inst_field[31:20]};
            // s
            3'b001:Imm_out = {{20{inst_field[31]}},inst_field[31:25],inst_field[11:7]};
            // sb
            3'b010:Imm_out = {{29{inst_field[31]}},inst_field[31],inst_field[7],inst_field[30:25],inst_field[11:8],1'b0};
            // uj
            3'b011:Imm_out = {{11{inst_field[31]}},inst_field[31],inst_field[19:12],inst_field[20],inst_field[30:21],1'b0};
            // u
            3'b100:Imm_out = {inst_field[31:12],12'h0};
            default: Imm_out = 31'h0;
        endcase
    end
endmodule

```


CPU controller:

```verilog
module SCPU_ctrl_more( 
input [31:0] inst_field,
//input[6:0]OPcode, //OPcode
//input[2:0]Fun3, //Function
//input Fun7, //Function
input MIO_ready, //CPU Wait
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




endmodule
```

