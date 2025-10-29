// Template for Northwestern - CompEng 361 - Lab3 -- Version 1.1
// Groupname:
// NetIDs:

// Some useful defines...please add your own
`define WORD_WIDTH 32
`define NUM_REGS 32
`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_STORE      7'b0100011 
`define FUNC_ADD      3'b000
`define AUX_FUNC_ADD  7'b0000000
`define AUX_FUNC_SUB  7'b0100000
`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10

`define IOPCODE 7'b0010011
`define LOPCODE 7'b0000011
`define SOPCODE 7'b0100011

module SingleCycleCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [`WORD_WIDTH-1:0] PC, InstWord; //pc is passed into MEM and InstWord is returned 
   wire [`WORD_WIDTH-1:0] DataAddr, StoreData, DataWord; //data ADDR is the address passed int to mem, 
   wire [1:0]  MemSize;  //00 is byte, 01 is half word, 10 is word
   wire        MemWrEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst;  //register address 
   wire [`WORD_WIDTH-1:0] Rdata1, Rdata2, RWrdata; //register data out and inout for write register
   wire        RWrEn;

   wire [`WORD_WIDTH-1:0] NPC, PC_Plus_4;
   wire [6:0]  opcode;

   wire [6:0]  funct7;
   wire [2:0]  funct3;

   wire [`WORD_WIDTH-1:0] I_OUT;
    wire [`WORD_WIDTH-1:0] S_OUT;

   wire invalid_op;
   
   // Only support R-TYPE ADD and SUB
   assign halt = invalid_op;
   assign invalid_op = !((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_ADD) &&
		      ((funct7 == `AUX_FUNC_ADD) || (funct7 == `AUX_FUNC_SUB)));
     
   // System State 
   Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
            .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
	      .AddrB(Rsrc2), .DataOutB(Rdata2), 
	      .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   Reg PC_REG(.Din(NPC), .Qout(PC), .WE(1'b1), .CLK(clk), .RST(rst));  //instead of PC in register file we are instantiateing one register at a time

   // Instruction Decode (since its continuouse assignment, all this is populated when MEM outputs .InstOut)
   assign opcode = InstWord[6:0];   
   assign Rdst = InstWord[11:7]; 
   assign Rsrc1 = InstWord[19:15]; 
   assign Rsrc2 = InstWord[24:20];
   assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
   assign funct7 = InstWord[31:25];  // R-Type

   assign MemWrEn = 1'b0; // Change this to allow stores
   assign RWrEn = 1'b1;  // At the moment every instruction will write to the register file

   // Hardwired to support R-Type instructions -- please add muxes and other control signals (RWrdata is the data writted to reg file)
   R_ExecutionUnit EU(.out(RWrdata), .opA(Rdata1), .opB(Rdata2), .func(funct3), .auxFunc(funct7));

   DataAddr = (opcode == LOPCODE) ? I_OUT : (opcode == SOPCODE) ? S_OUT : 32'b0;

   DataIn = (opcode == SOPCODE) ? Rdata2 : 32'b0;

   WE = (opcode == SOPCODE) ? 1'b1 : 0;  

   RWrdata = (opcode == LOPCODE && funct3 == 3'b000) ? {{24{DataWord[7]}}, (DataWord[7:0])} :
            (opcode == LOPCODE && funct3 == 3'b001) ? {{16{DataWord[15]}}, (DataWord[15:0])} 

   MemSize = (opcode == LOPCODE) ? funct3[1:0] : 0;

   DataWord = 
             


   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
   assign NPC = PC_Plus_4;
   
endmodule // SingleCycleCPU




// Incomplete version of Lab2 execution unit
// You will need to extend it. Feel free to modify the interface also
module R_ExecutionUnit(out, opA, opB, func, auxFunc);
   output [`WORD_WIDTH-1:0] out; //out is the data that is gettting fed into the register file
   input [`WORD_WIDTH-1:0]  opA, opB; //opA = data out of regA, opB = data out of regB
   input [2:0] 	 func;
   input [6:0] 	 auxFunc;

   wire [`WORD_WIDTH-1:0] 	 addSub;

   // Only supports add and subtract
   // assign addSub = (auxFunc == 7'b0100000) ? (opA - opB) : (opA + opB);
   // assign out = (func == 3'b000) ? addSub : 32'hXXXXXXXX;

   assign out = (func == 3'b000 && auxFunc == 7'b0000000) ? opA + opB  :  //add
                (func == 3'b000 && auxFunc == 7'b0110000) ? opA - opB  :  //sub
                (func == 3'b001 && auxFunc == 7'b0000000) ? opA << (opB & 5'b11111) :  //sll
                (func == 3'b010 && auxFunc == 7'b0000000) ? $signed(opA) < $signed(opB) : //slt
                (func == 3'b011 && auxFunc == 7'b0000000) ? $unsigned(opA) < $unsigned(opB) : //sltu
                (func == 3'b100 && auxFunc == 7'b0000000) ? opA ^ opB : //xor
                (func == 3'b101 && auxFunc == 7'b0100000) ? $signed(opA) >>> (opB & 5'b11111) :  //sra
                (func == 3'b110 && auxFunc == 7'b0000000) ? opA | opB :
                (func == 3'b111 && auxFunc == 7'b0000000) ? opA & opB :
                32'b0;
                

   
endmodule // R_ExecutionUnit


module I_ExecutionUnit (
   output [`WORD_WIDTH-1:0] out,
   input  [`WORD_WIDTH-1:0]  opA,
   input  [11:0] imm12,
   input   [2:0] func,
   input [6:0] opcode,

);

   //need to checkoutpot

   // module Mem(InstAddr, InstOut,
   //       DataAddr, DataSize, DataIn, DataOut, WE, CLK);
   //  input [31:0] InstAddr, DataAddr;
   //  input [1:0] 	DataSize;   
   //  input [31:0] DataIn;   
   //  output [31:0] InstOut, DataOut;  
   //  input      WE, CLK;

   wire [`WORD_WIDTH-1:0] memOut;

  assign out = (opcode == IOPCODE && func == 3'b000) ? opA + {{20{imm12[11]}}, (imm12)} : //addi
       (opcode == IOPCODE && func == 3'b010)  ?  opA < {{20{imm12[11]}}, (imm12)} :         //slti
       (opcode == LOPCODE && func == 3'b000) ? opA + {{20{imm12[11]}}, (imm12)} // LB :


//



endmodule

module S_ExecutionUnit (
   output [`WORD_WIDTH-1:0] out,
   input  [`WORD_WIDTH-1:0]  opA, opB
   input  [7:0] imm12_U,
   input  [4:0] imm12_L,
   input   [2:0] func,
   input [6:0] opcode,

);

   
   wire [11:0] imm12 = {imm12_U, imm12_L};

     assign out = (opcode == SOPCODE && func == 3'b000) ? opA + {{20{imm12[11]}}, (imm12)} // LB :





endmodule


