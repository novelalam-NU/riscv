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

`define ROPCODE 7'b0110011

`define IOPCODE_i 7'b0010011
`define IOPCODE_ld 7'b0000011
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
   wire [`WORD_WIDTH-1:0] R_OUT; //out from R execution unit

   wire invalid_op;
   
   // Only support R-TYPE ADD and SUB
   assign halt = invalid_op;
   //assign invalid_op = 
     
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

   // Hardwired to support R-Type instructions -- please add muxes and other control signals (RWrdata is the data writted to reg file)
   R_ExecutionUnit R_EU(.out(R_OUT), .opA(Rdata1), .opB(Rdata2), .func(funct3), .auxFunc(funct7));

   I_ExecutionUnit I_EU(.out(I_OUT), .opA(Rdata1), .imm12(InstWord[31:20]), .func(funct3), .opcode(opcode) );

   S_ExecutionUnit S_EU(.out(S_OUT), .opA(Rdata1), .imm12_U(InstWord[31:25]), .imm12_L(InstWord[11:7]) , .func(funct3), .opcode(opcode) );

   assign StoreData = (opcode == `SOPCODE) ? Rdata2 : 0;

   assign DataAddr = (opcode == `IOPCODE_ld) ? I_OUT : (opcode == `SOPCODE) ? S_OUT : 32'b0;

   // DataIn = (opcode == SOPCODE) ? Rdata2 : 32'b0;



   assign MemSize = (opcode == `IOPCODE_ld || opcode == `SOPCODE) ? 
            ((funct3 == 3'b000) ? `SIZE_BYTE :
            (funct3 == 3'b001) ? `SIZE_HWORD :
            (funct3 == 3'b010) ? `SIZE_WORD :
            `SIZE_WORD)  
            : 2'b00;       





   //for R type and Load writing to out to  register      
   assign MemWrEn = (opcode == `SOPCODE); // Change this to allow stores
   assign RWrEn = ( (opcode == `ROPCODE) || (opcode == `IOPCODE_ld) );  // only write to reg file if R or L type instruction 

   assign RWrdata = 
      (opcode == `ROPCODE) ? R_OUT :
      (opcode == `IOPCODE_ld && funct3 == 3'b000) ? {{24{DataWord[7]}},  DataWord[7:0]}  : // LB
      (opcode == `IOPCODE_ld && funct3 == 3'b001) ? {{16{DataWord[15]}}, DataWord[15:0]} : // LH
      (opcode == `IOPCODE_ld && funct3 == 3'b010) ? DataWord                              : // LW
      (opcode == `IOPCODE_ld && funct3 == 3'b100) ? {24'b0,  DataWord[7:0]}               : // LBU
      (opcode == `IOPCODE_ld && funct3 == 3'b101) ? {16'b0,  DataWord[15:0]}              : // LHU
      32'bz;


   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
   assign NPC = PC_Plus_4;
   
endmodule // SingleCycleCPU


//set the 





// Incomplete version of Lab2 execution unit
// You will need to extend it. Feel free to modify the interface also
module R_ExecutionUnit(out, opA, opB, func, auxFunc, opcode);
   output [`WORD_WIDTH-1:0] out; //out is the data that is gettting fed into the register file
   input [`WORD_WIDTH-1:0]  opA, opB; //opA = data out of regA, opB = data out of regB
   input [2:0] 	 func;
   input [6:0] 	 auxFunc;
   input [6:0]   opcode;

   wire [`WORD_WIDTH-1:0] 	 addSub;

   // Only supports add and subtract
   // assign addSub = (auxFunc == 7'b0100000) ? (opA - opB) : (opA + opB);
   // assign out = (func == 3'b000) ? addSub : 32'hXXXXXXXX;

   assign out = (opcode == `ROPCODE && func == 3'b000 && auxFunc == 7'b0000000) ? opA + opB  :  //add
                (opcode == `ROPCODE && func == 3'b000 && auxFunc == 7'b0100000) ? opA - opB  :  //sub
                (opcode == `ROPCODE && func == 3'b001 && auxFunc == 7'b0000000) ? opA << (opB & 5'b11111) :  //sll
                (opcode == `ROPCODE && func == 3'b010 && auxFunc == 7'b0000000) ? $signed(opA) < $signed(opB) : //slt
                (opcode == `ROPCODE && func == 3'b011 && auxFunc == 7'b0000000) ? $unsigned(opA) < $unsigned(opB) : //sltu
                (opcode == `ROPCODE && func == 3'b100 && auxFunc == 7'b0000000) ? opA ^ opB : //xor
                (opcode == `ROPCODE && func == 3'b101 && auxFunc == 7'b0100000) ? $signed(opA) >>> (opB & 5'b11111) :  //sra
                (opcode == `ROPCODE && func == 3'b110 && auxFunc == 7'b0000000) ? opA | opB :
                (opcode == `ROPCODE && func == 3'b111 && auxFunc == 7'b0000000) ? opA & opB :
                32'bz;
                

   
endmodule // R_ExecutionUnit


module I_ExecutionUnit (
   output [`WORD_WIDTH-1:0] out,
   input  [`WORD_WIDTH-1:0]  opA,
   input  [11:0] imm12,
   input   [2:0] func,
   input [6:0] opcode

);

   

assign out = 
    (opcode == `IOPCODE_i && func == 3'b000) ? opA + {{20{imm12[11]}}, imm12} :           // ADDI
    (opcode == `IOPCODE_i && func == 3'b010) ? ($signed(opA) < $signed({{20{imm12[11]}}, imm12})) : // SLTI
    (opcode == `IOPCODE_i && func == 3'b011) ? ($unsigned(opA) < $unsigned({{20{imm12[11]}}, imm12})) : // SLTIU
    (opcode == `IOPCODE_i && func == 3'b100) ? (opA ^ {{20{imm12[11]}}, imm12}) :         // XORI
    (opcode == `IOPCODE_i && func == 3'b110) ? (opA | {{20{imm12[11]}}, imm12}) :         // ORI
    (opcode == `IOPCODE_i && func == 3'b111) ? (opA & {{20{imm12[11]}}, imm12}) :         // ANDI
    (opcode == `IOPCODE_i && func == 3'b001) ? (opA << imm12[4:0]) :                      // SLLI
    (opcode == `IOPCODE_i && func == 3'b101 && imm12[11:5] == 7'b0000000) ? (opA >> imm12[4:0]) :  // SRLI
    (opcode == `IOPCODE_i && func == 3'b101 && imm12[11:5] == 7'b0100000) ? ($signed(opA) >>> imm12[4:0]) : // SRAI

    (opcode == `IOPCODE_ld && (func == 3'b000 || func == 3'b001 || func == 3'b010) ) ? opA + {{20{imm12[11]}}, imm12} :
    32'bz;


endmodule

module S_ExecutionUnit (
   output [`WORD_WIDTH-1:0] out,
   input  [`WORD_WIDTH-1:0]  opA,
   input  [6:0] imm12_U,
   input  [4:0] imm12_L,
   input   [2:0] func,
   input [6:0] opcode

);

   
   wire [11:0] imm12 = {imm12_U, imm12_L}; //load should go in i type 

   assign out = ( ((opcode == `SOPCODE) && (func == 3'b000 || func == 3'b001 || func == 3'b010)) ) ? opA + {{20{imm12[11]}}, imm12} : 32'bz;// SB, SH, SW         




endmodule


