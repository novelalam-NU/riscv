// Template for Northwestern - CompEng 361 - Lab3 -- Version 1.1
// Groupname: Park-Novel
// NetIDs: Novel: JGH4015 Park: nco1511

// Some useful defines...please add your own
`define WORD_WIDTH 32
`define NUM_REGS 32
`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_STORE      7'b0100011
`define OPCODE_JAL      7'b1101111
`define OPCODE_JALR      7'b1100111
`define OPCODE_LUI      7'b0110111
`define OPCODE_AUIPC      7'b0010111
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
   wire invalid_rtype= (opcode == `ROPCODE) &&
   !((( funct3 == 3'b000 && funct7 == 7'b0000000)  //add
                || (funct3 == 3'b000 && funct7 == 7'b0100000)   //sub
                || (funct3 == 3'b001 && funct7 == 7'b0000000)    //sll
                || (funct3 == 3'b010 && funct7 == 7'b0000000)    //slt
                || (funct3 == 3'b011 && funct7 == 7'b0000000)   //sltu
                || (funct3 == 3'b100 && funct7 == 7'b0000000)  //xor
                || (funct3 == 3'b101 && funct7 == 7'b0000000)  // srl
                || (funct3 == 3'b101 && funct7 == 7'b0100000)  //sra
                || (funct3 == 3'b110 && funct7 == 7'b0000000)   //or
                || (funct3 == 3'b111 && funct7 == 7'b0000000)  //and
                || (funct7 == 7'b0000001)
   ));
   wire invalid_itype = (opcode == 7'b0010011) &&
   !(
       (funct3 == 3'b000) || // ADDI
       (funct3 == 3'b010) || // SLTI
       (funct3 == 3'b011) || // SLTIU
       (funct3 == 3'b100) || // XORI
       (funct3 == 3'b110) || // ORI
       (funct3 == 3'b111) || // ANDI
       (funct3 == 3'b001 && InstWord[31:25] == 7'b0000000) || // SLLI
       (funct3 == 3'b101 && InstWord[31:25] == 7'b0000000) || // SRLI
       (funct3 == 3'b101 && InstWord[31:25] == 7'b0100000)    // SRAI
   );

   wire invalid_load =
   (opcode == 7'b0000011) &&
   !(
       (funct3 == 3'b000) || // LB
       (funct3 == 3'b001) || // LH
       (funct3 == 3'b010) || // LW
       (funct3 == 3'b100) || // LBU
       (funct3 == 3'b101)    // LHU
   );

   wire invalid_jalr =
   (opcode == 7'b1100111) && !(funct3 == 3'b000);


   wire misaligned_load =
   (opcode == 7'b0000011) &&
   ( (funct3 == 3'b001 && DataAddr[0]   != 1'b0) ||   // LH/LHU misalignement
     (funct3 == 3'b010 && DataAddr[1:0] != 2'b00) );  // LW misalignment
   
   wire invalid_store =
   (opcode == 7'b0100011) &&
   !(
       (funct3 == 3'b000) || // SB
       (funct3 == 3'b001) || // SH
       (funct3 == 3'b010)    // SW
   );

   wire misaligned_store = (opcode == `SOPCODE) &&
    ((funct3 == 3'b001 && S_OUT[0]      != 1'b0) || // SH misaligned
     (funct3 == 3'b010 && S_OUT[1:0]    != 2'b00)); // SW misaligned
   
   wire invalid_branch = (opcode == `OPCODE_BRANCH) &&
    !((funct3 == 3'b000) || // BEQ
      (funct3 == 3'b001) || // BNE
      (funct3 == 3'b100) || // BLT
      (funct3 == 3'b101) || // BGE
      (funct3 == 3'b110) || // BLTU
      (funct3 == 3'b111));  // BGEU

   wire fetch_misaligned = (PC[1:0] != 2'b00);



   // Only support R-TYPE ADD and SUB
   assign halt = invalid_op;
   assign invalid_op =  
    (InstWord !== InstWord) ? 1'b1 :
    (PC[1:0] != 2'b00) ? 1'b1 :
    (opcode == `OPCODE_COMPUTE) ? invalid_rtype :
    (opcode == `IOPCODE_i)      ? invalid_itype :
    (opcode == `IOPCODE_ld)     ? (invalid_load || misaligned_load)  :
    (opcode == `OPCODE_JALR)    ? invalid_jalr  :
    (opcode == `OPCODE_STORE)   ? (invalid_store || misaligned_store) :
    (opcode == `OPCODE_BRANCH)  ? invalid_branch :
    (opcode == `OPCODE_JAL)     ? 1'b0 :  
    (opcode == `OPCODE_LUI)     ? 1'b0 :
    (opcode == `OPCODE_AUIPC)   ? 1'b0 :
    (opcode == `OPCODE_LOAD)    ? (invalid_load || misaligned_load) :
    1'b1;


     
   // System State
   Mem   MEM(.InstAddr(PC), .InstOut(InstWord),
            .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1),
     .AddrB(Rsrc2), .DataOutB(Rdata2),
     .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   Reg PC_REG(.Din(NPC), .Qout(PC), .WE(!halt), .CLK(clk), .RST(rst));  //instead of PC in register file we are instantiateing one register at a time

   // Instruction Decode (since its continuouse assignment, all this is populated when MEM outputs .InstOut)
   assign opcode = InstWord[6:0];  
   assign Rdst = InstWord[11:7];
   assign Rsrc1 = InstWord[19:15];
   assign Rsrc2 = InstWord[24:20];
   assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
   assign funct7 = InstWord[31:25];  // R-Type



   // Hardwired to support R-Type instructions -- please add muxes and other control signals (RWrdata is the data writted to reg file)
   R_ExecutionUnit R_EU(.out(R_OUT), .opA(Rdata1), .opB(Rdata2), .func(funct3), .auxFunc(funct7), .opcode(opcode));

   I_ExecutionUnit I_EU(.out(I_OUT), .opA(Rdata1), .imm12(InstWord[31:20]), .func(funct3), .opcode(opcode) );

   S_ExecutionUnit S_EU(.out(S_OUT), .opA(Rdata1), .imm12_U(InstWord[31:25]), .imm12_L(InstWord[11:7]) , .func(funct3), .opcode(opcode) );

   // Branch unit
   wire [31:0] BR_NPC;
   wire BR_TAKE;
   Branch_ExecutionUnit BR_EU(.NPC_out(BR_NPC),.take_branch(BR_TAKE),.PC(PC),.Rdata1(Rdata1),.Rdata2(Rdata2),.InstWord(InstWord),.funct3(funct3),.opcode(opcode));

// Jump unit
   wire [31:0] JUMP_PC_TARGET, LINK_VALUE;
   Jump_ExecutionUnit J_EU(.PC_target(JUMP_PC_TARGET),.link_value(LINK_VALUE),.PC(PC),.Rdata1(Rdata1),.InstWord(InstWord),.opcode(opcode));

// U-type unit
   wire [31:0] U_OUT;
   U_ExecutionUnit U_EU(.out(U_OUT),.PC(PC),.InstWord(InstWord),.opcode(opcode));


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
   assign MemWrEn = (opcode == `SOPCODE) && !misaligned_store && !halt; // Change this to allow stores
   assign RWrEn = ( (opcode == `ROPCODE) || (opcode == `IOPCODE_ld) || (opcode == `IOPCODE_i) || (opcode == `OPCODE_LUI) || (opcode == `OPCODE_AUIPC) || (opcode == `OPCODE_JAL) || (opcode == `OPCODE_JALR) ) && (Rdst != 0) ;  // only write to reg file if R or L type instruction

   assign RWrdata =
      (opcode == `ROPCODE) ? R_OUT :
      (opcode == `IOPCODE_ld && funct3 == 3'b000) ? {{24{DataWord[7]}},  DataWord[7:0]}  : // LB
      (opcode == `IOPCODE_ld && funct3 == 3'b001) ? {{16{DataWord[15]}}, DataWord[15:0]} : // LH
      (opcode == `IOPCODE_ld && funct3 == 3'b010) ? DataWord                              : // LW
      (opcode == `IOPCODE_ld && funct3 == 3'b100) ? {24'b0,  DataWord[7:0]}               : // LBU
      (opcode == `IOPCODE_ld && funct3 == 3'b101) ? {16'b0,  DataWord[15:0]}              : // LHU
      (opcode == `IOPCODE_i)                      ? I_OUT                                 :            
      (opcode == `OPCODE_LUI || opcode == `OPCODE_AUIPC) ? U_OUT :
      (opcode == `OPCODE_JAL || opcode == `OPCODE_JALR) ? LINK_VALUE :
      32'b1010101010;


   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
   assign NPC =
   (opcode == `OPCODE_JAL)   ? JUMP_PC_TARGET :
   (opcode == `OPCODE_JALR)  ? JUMP_PC_TARGET :
   (opcode == `OPCODE_BRANCH && BR_TAKE) ? BR_NPC :
   PC_Plus_4;

endmodule // SingleCycleCPU


//set the





// Incomplete version of Lab2 execution unit
// You will need to extend it. Feel free to modify the interface also
module R_ExecutionUnit(out, opA, opB, func, auxFunc, opcode);
   output [`WORD_WIDTH-1:0] out; //out is the data that is gettting fed into the register file
   input [`WORD_WIDTH-1:0]  opA, opB; //opA = data out of regA, opB = data out of regB
   input [2:0] func;
   input [6:0] auxFunc;
   input [6:0]   opcode;

   wire [`WORD_WIDTH-1:0] addSub;
   wire signed [63:0] mult_signed   = $signed(opA) * $signed(opB);
   wire [63:0] mult_signed_u = $signed(opA) * $unsigned(opB);
   wire [63:0] mult_unsigned = $unsigned(opA) * $unsigned(opB);
   wire signed [31:0] div_signed   = $signed(opA) / $signed(opB);
   wire signed [31:0] rem_signed   = $signed(opA) % $signed(opB);
   // Only supports add and subtract
   // assign addSub = (auxFunc == 7'b0100000) ? (opA - opB) : (opA + opB);
   // assign out = (func == 3'b000) ? addSub : 32'hXXXXXXXX;

   assign out =
   
   
   
   (opcode == `ROPCODE && func == 3'b000 && auxFunc == 7'b0000000) ? opA + opB  :  //add
                (opcode == `ROPCODE && func == 3'b000 && auxFunc == 7'b0100000) ? opA - opB  :    //sub
                (opcode == `ROPCODE && func == 3'b001 && auxFunc == 7'b0000000) ? opA << (opB & 5'b11111) :    //sll
                (opcode == `ROPCODE && func == 3'b010 && auxFunc == 7'b0000000) ? $signed(opA) < $signed(opB) :   //slt
                (opcode == `ROPCODE && func == 3'b011 && auxFunc == 7'b0000000) ? $unsigned(opA) < $unsigned(opB) :   //sltu
                (opcode == `ROPCODE && func == 3'b100 && auxFunc == 7'b0000000) ? opA ^ opB : //xor
                (opcode == `ROPCODE && func == 3'b101 && auxFunc == 7'b0000000) ? opA >> (opB & 5'b11111) : // srl
                (opcode == `ROPCODE && func == 3'b101 && auxFunc == 7'b0100000) ? $signed(opA) >>> (opB & 5'b11111) :    //sra
                (opcode == `ROPCODE && func == 3'b110 && auxFunc == 7'b0000000) ? opA | opB :
                (opcode == `ROPCODE && func == 3'b111 && auxFunc == 7'b0000000) ? opA & opB :
                (opcode == `ROPCODE && func == 3'b000 && auxFunc == 7'b0000001) ? mult_signed[31:0] :  //mul
                (opcode == `ROPCODE && func == 3'b001 && auxFunc == 7'b0000001) ? mult_signed[63:32] :   //mulh
                (opcode == `ROPCODE && auxFunc == 7'b0000001 && func == 3'b010) ? mult_signed_u[63:32] ://mulhsu
                (opcode == `ROPCODE && auxFunc == 7'b0000001 && func == 3'b011) ? mult_unsigned[63:32] :    //mulhu
                (opcode == `ROPCODE && auxFunc == 7'b0000001 && func == 3'b100) ? div_signed : // div
                (opcode == `ROPCODE && auxFunc == 7'b0000001 && func == 3'b101) ? ($unsigned(opA) / $unsigned(opB)) :   // divu
                (opcode == `ROPCODE && auxFunc == 7'b0000001 && func == 3'b110) ? rem_signed : // rem
                (opcode == `ROPCODE && auxFunc == 7'b0000001 && func == 3'b111) ? ($unsigned(opA) % $unsigned(opB)) :   // remu
                32'b10101001010;



   
endmodule // R_ExecutionUnit


module I_ExecutionUnit (
   output [`WORD_WIDTH-1:0] out,
   input  [`WORD_WIDTH-1:0]  opA,
   input  [11:0] imm12,
   input   [2:0] func,
   input [6:0] opcode

);

   

assign out =
    (opcode == `IOPCODE_i && func == 3'b000) ? opA + {{20{imm12[11]}}, imm12} :              // ADDI
    (opcode == `IOPCODE_i && func == 3'b010) ? ($signed(opA) < $signed({{20{imm12[11]}}, imm12})) : // SLTI
    (opcode == `IOPCODE_i && func == 3'b011) ? ($unsigned(opA) < $unsigned({{20{imm12[11]}}, imm12})) : // SLTIU
    (opcode == `IOPCODE_i && func == 3'b100) ? (opA ^ {{20{imm12[11]}}, imm12}) :       // XORI
    (opcode == `IOPCODE_i && func == 3'b110) ? (opA | {{20{imm12[11]}}, imm12}) :      // ORI
    (opcode == `IOPCODE_i && func == 3'b111) ? (opA & {{20{imm12[11]}}, imm12}) :           // ANDI
    (opcode == `IOPCODE_i && func == 3'b001) ? (opA << imm12[4:0]) :                 // SLLI
    (opcode == `IOPCODE_i && func == 3'b101 && imm12[11:5] == 7'b0000000) ? (opA >> imm12[4:0]) : // SRLI
    (opcode == `IOPCODE_i && func == 3'b101 && imm12[11:5] == 7'b0100000) ? ($signed(opA) >>> imm12[4:0]) :   // SRAI

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

module U_ExecutionUnit (
   output [`WORD_WIDTH-1:0] out,
   input  [`WORD_WIDTH-1:0] PC,
   input  [31:0] InstWord,
   input  [6:0]  opcode
);

   wire [31:0] immU = {InstWord[31:12], 12'b0};

   assign out =
      (opcode == `OPCODE_LUI)   ? immU :
      (opcode == `OPCODE_AUIPC) ? PC + immU :
      32'bz;

endmodule

module Branch_ExecutionUnit (
   output [`WORD_WIDTH-1:0] NPC_out,  
   output take_branch,                
   input  [`WORD_WIDTH-1:0] PC,        
   input  [`WORD_WIDTH-1:0] Rdata1,    
   input  [`WORD_WIDTH-1:0] Rdata2,    
   input  [31:0] InstWord,            
   input  [2:0]  funct3,
   input  [6:0]  opcode
);

   // Branch immediate (B-type)
   wire [31:0] immB = {{20{InstWord[31]}}, InstWord[7], InstWord[30:25], InstWord[11:8], 1'b0};

   // Branch condition
   assign take_branch =
      (opcode == `OPCODE_BRANCH) && (
         (funct3 == 3'b000 && (Rdata1 == Rdata2)) || // BEQ
         (funct3 == 3'b001 && (Rdata1 != Rdata2)) || // BNE
         (funct3 == 3'b100 && ($signed(Rdata1) <  $signed(Rdata2))) ||   // BLT
         (funct3 == 3'b101 && ($signed(Rdata1) >= $signed(Rdata2))) ||   // BGE
         (funct3 == 3'b110 && ($unsigned(Rdata1) <  $unsigned(Rdata2))) ||   // BLTU
         (funct3 == 3'b111 && ($unsigned(Rdata1) >= $unsigned(Rdata2)))       // BGEU
      );

   
   assign NPC_out = PC + immB;

endmodule

module Jump_ExecutionUnit (
   output [`WORD_WIDTH-1:0] PC_target,  
   output [`WORD_WIDTH-1:0] link_value,  
   input  [`WORD_WIDTH-1:0] PC,
   input  [`WORD_WIDTH-1:0] Rdata1,
   input  [31:0] InstWord,
   input  [6:0]  opcode
);

   // Jump immediates
   wire [31:0] immJ = {{12{InstWord[31]}}, InstWord[19:12], InstWord[20], InstWord[30:21], 1'b0};
   wire [31:0] immI = {{20{InstWord[31]}}, InstWord[31:20]};

   // Link value
   assign link_value = PC + 4;

   // Target calculation
   assign PC_target =
       (opcode == `OPCODE_JAL)  ? (PC + immJ) :
       (opcode == `OPCODE_JALR) ? ((Rdata1 + immI) & 32'hFFFFFFFE) :
       32'bz;

endmodule
