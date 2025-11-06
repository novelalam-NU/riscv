// Testbench for Northwestern - CompEng 361 - Lab3 -- Version 1.1
`define CHAR_WIDTH 8
`define MAX_CHAR 80

module tb;
    reg clk, rst;
    reg exit;
    wire halt;
    reg [`CHAR_WIDTH*`MAX_CHAR-1:0] mem_in_fname, mem_out_fname, regs_in_fname, regs_out_fname;
    reg [`CHAR_WIDTH*`MAX_CHAR-1:0] signal_dump_fname;

    // Single Cycle CPU instantiation
    SingleCycleCPU CPU (halt, clk, rst);

    // Clock Period = 10 time units
    always
        #50 clk = ~clk;


    always @(posedge clk) begin 
        $display("%0t PC=%08x IR=%08x halt=%x exit=%x x0=%08x x1=%08x x2=%08x x3=%08x x4=%08x x5=%08x x6=%08x x7=%08x x8=%08x x9=%08x x10=%08x x11=%08x x12=%08x x13=%08x x14=%08x x15=%08x x16=%08x x17=%08x x18=%08x x19=%08x x20=%08x x21=%08x x22=%08x x23=%08x x24=%08x x25=%08x x26=%08x x27=%08x x28=%08x x29=%08x x30=%08x x31=%08x", 
                 $time, CPU.PC, CPU.InstWord, halt, exit,
                 CPU.RF.Mem[0], CPU.RF.Mem[1], CPU.RF.Mem[2], CPU.RF.Mem[3], CPU.RF.Mem[4], CPU.RF.Mem[5], CPU.RF.Mem[6], CPU.RF.Mem[7], CPU.RF.Mem[8], CPU.RF.Mem[9],
                 CPU.RF.Mem[10], CPU.RF.Mem[11], CPU.RF.Mem[12], CPU.RF.Mem[13], CPU.RF.Mem[14], CPU.RF.Mem[15], CPU.RF.Mem[16], CPU.RF.Mem[17], CPU.RF.Mem[18], CPU.RF.Mem[19],
                 CPU.RF.Mem[20], CPU.RF.Mem[21], CPU.RF.Mem[22], CPU.RF.Mem[23], CPU.RF.Mem[24], CPU.RF.Mem[25], CPU.RF.Mem[26], CPU.RF.Mem[27], CPU.RF.Mem[28], CPU.RF.Mem[29],
                 CPU.RF.Mem[30], CPU.RF.Mem[31]);
 
        #1000;   

    always @(negedge clk)
        if (halt)
            exit = 1;

    

    initial begin

        // Read commandline options
        if (!$value$plusargs("MEM_IN=%s", mem_in_fname))
            mem_in_fname = "mem_in.hex";
        if (!$value$plusargs("REGS_IN=%s", regs_in_fname))
            regs_in_fname = "regs_in.hex";
        if (!$value$plusargs("REGS_OUT=%s", regs_out_fname))
            regs_out_fname = "regs_out.hex";
        if (!$value$plusargs("MEM_OUT=%s", mem_out_fname))

            mem_out_fname = "mem_out.hex";
         if (!$value$plusargs("DUMP=%s", signal_dump_fname))
            signal_dump_fname = "single.vcd";

        // Clock and reset steup
        #0 rst = 0; exit = 0; clk = 0;
        #0 rst = 1; 
        
        

        // Load program memory and regs
        #0 $readmemh(mem_in_fname, CPU.MEM.Mem);
        #0 $readmemh(regs_in_fname, CPU.RF.Mem);

        // Dumpfile
        $dumpfile(signal_dump_fname);
        $dumpvars();

        // #0 $monitor($time,, "PC=%08x IR=%08x halt=%x exit=%x x1=%08x x2=%08x x3=%08x x4=%08x",
        //             CPU.PC, CPU.InstWord, halt, exit,
        //             CPU.RF.Mem[1], CPU.RF.Mem[2], CPU.RF.Mem[3], CPU.RF.Mem[4]);

        // Exit???
        wait(exit);
      
        // Dump memory and regs 
        #0 $writememh(regs_out_fname, CPU.RF.Mem);
        #0 $writememh(mem_out_fname, CPU.MEM.Mem);
        
        $finish;      
   end

endmodule // tb


module Mem(InstAddr, InstOut,
         DataAddr, DataSize, DataIn, DataOut, WE, CLK);
    input [31:0] InstAddr, DataAddr;
    input [1:0] 	DataSize;   
    input [31:0] DataIn;   
    output [31:0] InstOut, DataOut;  
    input      WE, CLK;
    reg [7:0] 	Mem[0:1024];

    wire [31:0] 	DataAddrH, DataAddrW, InstAddrW;

    // Instruction Addresses are word aligned
    assign InstAddrW = InstAddr & 32'hfffffffc; // this is why alignment is a thing, memory desont look at last 2 bits 
   
    assign DataAddrH = DataAddr & 32'hfffffffe;
    assign DataAddrW = DataAddr & 32'hfffffffc;

    // Little endian
    assign InstOut = {Mem[InstAddrW+3], Mem[InstAddrW+2], 
		Mem[InstAddrW+1], Mem[InstAddrW]};

    // Little endian
    assign DataOut = (DataSize == 2'b00) ? {4{Mem[DataAddr]}} :
	       ((DataSize == 2'b01) ? {2{Mem[DataAddrH+1],Mem[DataAddrH]}} :
		{Mem[DataAddrW+3], Mem[DataAddrW+2], Mem[DataAddrW+1], Mem[DataAddrW]});
   
     always @ (negedge CLK)
        if (WE) begin
	        case (DataSize)
            2'b00: begin // Write byte
                Mem[DataAddr] <= DataIn[7:0];
            end
            2'b01: begin  // Write halfword
                Mem[DataAddrH] <= DataIn[7:0];
                Mem[DataAddrH+1] <= DataIn[15:8];
            end
            2'b10, 2'b11: begin // Write word
                Mem[DataAddrW] <= DataIn[7:0];
                Mem[DataAddrW+1] <= DataIn[15:8];
                Mem[DataAddrW+2] <= DataIn[23:16];
                Mem[DataAddrW+3] <= DataIn[31:24];
            end
        endcase // case (Size)
     end
endmodule // Mem

module RegFile(AddrA, DataOutA,
	       AddrB, DataOutB,
	       AddrW, DataInW, WenW, CLK);
   input [4:0] AddrA, AddrB, AddrW;
   output [31:0] DataOutA, DataOutB;  
   input [31:0]  DataInW;
   input 	 WenW, CLK;
   reg [31:0] 	 Mem[0:31];
   
   assign DataOutA = (AddrA == 0) ? 32'h00000000 : Mem[AddrA];
   assign DataOutB = (AddrB == 0) ? 32'h00000000 : Mem[AddrB]; 

   always @ (negedge CLK) begin
     if (WenW) begin
       Mem[AddrW] <= DataInW;
     end
      Mem[0] <= 0; // Enforce the invariant that x0 = 0
   end
   
endmodule // RegFile


module Reg(Din, Qout, WE, CLK, RST);
   parameter width = 32;
   parameter init = 0;
   input [width-1:0] Din;
   output [width-1:0] Qout;
   input 	      WE, CLK, RST;

   reg [width-1:0]    Qout;
   
   always @ (negedge CLK or negedge RST)
     if (!RST)
       Qout <= init;
     else
       if (WE)
	     Qout <= Din;
  
endmodule // Reg
