/**************************************************************************************************************
* Description
*	This is the top-level of a MIPS processor
* This processor is written Verilog-HDL. Also, it is synthesizable into hardware.
* Parameter MEMORY_DEPTH configures the program memory to allocate the program to
* be execute. If the size of the program changes, thus, MEMORY_DEPTH must change.
* This processor was made for computer organization class at ITESO.
****************************************************************************************/


module MIPS_Processor
#(
	parameter MEMORY_DEPTH = 32
)

(
	// Inputs
	input clk,
	input reset,
	input [7:0] PortIn,
	// Output
	output [31:0] ALUResultOut,
	output [31:0] PortOut
);
//******************************************************************/
//******************************************************************/
assign  PortOut = 0;

//******************************************************************/
//******************************************************************/
// Data types to connect modules
wire BranchNE_wire;
wire P_ID_EX_BranchNE_wire; //output from pipeline 2
wire P_EX_MEM_BranchNE_wire;//output from pipeline 3

wire BranchEQ_wire;
wire P_ID_EX_BranchEQ_wire; //output from pipeline 2
wire P_EX_MEM_BranchEQ_wire;//output from pipeline 3

wire RegDst_wire;
wire NotZeroANDBrachNE;
wire ZeroANDBrachEQ;
wire ORForBranch;
wire ALUSrc_wire;
wire PCSrc_wire; //selector for first mux
wire MUX_ForJump_wire; //wire from first mux to PC

wire RegWrite_wire;
wire P_ID_EX_RegWrite_wire; //output from pipeline 2
wire P_EX_MEM_RegWrite_wire;//output from pipeline 3
wire P_MEM_WB_RegWrite_wire;//output from pipeline 4

wire MemWrite_wire;
wire P_ID_EX_MemWrite_wire;//output from pipeline 2
wire P_EX_MEM_MemWrite_wire;//output from pipeline 3

wire MemRead_wire; 
wire P_ID_EX_MemRead_wire;//output from pipeline 2
wire P_EX_MEM_MemRead_wire;//output from pipeline 3

wire MemtoReg_wire; //input for pipeline 2
wire P_ID_EX_MemtoReg_wire; //output from pipeline 2/input for pipe 3
wire P_EX_MEM_MemtoReg_wire; //output from pipeline 3
wire P_MEM_WB_MemtoReg_wire; //output from pipeline 4

wire Zero_wire;
wire P_EX_MEM_Zero_wire; //output from pipeline 3
wire [2:0] ALUOp_wire;
wire [3:0] ALUOperation_wire;
wire [4:0] WriteRegister_wire;
wire [4:0] RtOrRd_wire; //Mux output below ALUControl
wire [4:0] P_EX_MEM_RtOrRd_wire; //lower wire from 3rd pipe to last pipe
wire [4:0] P_MEM_WB_RtOrRd_wire; // wire from output 4th pipe to Write register in the RegisterFile

wire [31:0] PC_wire;

wire [31:0] MUX_WriteData_wire;//Last mux output

wire [31:0] MUX_Mem_wire; //ReadData output from Data Memory, also input for pipeline 4
wire [31:0] P_MEM_WB_MUX_Mem_wire;//output from pipeline 4

wire [31:0] P_IF_ID_PC4_wire;
wire [31:0] P_IF_ID_Instruction_wire;

wire [132:0] P_ID_EX_wire;

wire [132:0] P_ID_EX_wire;

wire [31:0] Instruction_wire;
wire [31:0] ReadData1_wire;

wire [31:0] ReadData2_wire;
wire [31:0] P_ID_EX_ReadData2_wire;//output from pipeline 2
wire [31:0] P_EX_MEM_ReadData2_wire;//output from pipeline 3
wire [31:0] P_MEM_WB_ReadData2_wire;//output from pipeline 4

wire [31:0] InmmediateExtend_wire;
wire [31:0] ReadData2OrInmmediate_wire;
wire [31:0] ALUResult_wire;
wire [31:0] P_EX_MEM_ALUResult_wire; //ALUResult wire, output from pipeline 3
wire [31:0] PC_4_wire;

wire [31:0] PC_Jump_wire;
wire [31:0] P_EX_MEM_PC_Jump_wire; // output from pipeline 3
wire [31:0] InmmediateExtendAnded_wire;
wire [31:0] PCtoBranch_wire;
integer ALUStatus;

//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/

Multiplexer2to1
#(
	.NBits(32)
)
MUX_JumptoRegister//In this diagram, the first MUX. Sets the program counter value to the normal value or the jump value
( 
	.Selector(PCSrc_wire),						//output from AND that is between pipe 3 & pipe 4
	.MUX_Data0(PC_4_wire),						//PC + 4
	.MUX_Data1(P_EX_MEM_ALUResult_wire),	//ALUResult from pipeline 3 output
	.MUX_Output(MUX_ForJump_wire)
);


Control
ControlUnit
(
	//.Jump(Jump_wire),
	.OP(P_IF_ID_Instruction_wire[31:26]),
	.RegDst(RegDst_wire),
	.BranchNE(BranchNE_wire),
	.BranchEQ(BranchEQ_wire),
	.MemRead(MemRead_wire),
	.MemWrite(MemWrite_wire),
	.MemtoReg(MemtoReg_wire),
	.ALUOp(ALUOp_wire),
	.ALUSrc(ALUSrc_wire),
	.RegWrite(RegWrite_wire)
);



ProgramMemory
#(
	.MEMORY_DEPTH(MEMORY_DEPTH)
)
ROMProgramMemory
(
	.Address(PC_wire),
	.Instruction(Instruction_wire)
);

Adder32bits
PC_Puls_4
(
	.Data0(PC_wire),
	.Data1(4),
	
	.Result(PC_4_wire)
);

PC_Register
ProgramCounter
(
	.clk(clk),
	.reset(reset),
	.NewPC(MUX_ForJump_wire),
	.PCValue(PC_wire)
)



//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/



///*First Pipe, IF/ID stage (Instruction Fetch/ Instruction decode)*
//******************************************************************/
Register_pipeline
P1_IF_ID
#(
	.N(64) // PC + 4 				= 32 bits
)			 // Instruction_wire = 32 bits

(
	.clk(clk),
	.reset(reset),
	.enable(enable),
	.DataInput({PC_4_wire,Instruction_wire}),
	
	.DataOutput({P_IF_ID_PC4_wire, P_IF_ID_Instruction_wire})
);
//******************************************************************/


Multiplexer2to1
#(
	.NBits(5)
)
MUX_ForRTypeAndIType
(
	.Selector(RegDst_wire),
	.MUX_Data0(Instruction_wire[20:16]),
	.MUX_Data1(Instruction_wire[15:11]),
	
	.MUX_Output(WriteRegister_wire)

);



RegisterFile
Register_File
(
	.clk(clk),
	.reset(reset),
	.RegWrite(P_MEM_WB_RegWrite_wire),					 //output from pipeline 4, P_MEM_WB_RegWrite_wire
	.WriteRegister(P_MEM_WB_RtOrRd_wire), 				 //output from pipeline 4, the bottom wire in the diagram
	.ReadRegister1(P_IF_ID_Instruction_wire[25:21]), //Pipe 1 output (Instruction wire [25:21])
	.ReadRegister2(P_IF_ID_Instruction_wire[20:16]), //Pipe 1 output (Instruction wire [20:16])
	.WriteData(MUX_WriteData_wire),         		    //Last mux output
	.ReadData1(ReadData1_wire),
	.ReadData2(ReadData2_wire)

);

SignExtend
SignExtendForConstants
(   
	.DataInput(P_IF_ID_Instruction_wire[15:0]),
   .SignExtendOutput(InmmediateExtend_wire)
);


///*Second Pipe, ID/EX stage (Instruction decode/ Execute)*
//******************************************************************/
Register_pipeline
P2_ID_EX
#(
	.N(149) //PC_4_wire  = 32 bits 		WB RegWrite_wire,MemtoReg_wire  = 2 bit
)			  //RD1, RD2   = 64 bits 		M  bne, beq, MemWrt, MemRd		  = 4 bits
			  //SignExtend = 32 bits 		EX RegDst, ALUOp(3), ALUSrc	  = 5 bits
(			  //rt, rd		= 10 bits
	.clk(clk),
	.reset(reset),
	.enable(enable),//				rd1								rd2																			rt												rd								WB-->														M--> 																															EX-->
	.DataInput({P_IF_ID_PC4_wire, ReadData1_wire,			ReadData2_wire,			InmmediateExtend_wire, 			 P_IF_ID_Instruction_wire[20:16], P_IF_ID_Instruction_wire[15:11],  RegWrite_wire, 			 MemtoReg_wire,		   BranchNE_wire, 			BranchEQ_wire,			  MemWrite_wire, 				MemRead_wire, 			 RegDst_wire, 			 ALUOp_wire, 			ALUSrc_wire}),
	.DataOutput({P_ID_EX_PC4_wire, P_ID_EX_ReadData1_wire, P_ID_EX_ReadData2_wire, P_ID_EX_InmmediateExtend_wire, P_ID_EX_Instruction_wire[20:16], P_ID_EX_Instruction_wire[15:11], P_ID_EX_RegWrite_wire, P_ID_EX_MemtoReg_wire, P_ID_EX_BranchNE_wire,  P_ID_EX_BranchEQ_wire, P_ID_EX_MemWrite_wire,   P_ID_EX_MemRead_wire, P_ID_EX_RegDst_wire, P_ID_EX_ALUOp_wire, P_ID_EX_ALUSrc_wire}),
);

//******************************************************************/


Adder32bits
PC_Plus_JumpAddress	//32 bit Adder, adds the current pc value plus the new jump direction
(
	.Data0(ShiftSignExtend_wire),
	.Data1(P_ID_EX_PC4_wire),//PC + 4 from Pipe2
	
	.Result(PC_Jump_wire)
);


ShiftLeft2
Shift_SignExtend //Shift after the sign extend module, conected to the direction adder
(
	.DataInput(P_ID_EX_InmmediateExtend_wire),
	.DataOutput(ShiftSignExtend_wire)
);


Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForReadDataAndInmediate
(
	.Selector(P_ID_EX_ALUSrc_wire),
	.MUX_Data0(P_ID_EX_ReadData2_wire),
	.MUX_Data1(P_ID_EX_InmmediateExtend_wire),
	
	.MUX_Output(ReadData2OrInmmediate_wire)

);


ALUControl
ArithmeticLogicUnitControl
(
	.ALUOp(P_ID_EX_ALUOp_wire),
	.ALUFunction(P_ID_EX_InmmediateExtend_wire[5:0]),
	.ALUOperation(ALUOperation_wire)

);



ALU
Arithmetic_Logic_Unit 
(
	.ALUOperation(ALUOperation_wire),
	.A(P_ID_EX_ReadData1_wire),
	.B(ReadData2OrInmmediate_wire),
	.Zero(Zero_wire),
	.ALUResult(ALUResult_wire)
);

Multiplexer2to1
#(
	.NBits(5)//5bits for output
)
MUX_ForRtRd//The mux below ALUControl
(
	.Selector(P_ID_EX_RegDst_wire),
	.MUX_Data0(P_ID_EX_Instruction_wire[20:16]),
	.MUX_Data1(P_ID_EX_Instruction_wire[15:11]),
	
	.MUX_Output(RtOrRd_wire)

);




///*Third Pipe, EX/MEM stage (Execute / Memory Access)*
//******************************************************************/
Register_pipeline
P3_EX_MEM
#(
	.N(108)  //RtOrRd_wire 	 				  = 5 bits 		      Zero_wire 				     				  = 1 bit		PC_Jump_wire	= 32 bits
)			  //P_ID_EX_ReadData2_wire 	  = 32 bits 		WB RegWrite_wire, P_ID_EX_MemtoReg_wire  = 2 bit
			  //ALUResult_wire 				  = 32 bits 		M  bne, beq, MemWrt, MemRd					  = 4 bits
(
	.clk(clk),
	.reset(reset),
	.enable(enable),//																																					WB-->						  								  M-->
	.DataInput({RtOrRd_wire, 			  P_ID_EX_ReadData2_wire,  ALUResult_wire, 			 Zero_wire, 			PC_Jump_wire, 			  P_ID_EX_RegWrite_wire,  P_ID_EX_MemtoReg_wire,  P_ID_EX_BranchNE_wire,   P_ID_EX_BranchEQ_wire,  P_ID_EX_MemWrite_wire,   P_ID_EX_MemRead_wire}),
	.DataOutput({P_EX_MEM_RtOrRd_wire, P_EX_MEM_ReadData2_wire, P_EX_MEM_ALUResult_wire, P_EX_MEM_Zero_wire, P_EX_MEM_PC_Jump_wire, P_EX_MEM_RegWrite_wire, P_EX_MEM_MemtoReg_wire, P_EX_MEM_BranchNE_wire,  P_EX_MEM_BranchEQ_wire, P_EX_MEM_MemWrite_wire,   P_EX_MEM_MemRead_wire})
);

//******************************************************************/


ANDGate
AndGate
(
	.A(P_EX_MEM_BranchNE_wire),
	.B(P_EX_MEM_BranchEQ_wire),
	.C(P_EX_MEM_Zero_wire),
	
	.D(PCSrc_wire)
);


DataMemory
#(	
	.DATA_WIDTH(32)
)
Data_Memory
(
	.WriteData(P_EX_MEM_ReadData2_wire), //from pipeline 3
	.Address(P_EX_MEM_ALUResult_wire),   //from pipeline 3
	.MemWrite(P_EX_MEM_MemWrite_wire),
	.MemRead(P_EX_MEM_MemRead_wire),
	.clk(clk),
	.ReadData(MUX_Mem_wire)//ReadData output
);



///*Fourth Pipe, MEM/WB stage (Memory Access / Write Back)*
//******************************************************************/
Register_pipeline
P4_MEM_WB
#(
	.N(71) //P_EX_MEM_RtOrRd_wire 	  = 5 bits 		   (WB) P_EX_MEM_RegWrite_wire, P_EX_MEM_MemtoReg_wire  = 2 bit
)			  //P_EX_MEM_ReadData2_wire  = 32 bits 		
			  //MUX_Mem_wire   			  = 32 bits 		
(
	.clk(clk),
	.reset(reset),
	.enable(enable),//																		  				WB-->						 
	.DataInput ({P_EX_MEM_RtOrRd_wire, P_EX_MEM_ReadData2_wire, MUX_Mem_wire, 			  P_EX_MEM_RegWrite_wire, P_EX_MEM_MemtoReg_wire}),
	.DataOutput({P_MEM_WB_RtOrRd_wire, P_MEM_WB_ReadData2_wire, P_MEM_WB_MUX_Mem_wire, P_MEM_WB_RegWrite_wire, P_MEM_WB_MemtoReg_wire})
);

//******************************************************************/

Multiplexer2to1
#(
	.NBits(32)
)
MUX_MemtoReg //Mux 3 aftter tha data memory, choose between ALUResult and NewAddress
(
	.Selector(P_MEM_WB_MemtoReg_wire),
	.MUX_Data0(P_MEM_WB_MUX_Mem_wire),
	.MUX_Data1(P_MEM_WB_ReadData2_wire),

	.MUX_Output(MUX_WriteData_wire)

);


assign ALUResultOut = ALUResult_wire;


endmodule 