/******************************************************************
* Description
*	This is the data memory for the MIPS processor
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	01/03/2014
******************************************************************/

module DataMemory 
#(	parameter DATA_WIDTH=32, //tamaño de la memoria
	parameter MEMORY_DEPTH = 1024

)
(
	input [DATA_WIDTH-1:0] WriteData,
	input [DATA_WIDTH-1:0]  Address,
	input MemWrite,MemRead, clk,
	output  [DATA_WIDTH-1:0]  ReadData
);
	
	// Declare the RAM variable
	reg [DATA_WIDTH-1:0] ram[MEMORY_DEPTH-1:0];
	wire [DATA_WIDTH-1:0] ReadDataAux;
	
	wire [DATA_WIDTH-1:0] CorrectMemory;
	assign CorrectMemory = Address ;//- 32'h7fffeffc;
	//assign CorrectMemory = Address - 32'h61A80;// 40 000 decimal
	wire [DATA_WIDTH-1:0] newAddress;
	assign newAddress = {2'b0, CorrectMemory[(DATA_WIDTH-1):2]};


	always @ (negedge clk)
	begin
		// Write
		if (MemWrite)
			ram[newAddress] <= WriteData;
	end
	assign ReadDataAux = ram[newAddress];
  	assign ReadData = {DATA_WIDTH{MemRead}}& ReadDataAux;

endmodule
//datamemory//