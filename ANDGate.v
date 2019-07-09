/******************************************************************
* Description
*		This is an AND gate:
* Version:
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	01/03/2014
******************************************************************/
module ANDGate
(
	input A,
	input B,
	input C,
	output reg D
);

always@(*)
	D = (A | B) & C; //Modified for beq & bne

endmodule
//andgate//