
`timescale 1ns/100fs

module wrap_saed32_64x4(CLK, A0, D0, Q0, WE0, WEM0, CE0, A1, D1, Q1, WE1, CE1, WEM1);

input CLK;
input [5:0] A0;
input [3:0] D0;
input [3:0] WEM0;   
output [3:0] Q0;
input WE0;
input CE0;
input [5:0] A1;
input [3:0] D1;
input [3:0] WEM1;   
output [3:0] Q1;
input WE1;
input CE1;

wire cs0;
wire cs1;

wire re0;
wire re1;

wire we0;
wire we1;

assign cs0 = ~CE0;
assign cs1 = ~CE1;

assign we0 = ~(CE0 & WE0);
assign we1 = ~(CE1 & WE1);

assign re0 = ~CE0 & WE0;
assign re1 = ~CE1 & WE1;

  SRAM2RW64X4 ram (
     .A1(A0),
     .A2(A1),
     .CE1(CLK),
     .CE2(CLK),
     .WEB1(we0),
     .WEB2(we1),
     .OEB1(re0),
     .OEB2(re1),
     .CSB1(cs0),
     .CSB2(cs1),
     .I1(D0),
     .I2(D1),
     .O1(Q0),
     .O2(Q1)
     );

endmodule

