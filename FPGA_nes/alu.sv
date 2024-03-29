`define ADD 0
`define SUB 1
`define AND 2
`define OR  3
`define XOR 4
`define INC 5
`define DEC 6
`define SHR 7
`define SHL 8
`define RTR 9
`define RTL 10
`define LD  12

// RAM access time ~70ns?
// NES version of 6502 doesn't use BCD 

module alu (input clk, input immediate, input i_adc,
				input [7:0]registers[3:0],
				input [7:0]operand,
				input [7:0]newvalue,
            input [7:0]operand1,
            input [7:0]operand2,
            input carry,
            input [4:0]op,
            output [7:0]result,
            output carrybit,
            output overflow
           );

    reg [7:0]output_reg = 8'hxx;
    reg carry_reg = 1'hx;
    reg overflow_reg = 1'hx;

    wire [7:0]operation_result = op == `ADD ? operand1 + operand2 + carry:
                                 op == `SUB ? operand1 - operand2 - carry:
                                 op == `AND ? operand1 & operand2 :
                                 op == `OR  ? operand1 | operand2 :
                                 op == `XOR ? operand1 ^ operand2 :
                                 op == `INC ? operand1 + 1        :
                                 op == `DEC ? operand1 - 1        :
                                 op == `SHR ? operand1 >> 1       :
                                 op == `SHL ? operand1 << 1       :
                                 op == `RTR ?(operand1 >> 1) & (carry << 7 | 8'h7F) :
                                 op == `RTL ?(operand1 << 1) | carry                :
                                 op == `LD  ? operand1            : // no operation needed
                                 8'hxx;
    
    wire carry_wire = (op == `ADD) ? 
                    ((operand1[7] == 1) && (operand2[7] == 1)) ||
                    ((operand1[7] != operand2[7]) && (newvalue[7] == 0)): 
                    (op == `SUB) ? (newvalue <= 0) : 0; // FIX THIS 

    wire overflow_wire = i_adc && immediate ? 
                    ((registers[0][7] == 1) && (operand[7] == 1) && (newvalue[7] != 1) ||
                     (registers[0][7] == 0) && (operand[7] == 0) && (newvalue[7] != 0)) : 0; 

    assign result = output_reg;
    assign carrybit = carry_reg;
    assign overflow = overflow_reg;

    // ALU operations take 1 cycle 
    always @(posedge clk)begin
        output_reg <= operation_result;
        carry_reg <= carry_wire;
        overflow_reg <= overflow_wire;
    end
endmodule