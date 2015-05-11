/* instruction memory */

`timescale 1ps/1ps
`define CYCLES 2

// based on readings, 1 cycle latency between memory and output... 

module mem(input clk,
    // read ports
    input [15:0]raddr, output [7:0]rdata, 
    // write port
    input wen, input [15:0]writeAddress, input [7:0]writeData);

    reg [7:0]mem[65536:0];

    /* Simulation -- read initial content from file */
    initial begin
        $readmemh("mem.hex",mem);
    end

    reg [7:0] in = 16'hxxxx;

    assign rdata = in;

    always @(posedge clk) begin
        if (wen) begin
            $display("#mem[%x] <= %x",writeAddress,writeData);
            mem[writeAddress] <= writeData;
        end        
        in <= mem[raddr];
    end

endmodule
