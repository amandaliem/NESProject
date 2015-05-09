/* instruction memory */

`timescale 1ps/1ps

module mem(input clk,
    // read ports
    input [7:0]raddr0, output [7:0]rdata0,
    // write port
    input writeEnable, input [7:0]writeAddress, input [7:0]writeData);

    reg [7:0]mem[1023:0];

    /* Simulation -- read initial content from file */
    initial begin
        $readmemh("mem.hex",mem);
    end

    /* memory address register */
    reg [7:0]in0 = 16'hxxxx;
    reg [7:0]in1 = 16'hxxxx;

    /* memory data register */
    reg [7:0]out0 = 16'hxxxx;
    reg [7:0]out1 = 16'hxxxx;

    assign rdata0 = out0;
    assign rdata1 = out1;

    always @(posedge clk) begin
        if (writeEnable) begin
            $display("#mem[%x] <= %x",writeAddress,writeData);
            mem[writeAddress] <= writeData;
        end
        in0 <= raddr0;
        out0 <= mem[in0];        
    end

endmodule
