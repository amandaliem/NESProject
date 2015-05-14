`timescale 1ps/1ps


module main();


wire m_clk;
// Master clock is split into PPU clock and CPU clock 
master_clock c0(m_clk);

    initial begin
        $dumpfile("cpu.vcd");
        $dumpvars(1,main);
        $dumpvars(1,c);
    end

wire ppu_clk;
wire cpu_clk;
clock_gen c1(m_clk, ppu_clk, cpu_clk);

wire [15:0]address_bus_in;
wire [7:0] data_bus_in;
wire [15:0]address_bus_out;
wire [7:0] data_bus_out;
wire wen;

cpu c(cpu_clk);


endmodule
