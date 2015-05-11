`timescale 1ps/1ps


module main();


wire m_clk;
// Master clock is split into PPU clock and CPU clock 
master_clock c0(m_clk);

    initial begin
        $dumpfile("cpu.vcd");
        $dumpvars(1,main);
    end


wire ppu_clk;
wire cpu_clk;
clock_gen c1(m_clk, ppu_clk, cpu_clk);

cpu c(cpu_clk);

endmodule
