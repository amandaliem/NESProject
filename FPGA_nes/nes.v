module main(input m_clk);
	// Master clock is split into PPU clock and CPU clock 
	initial begin
		//$dumpfile("cpu.vcd");
		//$dumpvars(1,main);
		//$dumpvars(1,c);
	end
		 
	wire ppu_clk;
	wire cpu_clk;
	clock_gen c1(m_clk, ppu_clk, cpu_clk);

	cpu c(cpu_clk);
endmodule
