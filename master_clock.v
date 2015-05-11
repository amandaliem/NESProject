/* clock */
`timescale 1ps/1ps

// 21.4773 MHz = 21477300 cycles / second
// 21477300 cycles / second = 1 cycle = 46.56 ns per cycle ( round ?? ) 
// 46.56 ns = 46560.7874 ps

module master_clock(output clk);
    reg theClock = 1;

    assign clk = theClock;
    
    always begin
        #46560;
        theClock = !theClock;
    end
endmodule
