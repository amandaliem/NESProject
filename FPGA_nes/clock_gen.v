/* clock */
// NOTE: subject to change - how to synchronize PPU/CPU? 
// PPU = m_clock / 4  = 21.4773 MHz / 4  = 5.37 MHz
// CPU = m_clock / 12 = 21.4773 MHz / 12 = 1.79 MHz

module clock_gen (input m_clk, output ppu_clk, output cpu_clk);
    
    reg [3:0] cycle = 0;
    reg ppu = 1;
    reg cpu = 0;

    assign ppu_clk = ppu;
    assign cpu_clk = cpu;

    // cpu only runs 1/12 during the 12th cycle (cycle 11) 
    // ppu only runs 4/12 during cycles 0, 3, 6, 9
    always @(posedge m_clk) begin
        if (cycle == 10)begin
            cpu <= 1;
            ppu <= 0;
        end
        else if (cycle == 11 || cycle == 2 || cycle == 5 || cycle == 8) begin
            ppu <= 1;
            cpu <= 0;
        end
        else begin
            ppu <= 0;
            cpu <= 0;
        end
            
        if (cycle == 11)
            cycle <= 0;
        else
            cycle <= cycle + 1;
    end
endmodule