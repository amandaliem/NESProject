module modePrinter(
            input clk,
            input immediate, 
            input absolute, 
            input zpg_absolute,
            input implied,
            input accumulator, 
            input abs_indexed_x,
            input abs_indexed_y,
            input zpg_indexed_x,
            input zpg_indexed_y,
            input indirect,
            input indirect_x,
            input indirect_y,
            input relative,
            input [7:0]instruction,
            input newinst
);


wire unknown = !immediate && !absolute && !zpg_absolute && !implied && !accumulator
               && !abs_indexed_x && !abs_indexed_y && !zpg_indexed_x && !zpg_indexed_y
               && !indirect && !indirect_x && !indirect_y && !relative;

always @(posedge clk)begin
    if (newinst) begin
    if (immediate)
        $display("immediate %x", instruction);
    if (absolute)
        $display("absolute %x", instruction);
    if (zpg_absolute)
        $display("zpg_absolute %x", instruction);
   if (implied)
        $display("implied %x", instruction);
   if (accumulator)
        $display("accumulator %x", instruction);
   if (abs_indexed_x)
        $display("abs_indexed_x %x", instruction);
   if (abs_indexed_y)
        $display("abs_indexed_y %x", instruction);
   if (zpg_indexed_x)
        $display("zpg_indexed_x %x", instruction);
   if (zpg_indexed_y)
        $display("zpg_indexed_y %x", instruction);
   if (indirect)
        $display("indirect %x", instruction);
   if (indirect_x)
        $display("indirect_x %x", instruction);
   if (indirect_y)
        $display("indirect_y %x", instruction);
    if (relative)
        $display("relative %x", instruction);
   if (unknown)
        $display("unknown %x", instruction);
   end
end


endmodule
