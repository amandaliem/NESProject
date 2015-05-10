`timescale 1ps/1ps

// INSTRUCTIONS //
`define F0 0
`define F1 1 

// RAM access time ~70ns?
// NES version of 6502 doesn't use BCD 

module main();

    initial begin
        $dumpfile("cpu.vcd");
        $dumpvars(1,main);
    end

    // clock
    wire clk;
    clock c0(clk);

    reg [15:0] cycles = 0;

    // fetch 
    wire [7:0]memOut;
    wire [7:0]memIn = pc; 

    wire wen = 0;
    reg [7:0]res = 8'bxx;
    reg [7:0]raddr = 8'bxx;

    mem i0(clk,memIn,memOut, wen, raddr, res);


    reg [15:0] state = `F0;
    wire [7:0]instruction = memOut;

    wire isOdd = instruction[7:4] % 2 == 1;
    wire isEven = instruction[7:4] % 2 == 0;

    wire zeropage = (instruction[3:0] == 4) || (instruction[3:0] == 5) || 
                    (instruction[3:0] == 6); 
  
    wire oddops = (instruction[3:0] == 1) || (instruction[3:0] == 5) ||
                  (instruction[3:0] == 9) || (instruction[3:0] == 4'hd);
    
    wire evenops = (instruction[3:0] == 6) || ((instruction[3:0] == 10) && !higher_half)||
                   (instruction[3:0] == 14);

    wire zerofourc = (instruction[3:0] == 0) || (instruction[3:0] == 4) ||
                     (instruction[3:0] == 12);

    // 1, 5, 9, D
    wire i_ora = oddops && (instruction[7:4] == 0 || instruction[7:4] == 1);
    wire i_and = oddops && (instruction[7:4] == 2 || instruction[7:4] == 3);
    wire i_eor = oddops && (instruction[7:4] == 4 || instruction[7:4] == 5);
    wire i_adc = oddops && (instruction[7:4] == 6 || instruction[7:4] == 7);
    wire i_sta = oddops && (instruction[7:4] == 8 || instruction[7:4] == 9);
    wire i_lda = oddops && (instruction[7:4] == 10 || instruction[7:4] == 11);
    wire i_cmp = oddops && (instruction[7:4] == 12 || instruction[7:4] == 13);
    wire i_sbc = oddops && (instruction[7:4] == 14 || instruction[7:4] == 15);

    // 6, a, e
    wire i_asl = evenops && (instruction[7:4] == 0 || instruction[7:4] == 1);
    wire i_rol = evenops && (instruction[7:4] == 2 || instruction[7:4] == 3);
    wire i_lsr = evenops && (instruction[7:4] == 4 || instruction[7:4] == 5);
    wire i_ror = evenops && (instruction[7:4] == 6 || instruction[7:4] == 7);
    wire i_stx = evenops && (instruction[7:4] == 8 || instruction[7:4] == 9);
    wire i_ldx = evenops && (instruction[7:4] == 10 || instruction[7:4] == 11);
    wire i_dec = evenops && (instruction[7:4] == 12 || instruction[7:4] == 13);
    wire i_inc = evenops && (instruction[7:4] == 14 || instruction[7:4] == 15);

    // 0, 4, c
    wire i_cpx = zerofourc && (instruction[7:4] == 14);
    wire i_cpy = zerofourc && (instruction[7:4] == 12);
    wire i_ldy = !(instruction == 8'hb0) && zerofourc && ((instruction[7:4] == 10) || (instruction[7:4] == 11));

    wire branch = ((instruction[3:0] == 0) && isOdd) || i_jmp;
    // branches
    wire i_bpl = branch && (instruction[7:4] == 1);
    wire i_bmi = branch && (instruction[7:4] == 3);
    wire i_bvc = branch && (instruction[7:4] == 5);
    wire i_bvs = branch && (instruction[7:4] == 7);
    wire i_bcc = branch && (instruction[7:4] == 9);
    wire i_bcs = branch && (instruction[7:4] == 11);
    wire i_mne = branch && (instruction[7:4] == 13);
    wire i_beq = branch && (instruction[7:4] == 15);
    wire i_bit = (instruction == 8'h24) || (instruction == 8'h2c);
    wire i_jmp = (instruction == 8'h4c) || (instruction == 8'h6c);

    wire flags = isOdd && (instruction[3:0] == 8) && !i_tya;
    // flags
    wire i_clc = flags && (instruction[7:4] == 1);
    wire i_sec = flags && (instruction[7:4] == 3);
    wire i_cli = flags && (instruction[7:4] == 5);
    wire i_sei = flags && (instruction[7:4] == 7);
    wire i_clv = flags && (instruction[7:4] == 11);
    wire i_cld = flags && (instruction[7:4] == 13);
    wire i_sed = flags && (instruction[7:4] == 15);

    // transfer
    wire i_tax = instruction == 8'haa; 
    wire i_tay = instruction == 8'ha8;
    wire i_tsx = instruction == 8'hba;
    wire i_txa = instruction == 8'h8a;
    wire i_txs = instruction == 8'h9a;
    wire i_tya = instruction == 8'h98; 

    // misc
    wire i_sty = (instruction == 8'h84) || (instruction == 8'h94) || (instruction == 8'h8c);
    wire i_nop = (instruction == 8'hea);
    wire i_dex = instruction == 8'hca;
    wire i_dey = instruction == 8'h88;
    wire i_inx = instruction == 8'he8;
    wire i_iny = instruction == 8'hc8;

    wire i_brk = instruction == 0;
    wire i_jsr = instruction == 8'h20;
    wire i_rti = instruction == 8'h40;
    wire i_rts = instruction == 8'h60;

    wire stack_inst = isEven && !higher_half && (instruction[3:0] == 8);
    // stack
    wire i_php = stack_inst && (instruction[7:4] == 0);
    wire i_plp = stack_inst && (instruction[7:4] == 2);
    wire i_pha = stack_inst && (instruction[7:4] == 4);
    wire i_pla = stack_inst && (instruction[7:4] == 6);

    wire higher_half = instruction[7:4] >= 8;


    // modes
    wire immediate = (instruction == 8'ha2) || ((instruction[3:0] == 9) && isEven) || ((instruction[3:0] == 0) 
                       && higher_half && isEven);   // requires 1 operand
    wire absolute  = (((instruction[3:0] == 12) || (instruction[3:0] == 13) || (instruction[3:0] == 14)) 
                       && isEven) || (instruction == 8'h20) && !indirect;   // requires 2 operands
    wire zpg_absolute = ((instruction[3:0] == 4) || (instruction[3:0] == 5) || (instruction[3:0] == 6)) 
                       && isEven;   // requires 1 operand
    wire implied = (instruction == 8'h40) || (instruction == 8'h60) || (instruction == 0) || (instruction[3:0] == 8) || ((instruction[3:0] == 10) && higher_half); // 0 operands
    wire accumulator = (instruction[3:0] == 10) && !higher_half && isEven;   // 0 operands
    wire abs_indexed_x = ((instruction[3:0] == 12) || (instruction[3:0] == 13) || (instruction[3:0] == 14)) && isOdd && !abs_indexed_y; // 2 operands
    wire abs_indexed_y = ((instruction[3:0] == 14) && (instruction[7:4] == 11)) || ((instruction[3:0] == 9) && isOdd); // 2 operands
    wire zpg_indexed_x = ((instruction[3:0] == 4) || (instruction[3:0] == 5) || (instruction[3:0] == 6)) && isOdd && !zpg_indexed_y; // 1 operand
    wire zpg_indexed_y = ((instruction[3:0] == 6) && ((instruction[7:4] == 9) || (instruction[7:4] == 11))); // 1 operand
    wire indirect = (instruction[3:0] == 12) && (instruction[7:4] == 6);      // 2 operands
    wire indirect_x = (instruction[3:0] == 1) && isEven;    // 1 operand
    wire indirect_y = (instruction[3:0] == 1) && isOdd;    // 1 operand
    wire relative = (instruction[3:0] == 0) && isOdd;      // 1 operand

    // for debugging 
    modePrinter mp(clk, immediate, absolute, zpg_absolute, implied, accumulator, abs_indexed_x, abs_indexed_y, zpg_indexed_x, zpg_indexed_y, indirect, indirect_x, indirect_y, relative, instruction);


    // registers
    reg [15:0]pc = 0;
    reg [7:0] ac = 0;
    reg [7:0] x  = 0;
    reg [7:0] y  = 0;
    reg [7:0] sr = 0;
    reg [7:0] sp = 0;

    // status bits
    wire negative  = sr[7];
    wire overflow  = sr[6];
    wire ignored   = sr[5];
    wire break     = sr[4];
    wire decimal   = sr[3];
    wire interrupt = sr[2];
    wire zero      = sr[1];
    wire carry     = sr[0];

    always @(posedge clk) begin
        pc <= pc + 1;
        cycles <= cycles + 1;
        if (!(instruction === 8'hxx))begin
            //$display("instruction %x %d", instruction, isOdd);
            if (instruction == 8'hff)
                $finish;
        end 
    end

endmodule
