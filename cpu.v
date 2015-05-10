`timescale 1ps/1ps

// STATES //
`define F0 0
`define F1 1 

// STATUS REGISTER BITS//
`define NEGATIVE 7
`define OVERFLOW 6  // 5 is unused
`define BREAK    4
`define DECIMAL  3
`define INT      2
`define ZERO     1
`define CARRY    0

// HEX //
`define A 10
`define B 11
`define C 12
`define D 13
`define E 14
`define F 15

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
    wire [7:0]inst = memOut;
    wire [7:0]instruction = memOut;

    // instruction decoding // 
    wire isOdd = inst[7:4] % 2 == 1;
    wire isEven = inst[7:4] % 2 == 0;
    wire higher_half = instruction[7:4] >= 8;
    
    // instruction groups 
    wire oddops = low(inst, 1) || low(inst, 5) ||
                  low(inst, 9) || low(inst, `D);
    wire evenops = low(inst, 6) || (low(inst, 10) && !higher_half) || 
                   low(inst, 14);
    wire zerofourc = low(inst, 0) || low(inst, 4) ||
                     low(inst, `C);

    // instructions are split into high and low nibbles
    function high;
    input [7:0] inst;
    input [3:0] code;
    high = inst[7:4] == code;
    endfunction

    function low;
    input [7:0] inst;
    input [3:0] code;
    low = inst[3:0] == code;
    endfunction

    // 0 low-nibble (!higher_half) 
    wire i_brk = instruction == 0;
    wire i_jsr = instruction == 8'h20;
    wire i_rti = instruction == 8'h40;
    wire i_rts = instruction == 8'h60;
   
    // branches 0 low-nibble (odd high-nibble)
    wire branch = ((instruction[3:0] == 0) && isOdd) || i_jmp;
    wire i_bpl = branch && high(inst, 1);
    wire i_bmi = branch && high(inst, 3);
    wire i_bvc = branch && high(inst, 5);
    wire i_bvs = branch && high(inst, 7);
    wire i_bcc = branch && high(inst, 9);
    wire i_bcs = branch && high(inst, `B);
    wire i_bne = branch && high(inst, `D);
    wire i_beq = branch && high(inst, `F);
    wire i_bit = (instruction == 8'h24) || (instruction == 8'h2c);
    wire i_jmp = (instruction == 8'h4c) || (instruction == 8'h6c);

    // 0, 4, c low-nibble (zerofourc)
    wire i_cpx = zerofourc && high(inst, `E);
    wire i_cpy = zerofourc && high(inst, `C);
    wire i_ldy = zerofourc && !(instruction == 8'hb0) && (high(inst, 10) || high(inst, 11));
    
    // 1, 5, 9, D low-nibble (oddops)
    wire i_ora = oddops && (high(inst, 0) || high(inst, 1));
    wire i_and = oddops && (high(inst, 2) || high(inst, 3));
    wire i_eor = oddops && (high(inst, 4) || high(inst, 5));
    wire i_adc = oddops && (high(inst, 6) || high(inst, 7));
    wire i_sta = oddops && (high(inst, 8) || high(inst, 9));
    wire i_lda = (oddops && (high(inst, `A) || high(inst, `B))) || (instruction == 8'ha2);
    wire i_cmp = oddops && (high(inst, `C) || high(inst, `D));
    wire i_sbc = oddops && (high(inst, `E) || high(inst, `F));

    // 6, 10 (lower half high), 14 low-nibble (evenops)
    wire i_asl = evenops && (high(inst, 0) || high(inst, 1));
    wire i_rol = evenops && (high(inst, 2) || high(inst, 3));
    wire i_lsr = evenops && (high(inst, 4) || high(inst, 5));
    wire i_ror = evenops && (high(inst, 6) || high(inst, 7));
    wire i_stx = evenops && (high(inst, 8) || high(inst, 9));
    wire i_ldx = evenops && (high(inst, `A) || high(inst, `B));
    wire i_dec = evenops && (high(inst, `C) || high(inst, `D));
    wire i_inc = evenops && (high(inst, `E) || high(inst, `F));

    // flags 8 low-nibble & odd high-nibble (tya exception)
    wire flags = isOdd && low(inst, 8) && !i_tya;
    wire i_clc = flags && high(inst, 1);
    wire i_sec = flags && high(inst, 3);
    wire i_cli = flags && high(inst, 5);
    wire i_sei = flags && high(inst, 7);
    wire i_clv = flags && high(inst, 11);
    wire i_cld = flags && high(inst, 13);
    wire i_sed = flags && high(inst, 15);

    // stack 8 low-nibble & even high-nibble & !higher_half
    wire stack_inst = isEven && !higher_half && (instruction[3:0] == 8);
    wire i_php = stack_inst && (instruction[7:4] == 0);
    wire i_plp = stack_inst && (instruction[7:4] == 2);
    wire i_pha = stack_inst && (instruction[7:4] == 4);
    wire i_pla = stack_inst && (instruction[7:4] == 6);

    // transfer (no pattern...?)
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
    

    // modes
    wire immediate = (instruction == 8'ha2) || 
                     ((instruction[3:0] == 9) && isEven) || 
                     ((instruction[3:0] == 0) && higher_half && isEven);   // requires 1 operand
    wire absolute  = (((instruction[3:0] == 12) || (instruction[3:0] == 13) || (instruction[3:0] == 14)) && isEven) || 
                     (instruction == 8'h20) && 
                     !indirect;   // requires 2 operands
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
    //modePrinter mp(clk, immediate, absolute, zpg_absolute, implied, accumulator, abs_indexed_x, abs_indexed_y, zpg_indexed_x, zpg_indexed_y, indirect, indirect_x, indirect_y, relative, instruction);

    instprinter ip(clk, i_adc, i_and, i_asl, i_bcc, i_bcs, i_beq, i_bit, i_bmi, i_bne, i_bpl, i_brk, i_bvc, i_bvs, i_clc, i_cld, i_cli, i_clv, i_cmp, i_cpx, i_cpy, i_dec, i_dex, i_dey, i_eor, i_inc, i_inx, i_iny, i_jmp, i_jsr, i_lda, i_ldx, i_ldy, i_lsr, i_nop, i_ora, i_pha, i_php, i_pla, i_plp, i_rol, i_ror, i_rti, i_rts, i_sbc, i_sec, i_sed, i_sei, i_sta, i_stx, i_sty, i_tax, i_tay, i_tsx, i_txa, i_txs, i_tya, inst);

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
