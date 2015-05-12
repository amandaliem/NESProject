`timescale 1ps/1ps

// STATES //
`define INIT 10
`define F0 0
`define F1 1 
`define HLT 2
`define F2 3
`define HLT2 4
`define M0 5
`define WB 6

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

// REGISTERS //
`define AC 0
`define X 1
`define Y 2
`define SP 3

// OPS // 
`define ADD 0
`define SUB 1
`define AND 2
`define OR  3
`define XOR 4
`define INC 5
`define DEC 6
`define SHR 7
`define SHL 8
`define RTR 9
`define RTL 10
`define LD  12

// RAM access time ~70ns?
// NES version of 6502 doesn't use BCD 

module cpu(input clk);

    // set to 1 for print statments
    reg verbose = 0;
    reg status_v = 0;
    reg [15:0] cycles = 0;


    reg [7:0] registers[3:0];
    //              REGISTER UPDATE             //
    wire modify_ac = i_lda | i_adc | i_and | i_asl | i_eor | i_lsr | i_ora | 
                     i_rol | i_ror | i_sbc | i_txa | i_tya;
    wire modify_x = i_tax | i_inx | i_tsx | i_dex | i_ldx;
    wire modify_y = i_dey | i_iny | i_ldy | i_tay;
    wire modify_sp = i_txs;

    wire [1:0]modified_reg = modify_ac ? `AC :
                             modify_x  ? `X  :
                             modify_y  ? `Y  :
                             modify_sp ? `SP :
                             2'bxx;       

    wire is_done = immediate | implied | accumulator ? (state == `F1) : 
                   absolute  ? (state == `M0) :
                   zpg_absolute ? (state == `M0) :
                   0;

    wire ready = (modify_ac | modify_x | modify_y | modify_sp) & is_done;

    wire [4:0] op = i_adc ? `ADD :
                     i_sbc ? `SUB : 
                     i_and ? `AND :
                     i_ora ? `OR  : 
                     i_eor ? `XOR : 
                     (i_inx | i_iny | i_inc) ? `INC :
                     (i_dex | i_dey | i_dec) ? `DEC :
                     i_lsr ? `SHR :
                     i_asl ? `SHL :
                     i_ror ? `RTR :
                     i_rol ? `RTL :
                     (i_lda | i_ldx | i_ldy | i_tax | 
                     i_tay | i_tsx | i_txa | i_txs | 
                     i_tya) ? `LD : 4'bx;
/*
    wire [7:0] newvalue = i_lda ? operand : 
                          i_adc && immediate ? registers[0] + operand + carry:
                          i_tax ? registers[0] : 
                          i_inx ? registers[1] + 1 : 8'hxx;
*/

    wire [7:0] operand1 = i_adc | i_sbc | i_and | i_ora | i_eor | 
                         ((i_lsr | i_asl | i_ror | i_rol) & accumulator) 
                         | i_tax | i_tay ? registers[`AC] : 
                          i_inx | i_dex | i_txa | i_txs ? registers[`X] : 
                          i_iny | i_dey | i_tya ? registers[`Y] : 
                          i_tsx ? registers[`SP] : 
                          (i_lda | i_ldx | i_ldy) ? memOut : 8'hxx;

    wire [7:0] operand2 = i_adc | i_sbc | i_and | i_ora | i_eor ? memOut : 8'hxx;
                          

    wire [7:0] newvalue = (op == `ADD) ? operand1 + operand2 + carry : 
                          (op == `SUB) ? operand1 - operand2 - carry :
                          (op == `AND) ? operand1 & operand2 :
                          (op == `OR)  ? operand1 | operand2 : 
                          (op == `XOR) ? operand1 ^ operand2 : 
                          (op == `INC) ? operand1 + 1 : 
                          (op == `DEC) ? operand1 - 1 :
                          (op == `SHR) ? operand1 >> 1 : 
                          (op == `SHL) ? operand1 << 1 :
                          (op == `RTR) ? (operand1 >> 1) & (carry << 7 | 8'h7F) :
                          (op == `RTL) ? (operand1 << 1) | (carry):
                          (op == `LD)  ? operand1 : 
                          8'hxx;

    reg [7:0] temp_low = 8'hx;

    // need to test overflow cases
    wire carrybit = (op == `ADD) ? 
                    ((operand1[7] == 1) && (operand2[7] == 1)) ||
                    ((operand1[7] != operand2[7]) && (newvalue[7] == 0)): 
                    (op == `SUB) ? (newvalue <= 0) : 0; // FIX THIS 

    wire overflowbit = i_adc && immediate ? 
                    ((registers[0][7] == 1) && (operand[7] == 1) && (newvalue[7] != 1) ||
                     (registers[0][7] == 0) && (operand[7] == 0) && (newvalue[7] != 0)) : 0; 

    always @(posedge clk) begin
        // only change one register at a time 
        if (ready) begin
            if (carrybit)begin
                if (verbose)
                    $display("carrying!");
                sr[`CARRY] <= 1;
            end
            if (overflowbit)begin
                if (verbose)
                    $display("overflowed!");
                sr[`OVERFLOW] <= 1;
            end
            if (newvalue[7] == 1)
                sr[`NEGATIVE] <= 1;
            else
                sr[`NEGATIVE] <= 0;
            if (newvalue == 0)
                sr[`ZERO] <= 1;
            else 
                sr[`ZERO] <= 0;
            if (i_rol | i_asl) begin
                sr[`CARRY] <= operand1[7];
            end
            if (i_ror | i_lsr)begin
                sr[`CARRY] <= operand1[0];
             end
            
            registers[modified_reg] <= newvalue;
            $display("#regs[%d] <= %x", modified_reg, newvalue);
        end
    end

    wire reading = (absolute | zpg_absolute) && !i_sta && !i_stx && !i_sty;

    // fetch 
    wire [7:0]memOut;
    wire [15:0]memIn = absolute & reading & (state == `F2) ? {memOut, temp_low} :
                       zpg_absolute & reading & (state == `F1) ? {8'h00, memOut} : 
                      {8'h00, pc}; 

    wire wen = zpg_absolute && !reading ? (state == `F1) : 
               writeback ? (state == `M0) : write_enable;

    reg write_enable = 1'bx;
    wire [15:0] write_address = absolute ? {memOut , temp_low} : 
                                zpg_absolute & !writeback ? {8'h00, memOut} : 
                                writeback & (state == `M0) ? addresscache : waddr;
    wire [7:0] reswire = zpg_absolute & !writeback ? write_value : 
                         writeback & (state == `M0) ? writeback_value : res;
    reg [7:0]res = 8'bxx;
    reg [15:0]waddr = 8'bxx;

    mem i0(clk,memIn,memOut, wen, write_address, reswire);

    reg [15:0] state = `INIT;
    reg [7:0]inst = 8'hxx;
    reg [15:0] addresscache;

    // inst decoding // 
    wire isOdd = inst[7:4] % 2 == 1;
    wire isEven = inst[7:4] % 2 == 0;
    wire higher_half = inst[7:4] >= 8;
    
    // inst groups 
    wire oddops = low(inst, 1) || low(inst, 5) ||
                  low(inst, 9) || low(inst, `D);
    wire evenops = low(inst, 6) || (low(inst, 10) && !higher_half) || 
                   low(inst, 14);
    wire zerofourc = low(inst, 0) || low(inst, 4) ||
                     low(inst, `C);

    // insts are split into high and low nibbles
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
    wire i_brk = inst == 0;
    wire i_jsr = inst == 8'h20;
    wire i_rti = inst == 8'h40;
    wire i_rts = inst == 8'h60;
   
    // branches 0 low-nibble (odd high-nibble)
    wire branch = ((inst[3:0] == 0) && isOdd) || i_jmp;
    wire i_bpl = branch && high(inst, 1);
    wire i_bmi = branch && high(inst, 3);
    wire i_bvc = branch && high(inst, 5);
    wire i_bvs = branch && high(inst, 7);
    wire i_bcc = branch && high(inst, 9);
    wire i_bcs = branch && high(inst, `B);
    wire i_bne = branch && high(inst, `D);
    wire i_beq = branch && high(inst, `F);
    wire i_bit = (inst == 8'h24) || (inst == 8'h2c);
    wire i_jmp = (inst == 8'h4c) || (inst == 8'h6c);

    // 0, 4, c low-nibble (zerofourc)
    wire i_cpx = zerofourc && high(inst, `E);
    wire i_cpy = zerofourc && high(inst, `C);
    wire i_ldy = zerofourc && !(inst == 8'hb0) && (high(inst, 10) || high(inst, 11));
    
    // 1, 5, 9, D low-nibble (oddops)
    wire i_ora = oddops && (high(inst, 0) || high(inst, 1));
    wire i_and = oddops && (high(inst, 2) || high(inst, 3));
    wire i_eor = oddops && (high(inst, 4) || high(inst, 5));
    wire i_adc = oddops && (high(inst, 6) || high(inst, 7));
    wire i_sta = oddops && (high(inst, 8) || high(inst, 9));
    wire i_lda = (oddops && (high(inst, `A) || high(inst, `B))) || (inst == 8'ha2);
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
    wire stack_inst = isEven && !higher_half && (inst[3:0] == 8);
    wire i_php = stack_inst && (inst[7:4] == 0);
    wire i_plp = stack_inst && (inst[7:4] == 2);
    wire i_pha = stack_inst && (inst[7:4] == 4);
    wire i_pla = stack_inst && (inst[7:4] == 6);

    // transfer (no pattern...?)
    wire i_tax = inst == 8'haa; 
    wire i_tay = inst == 8'ha8;
    wire i_tsx = inst == 8'hba;
    wire i_txa = inst == 8'h8a;
    wire i_txs = inst == 8'h9a;
    wire i_tya = inst == 8'h98; 

    // misc 
    wire i_sty = (inst == 8'h84) || (inst == 8'h94) || (inst == 8'h8c);
    wire i_nop = (inst == 8'hea);
    wire i_dex = inst == 8'hca;
    wire i_dey = inst == 8'h88;
    wire i_inx = inst == 8'he8;
    wire i_iny = inst == 8'hc8;

    // modes
    wire immediate = (inst == 8'ha2) || (low(inst, 9) && isEven) || (low(inst, 0) && higher_half && isEven);   // 1 operand
    wire absolute  = ((low(inst, `C) || low(inst, `D) || low(inst, `E)) && isEven) || (inst == 8'h20) && !indirect;   // requires 2 operands
    wire zpg_absolute = (low(inst, 4) || low(inst, 5) || low(inst, 6)) && isEven;   // requires 1 operand
    wire implied = (inst == 8'h40) || (inst == 8'h60) || (inst == 0) || low(inst, 8) || (low(inst, `A) && higher_half); // 0 operands
    wire accumulator = low(inst, `A) && !higher_half && isEven;   // 0 operands
    wire abs_indexed_x = (low(inst, `C) || low(inst, `D) || low(inst, `E)) && isOdd && !abs_indexed_y; // 2 operands
    wire abs_indexed_y = (inst == 8'hbe) || (low(inst, 9) && isOdd); // 2 operands
    wire zpg_indexed_x = (low(inst, 4) || low(inst, 5) || low(inst, 6)) && isOdd && !zpg_indexed_y; // 1 operand
    wire zpg_indexed_y = (low(inst, 6) && (high(inst, 9) || high(inst, `B))); // 1 operand
    wire indirect = inst == 8'h6c;      // 2 operands
    wire indirect_x = low(inst, 1) && isEven;    // 1 operand
    wire indirect_y = low(inst, 1) && isOdd;    // 1 operand
    wire relative = low(inst, 0) && isOdd;      // 1 operand

    reg newinst = 0;

    wire [7:0] write_value = i_sta ? registers[`AC] :
                             i_stx ? registers[`X]  :
                             i_sty ? registers[`Y]  : 
                             i_asl ? 0              :
                             i_dec ? 0              :
                             i_inc ? 0              :
                             i_lsr ? 0              :
                             i_rol ? 0              :
                             i_ror ? 0              :
                             8'hxx;

    // for debugging 
    //modePrinter mp(clk, immediate, absolute, zpg_absolute, implied, accumulator, abs_indexed_x, abs_indexed_y, zpg_indexed_x, zpg_indexed_y, indirect, indirect_x, indirect_y, relative, inst, newinst);

    //instprinter ip(clk, i_adc, i_and, i_asl, i_bcc, i_bcs, i_beq, i_bit, i_bmi, i_bne, i_bpl, i_brk, i_bvc, i_bvs, i_clc, i_cld, i_cli, i_clv, i_cmp, i_cpx, i_cpy, i_dec, i_dex, i_dey, i_eor, i_inc, i_inx, i_iny, i_jmp, i_jsr, i_lda, i_ldx, i_ldy, i_lsr, i_nop, i_ora, i_pha, i_php, i_pla, i_plp, i_rol, i_ror, i_rti, i_rts, i_sbc, i_sec, i_sed, i_sei, i_sta, i_stx, i_sty, i_tax, i_tay, i_tsx, i_txa, i_txs, i_tya, inst);

    // registers
    reg [15:0]pc = 0;
    reg [7:0] sr = 8'b00110000;

    wire writeback = i_asl | i_dec | i_inc | i_lsr | i_rol | i_ror;

    wire [7:0] writeback_value = i_asl ? (memOut << 1 ) :
                           i_dec ? (memOut - 1 )  : 
                           i_inc ? (memOut + 1 )  :
                           i_lsr ? (memOut >> 1 ) :
                           i_rol ? (memOut << 1) | (carry) : 
                           i_ror ? (memOut >> 1) & (carry << 7 | 8'h7F) : 8'hxx;
  

    /*
    reg [7:0] ac = 0;
    reg [7:0] x  = 0;
    reg [7:0] y  = 0;
    reg [7:0] sp = 0;
    */
    
    // status bits
    wire negative  = sr[7];
    wire overflow  = sr[6];
    wire ignored   = sr[5];
    wire break     = sr[4];
    wire decimal   = sr[3];
    wire interrupt = sr[2];
    wire zero      = sr[1];
    wire carry     = sr[0];

    wire [7:0]operand = memOut;
    
    wire crossed = 0; // if a page boundary is crossed

    // implied = 1 byte
    // absolute, absolute x, absolute y = 3 bytes 
    // else 2 bytes
    always @(posedge clk) begin
        case (state)
            `INIT: begin
                pc <= pc + 1;
                registers[0] = 0;
                registers[1] = 0;
                registers[2] = 0;
                registers[3] = 0;
                state <= `F0;
            end
            `F0: begin
                write_enable <= 0;
                if (verbose)
                    $display("F0 %x %x", memOut, inst);
                if (status_v)
                    $display("sr %b", sr);
                if (memOut == 0)
                    state <= `HLT;
                if (memOut === 8'hxx)
                    state <= `HLT;
                if (implied === 1'bx | !(implied | accumulator) )
                    state <= `F1;
                inst <= memOut;
                newinst <= 1;
                pc <= pc + 1;
            end
            `F1 : begin
                if (verbose)
                    $display("F1 %x %x", memOut, inst);
                if (memOut === 8'hxx)
                    state <= `HLT;

                if (immediate)
                    state <= `F0;
                if (!zpg_absolute) // || !reading)
                    pc <= pc + 1;
                if (absolute)begin
                    temp_low <= memOut;
                    if (!reading) 
                       write_enable <= 1;
                    state <= `F2;
                end
                if (zpg_absolute)begin 
                    addresscache <= memIn;
                    state <= `M0;
                end
                if (i_sta && absolute)
                    res <= registers[0];
                if (implied | accumulator)begin
                    inst <= memOut;
                end
            end
            `F2 : begin
               if (verbose)
                    $display("F2 %x %x", memOut, inst);
                if (memOut === 8'hxx)
                    state <= `HLT;
                if (crossed & !reading) begin
                    write_enable <= 1;
                end
                else begin
                    write_enable <= 0;
                    if (!reading)
                        state <= `F0;
                    else begin
                        addresscache <= {8'h00, memOut};
                        state <= `M0;
                    end
                end
                if (!absolute || !reading)
                    pc <= pc + 1;
            end
            `M0 : begin
                if (verbose)
                    $display("M0 %x %x", memOut, inst);
                if (memOut == 8'hxx)
                    state <= `HLT;
                else  
                    state <= `F0;
                if (writeback & i_rol)
                    sr[`CARRY] <= memOut[7];
                if (writeback & i_ror)
                    sr[`CARRY] <= memOut[0];
                if (writeback & writeback_value == 0)
                    sr[`ZERO] <= 1;
                else 
                    sr[`ZERO] <= 0;
                if (writeback & writeback_value[7] == 1)
                    sr[`NEGATIVE] <= 1;
                else
                    sr[`NEGATIVE] <= 0;
                pc <= pc + 1;
            end
          /*  `WB : begin
                if (verbose)
                    $display("WB %x %x", memOut, inst);
                pc <= pc + 1;
                state <= `F0;
            end */
            `HLT : begin
                state <= `HLT2;
            end
            `HLT2 : begin
                $finish;
            end
        endcase
        
        cycles <= cycles + 1;
    end

endmodule
