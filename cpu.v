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
`define X0 7
`define M1 8
`define M2 9

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
    reg cycles_v = 0;

    reg [15:0] cycles = 0;

    // ------- Instruction decoding -------- // 
    wire isOdd = inst[7:4] % 2 == 1;    // high bit is odd
    wire isEven = inst[7:4] % 2 == 0;   // high bit is even
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
    wire branch = ((inst[3:0] == 0) && isOdd);
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
    wire i_lda = (oddops && (high(inst, `A) || high(inst, `B)));
    wire i_cmp = oddops && (high(inst, `C) || high(inst, `D));
    wire i_sbc = oddops && (high(inst, `E) || high(inst, `F));

    // 6, 10 (lower half high), 14 low-nibble (evenops)
    wire i_asl = evenops && (high(inst, 0) || high(inst, 1));
    wire i_rol = evenops && (high(inst, 2) || high(inst, 3));
    wire i_lsr = evenops && (high(inst, 4) || high(inst, 5));
    wire i_ror = evenops && (high(inst, 6) || high(inst, 7));
    wire i_stx = evenops && (high(inst, 8) || high(inst, 9));
    wire i_ldx = (evenops && (high(inst, `A) || high(inst, `B))) || (inst == 8'ha2);
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
    wire immediate = (inst == 8'ha2) || (low(inst, 9) && isEven) || (low(inst, 0) && higher_half && isEven);            // 1 operand
    wire absolute  = (((low(inst, `C) || low(inst, `D) || low(inst, `E)) && isEven) || (inst == 8'h20)) && !indirect;     // 2 operands
    wire zpg_absolute = (low(inst, 4) || low(inst, 5) || low(inst, 6)) && isEven;                                       // 1 operand
    wire implied = (inst == 8'h40) || (inst == 8'h60) || (inst == 0) || low(inst, 8) || (low(inst, `A) && higher_half); // 0 operands
    wire accumulator = low(inst, `A) && !higher_half && isEven;                                                         // 0 operands
    wire abs_indexed_x = (low(inst, `C) || low(inst, `D) || low(inst, `E)) && isOdd && !abs_indexed_y;                  // 2 operands
    wire abs_indexed_y = (inst == 8'hbe) || (low(inst, 9) && isOdd);                                                    // 2 operands
    wire zpg_indexed_x = (low(inst, 4) || low(inst, 5) || low(inst, 6)) && isOdd && !zpg_indexed_y;                     // 1 operand
    wire zpg_indexed_y = (low(inst, 6) && (high(inst, 9) || high(inst, `B)));                                           // 1 operand
    wire indirect = inst == 8'h6c;                                                                                      // 2 operands
    wire indirect_x = low(inst, 1) && isEven;                                                                           // 1 operand
    wire indirect_y = low(inst, 1) && isOdd;                                                                            // 1 operand
    wire relative = low(inst, 0) && isOdd;                                                                              // 1 operand

    // --------------------------------------- //

    // ------------- REGISTERS -------------- //
    reg [7:0] registers[3:0];
    reg [15:0]pc = 0;
    reg [7:0] sr = 8'b00110000;
    // -------------------------------------- //
    
    //----------- REGSITER UPDATE -----------------//
    wire modify_ac = i_lda | i_adc | i_and | i_asl | i_eor | i_lsr | 
                     i_ora | i_rol | i_ror | i_sbc | i_txa | i_tya | i_pla;
    wire modify_x  = i_tax | i_inx | i_tsx | i_dex | i_ldx;
    wire modify_y  = i_dey | i_iny | i_ldy | i_tay;
    wire modify_sp = i_txs;
    wire modify_sr = i_plp;

    wire [1:0]modified_reg = (state == `F2) ? `SP :
                             modify_ac ? `AC :
                             modify_x  ? `X  :
                             modify_y  ? `Y  :
                             modify_sp ? `SP :
                             modify_sr ? 5   :
                             2'bxx;       

    wire register_ren = (i_pha | i_php) ? state == `F2 : 
                        (i_pla | i_plp) ? (state == `F2 | state == `F0) : state == `F0;

    wire ready = ((state == `WB) & (i_pha | i_php)) | ((state == `M0) & (i_pla | i_plp)) | 
                 ((state == `WB) & i_jsr) |
                 ((state == `M0) & i_rts) |
                 ((modify_ac | modify_x | modify_y | modify_sp | modify_sr) & register_ren);
    wire [7:0] newvalue = (i_pla | i_plp) ? temp_stack : alu_result;
    reg [7:0] temp_stack;


    always @(posedge clk) begin
        // only change one register at a time 
        if (ready & (state == `F0)) begin
            if (alu_carry)begin
                if (verbose)
                    $display("carrying!");
                sr[`CARRY] <= 1;
            end
            if (alu_overflow)begin
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
            if (modified_reg != 4) begin
                registers[modified_reg] <= newvalue;
                $display("#regs[%d] <= %x", modified_reg, newvalue);
            end
            else begin
                sr <= newvalue;
                $display("#regs[4] <= %b", modified_reg, newvalue);
            end
        end
        else if (ready) begin
            registers[`SP] <= alu_result;
            $display("#regs[3] <= %x", alu_result);
        end
    end
    // ------------------------------------------- //

    // ------------- ALU --------------- //

    wire [4:0] op = 
                    (state == `F1) && (zpg_indexed_x | zpg_indexed_y | abs_indexed_x | abs_indexed_y) ? `ADD :  
                    (state == `F1) && (indirect_y) ? `INC :
                    (state == `F2) && (indirect_y) ? `ADD :
                    (state == `F2) && (abs_indexed_x | abs_indexed_y) & alu_carry ? `INC :  
                    (state == `F1) && indirect_x ? `ADD :
                    (state == `F2) & (indirect_x) ?  `INC :
                    (i_cmp | i_cpx | i_cpy) ? `SUB :
                    i_pha | i_php | i_jsr ? `DEC :
                    i_pla | i_plp | i_rts ? `INC :
                    i_adc ? `ADD :
                    i_sbc ? `SUB : 
                    i_bit | i_and ? `AND :
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

    wire[7:0] ac = registers[`AC];
    wire[7:0] x = registers[`X];
    wire [7:0] y = registers[`Y];

    wire [7:0] operand1 = (state == `M0) & i_jsr ? registers[`SP] :
                          (state == `X0) & i_jsr ? alu_result :
                          (state == `F1) & i_rts ? registers[`SP] : 
                          (state == `F2) & i_rts ? alu_result :
                          (state == `F1) & (zpg_indexed_x | zpg_indexed_y | abs_indexed_x | abs_indexed_y | indirect_x | indirect_y) ? memOut :
                          (state == `F2) & (indirect_y) ? memOut : 
                          (state == `F2) & (abs_indexed_x | abs_indexed_y) & alu_carry ? memOut :
                          (state == `F2) & (indirect_x) ? alu_result :
                          (state == `M0) & (indirect_x) ? alu_result :
                          i_bit ? memOut :
                          (i_cmp | i_cpx | i_cpy) ? registers[`X] :
                          i_pha | i_php | i_pla | i_plp ? registers[`SP] :
                          (state == `M0) && (i_inc | i_dec | i_asl | i_lsr | i_rol | i_ror) ? memOut :
                           i_adc | i_sbc | i_and | i_ora | i_eor | 
                          ((i_lsr | i_asl | i_ror | i_rol) & accumulator) 
                           | i_tax | i_tay ? registers[`AC] : 
                          i_inx | i_dex | i_txa | i_txs ? registers[`X] : 
                          i_iny | i_dey | i_tya ? registers[`Y] : 
                          i_tsx ? registers[`SP] : 
                          (i_lda | i_ldx | i_ldy) ? memOut : 8'hxx;

    wire [7:0] operand2 = (i_bit) ? registers[`AC] :
                          (i_cmp | i_cpx | i_cpy) ? memOut :
                          (zpg_indexed_x | abs_indexed_x | indirect_x) & (state == `F1) ? registers[`X] : 
                          (zpg_indexed_y | abs_indexed_y) & (state == `F1) ? registers[`Y] :
                          (state == `F2) & indirect_y ? registers[`Y] :
                          (state == `X0) & (zpg_indexed_y | zpg_indexed_x | abs_indexed_y | abs_indexed_x) ? memOut : 
                          (i_adc | i_sbc | i_and | i_ora | i_eor) ? memOut : 8'hxx;
                          
    wire [7:0] alu_result;
    wire alu_carry;
    wire alu_overflow;

    alu a0(clk, operand1, operand2, sr[`CARRY], op, alu_result, alu_carry, alu_overflow);
    // --------------------------------- //


    // --------------------------- MEMORY --------------------------- // 
    
    wire [15:0] pc_value = memOut[7] == 1 ? pc - corrected_value : 
                           memOut + pc; 
    wire [7:0] corrected_value = ~memOut + 1;

    reg [7:0] temp_low = 8'hx;
    wire [7:0]memOut;
    wire [15:0]memIn = (state == `F2) & i_rts ? {8'h01, alu_result} :
                       (state == `M0) & i_rts ? {8'h01, alu_result} :
                       relative & branch_taken & (state == `F1) ? pc_value :
                       (state == `F1) & (indirect_y) ? {8'h00, memOut} : 
                       (state == `F2) & (indirect_y) ? {8'h00, alu_result} :
                       (state == `M0) & (indirect_y) ? {memOut, alu_result} :
                       (state == `M0) & (i_pla | i_plp) ? {8'h01, alu_result} :
                       absolute & reading & (state == `F2) ? {memOut, temp_low} :
                       (abs_indexed_x | abs_indexed_y) & 
                       reading & (state == `F2) ? {memOut, alu_result} : 
                       (zpg_indexed_x | zpg_indexed_y | indirect_x) & 
                       reading & (state == `F2) ? {8'h00, alu_result} :
                       (indirect_x) & (state == `M0) ? {8'h00, alu_result} : 
                       indirect_x & (state == `M1) ? {memOut, temp_low} : 
                       zpg_absolute & reading & (state == `F1) ? {8'h00, memOut} : 
                       indirect & (state == `F2) ? {memOut, temp_low} : 
                       indirect & (state == `M0) ? incremented_address : 
                       {8'h00, pc}; 
    
    reg write_enable = 1'bx;
    reg [7:0]res = 8'bxx;
    reg [15:0]waddr = 8'bxx;

    mem i0(clk,memIn,memOut, write_enable, waddr, res);

    // --------------------------------------------------------------- //

    reg [15:0] state = `INIT;
    reg [7:0]inst = 8'hxx;
    reg [15:0] addresscache;

    wire reading = !store;
    wire writeback = i_asl | i_dec | i_inc | i_lsr | i_rol | i_ror; // read, modify, write
    wire store   = i_sta | i_stx | i_sty;
    
    // status bits
    wire negative  = sr[7];
    wire overflow  = sr[6];
    wire ignored   = sr[5];
    wire break     = sr[4];
    wire decimal   = sr[3];
    wire interrupt = sr[2];
    wire zero      = sr[1];
    wire carry     = sr[0];

    wire branch_taken = i_bcc ? !carry : 
                        i_bcs ? carry  :
                        i_beq ? zero   :
                        i_bmi ? negative :
                        i_bne ? !zero  :
                        i_bpl ? !negative :
                        i_bvc ? !overflow :
                        i_bvs ? overflow  : 0;
    
    wire [7:0]operand = memOut;
    
    wire crossed = 0; // if a page boundary is crossed

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

    reg [15:0] incremented_address = 16'hxx;


    always @(posedge clk) begin
        case (state)
            `INIT: begin
                pc <= pc + 1;
                registers[0] = 0;
                registers[1] = 0;
                registers[2] = 0;
                registers[3] = 8'hff;
                state <= `F0;
            end
            `F0: begin 
                // ----- fetch opcode + finish last instruction ----- // 
                // memOut = instruction
                write_enable <= 0;
                if (verbose)
                    $display("F0 %x %x", memOut, inst);
                if (cycles_v && !(inst === 8'hxx))
                    $display("instruction %x completed in %d cycles", inst, cycles);
                
                cycles <= 1; // reset cycles
                
                if (memOut === 8'hxx)
                    state <= `HLT;
                else 
                    state <= `F1;
                
                // ----- STATUS REG UPDATES ---------- //
                if (i_bit)
                    sr[`ZERO] <= (alu_result == 0);
                
                if (i_clc) 
                    sr[`CARRY] <= 0;
                if (i_cld)
                    sr[`DECIMAL] <= 0;
                if (i_cli)
                    sr[`INT] <= 0;
                if (i_clv)
                    sr[`OVERFLOW] <= 0;
                if (i_sec)
                    sr[`CARRY] <= 1;
                if (i_sed)
                    sr[`DECIMAL] <= 1;
                if (i_sei)
                    sr[`INT] <= 1;

                if (i_cmp | i_cpx | i_cpy) begin
                    sr[`NEGATIVE] <= alu_result[7];
                    sr[`CARRY] <= (alu_result >= 0);
                    sr[`ZERO] <= alu_result == 0;
                end
                // --------------------------------- //
                
                if (!(inst === 8'hxx) & (accumulator | implied) & !branch_taken & !i_rts) begin
                    inst <= addresscache; 
                    pc <= pc + 1;
                end
                else begin
                    pc <= pc + 1;
                    inst <= memOut;
                end
            end
            `F1 : begin
                // ----------------- decode --------------------- //
                if (verbose)
                    $display("F1 %x %x", memOut, inst);
                if (memOut === 8'hxx | i_brk)
                    state <= `HLT;
                if (status_v)
                    $display("sr %b", sr);
                if (relative)begin
                    if (branch_taken) begin
                        pc <= pc_value + 1;
                    end
                   else 
                        pc <= pc + 1;
                   state <= `F0;
                end 
                else if (accumulator | implied) begin
                    // memOut is discarded & re-read: save the address 
                    addresscache <= memOut;
                    if (i_pha | i_php)begin
                        write_enable <= 1;
                        waddr <= {8'h01, registers[`SP]};
                        if (i_pha) 
                            res <= registers[`AC];
                        else 
                            res <= sr;
                        state <= `WB;
                    end
                    else if (i_pla | i_plp)
                        state <= `M0;
                    else if (i_jsr | i_rts) 
                        state <= `F2;
                    else
                        state <= `F0;
                end 
                else if (immediate) begin
                    // memOut is only operand - no further reads needed 
                    state <= `F0;
                    pc = pc + 1;
                end
                else if (store) begin
                    // sta, stx, sty
                    if (zpg_absolute) begin
                        state <= `WB;
                        write_enable <= 1;
                        waddr <= {8'h00, memOut};
                        res <= write_value;
                    end
                    else begin
                        if (!(zpg_indexed_x | zpg_indexed_y))
                            pc <= pc + 1;
                        temp_low <= memOut;
                        state <= `F2;
                    end
                end 
                else begin
                    if (zpg_absolute) begin
                        if (!writeback) begin
                            state <= `X0;
                        end
                        else begin
                            addresscache <= memIn;
                            state <= `M0;
                        end
                    end 
                    else begin
                        pc <= pc + 1;
                        temp_low <= memOut;
                        state <= `F2;
                    end 
                end 
                cycles <= cycles + 1;
            end
            `F2 : begin
                // ------------- reading second operand ------------ //
               if (verbose)
                    $display("F2 %x %x", memOut, inst);

               if (store) begin// lda, ldx, ldy done after this step if absolute, zpg_indexed
                    if (!(indirect_x | indirect_y | indirect))begin
                        write_enable <= 1;
                        res <= write_value;
                        if (!(zpg_indexed_x | zpg_indexed_y))
                            waddr <= {memOut, temp_low};
                        else
                            waddr <= {8'h00, alu_result}; // no overflow possible for zpg indexed
                        state <= `WB;
                    end else begin
                        state <= `M0;
                    end
                end
                else if (i_jmp & absolute)begin
                    pc <= {memOut, temp_low} + 1;
                    state <= `F0;
                end
                else if (i_rts) begin
                    state <= `M0;
                end
                else begin  
                    if (zpg_indexed_x | zpg_indexed_y | i_bit)begin
                        // alu_result has needed memIn address
                        state <= `X0;
                    end
                    else begin
                        addresscache <= {memOut, temp_low};
                        incremented_address <= {memOut, temp_low} + 1;
                        state <= `M0;
                    end
                
                end
                cycles <= cycles + 1;
            end
            `M0 : begin
                // -------------- reading a value from memory ------------ // 
                if (verbose)
                    $display("M0 %x %x", memOut, inst);
                if (store)begin
                    state <= `WB;
                end
                else if (i_jsr)begin
                    state <= `X0;
                    res <= pc[7:0];
                    waddr <= {8'h01, registers[`SP]};
                    write_enable <= 1;
                end
                else if (i_rts) begin
                    temp_low <= memOut;
                    state <= `X0;
                end
                else if (writeback) begin // may have to move this
                    state <= `X0;
                end
                else if (indirect)begin
                    temp_low <= memOut;
                    state <= `M1;
                end 
                else if (indirect_x) begin
                    state <= `M1;
                    temp_low <= memOut;
                end
                else if (indirect_y) begin
                    if (store) 
                        state <= `WB;
                    else 
                        state <= `X0;
                end
                else if (implied) begin
                    // alu_result has correct stack address
                    state <= `X0;
                end 
                else begin 
                    pc <= pc + 1;
                    state <= `F0;
                end
                cycles <= cycles + 1;
            end
            `M1 : begin
                // ------------- second byte of indirect ----------------- //
                if (verbose)    
                    $display("M1 %x %x", memOut, inst);
                if (i_jmp) begin
                    pc <= {memOut, temp_low} + 1;
                    state <= `F0;
                end
                else if (indirect_x) begin
                    state <= `X0;
                end
                else begin
                    state <= `F0;
                end
                cycles <= cycles + 1;
            end
            `X0 : begin
                // --------------- using the ALU for writeback --------------- //
                 if (verbose)
                    $display("X0 %x %x", memOut, inst); 
                 if (i_bit | i_cmp | i_cpx | i_cpy) begin
                    state <= `F0;
                 end if (zpg_absolute & !writeback)begin
                    state <= `F0;
                    pc <= pc + 1;
                 end
                 else if (i_jsr)begin
                    state <= `WB;
                    res <= pc[15:8];
                    waddr <= {8'h01, alu_result};
                    pc <= addresscache;
                 end
                 else if (i_rts) begin
                    state <= `WB;
                    pc <= {temp_low, memOut};
                 end
                 else if ((zpg_indexed_y | zpg_indexed_x) & store) begin
                    state <= `WB;
                    res <= write_value;
                    write_enable <= 1;
                    waddr <= {8'h00, alu_result};
                 end
                 else if (zpg_indexed_y | zpg_indexed_x) begin
                    state <= `F0;
                 end
                 else if (indirect_x | indirect_y) begin
                    state <= `F0;
                 end
                 else if (implied) begin
                    state <= `F0;
                    temp_stack <= memOut;   
                 end 
                 else begin
                    state <= `WB;
                    res <= alu_result;
                    write_enable <= 1;
                    waddr <= addresscache;
                 end
                // ------ update status register ----- // 
                if (writeback & i_rol)
                    sr[`CARRY] <= alu_result[7];
                if (writeback & i_ror)
                    sr[`CARRY] <= alu_result[0];
                if (writeback & alu_result == 0)
                    sr[`ZERO] <= 1;
                else 
                    sr[`ZERO] <= 0;
                if (writeback & alu_result[7] == 1)
                    sr[`NEGATIVE] <= 1;
                else
                    sr[`NEGATIVE] <= 0;
                
                if (i_bit) begin
                    sr[7] <= memOut[7];
                    sr[6] <= memOut[6];
                end

                cycles <= cycles + 1;
           end
          `WB : begin
                // -------------- write back to memory ------------------- //
                write_enable <= 0;
                if (verbose)
                    $display("WB %x %x", memOut, inst);
                if (!implied | i_rts)
                    pc <= pc + 1;
                state <= `F0;
                cycles <= cycles + 1;
           end 
            `HLT : begin
                if (verbose)
                    $display("HLT %x %x", memOut, inst);
                if (status_v)
                    $display("sr %b", sr);
                state <= `HLT2;
            end
            `HLT2 : begin
                $finish;
            end
        endcase
        
    end

endmodule
