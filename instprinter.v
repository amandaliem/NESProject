module instprinter(input clk, input i_adc, input i_and, input i_asl, input i_bcc, input i_bcs, input i_beq, input i_bit, input i_bmi, input i_bne, input i_bpl, input i_brk, input i_bvc, input i_bvs, input i_clc, input i_cld, input i_cli, input i_clv, input i_cmp, input i_cpx, input i_cpy, input i_dec, input i_dex, input i_dey, input i_eor, input i_inc, input i_inx, input i_iny, input i_jmp, input i_jsr, input i_lda, input i_ldx, input i_ldy, input i_lsr, input i_nop, input i_ora, input i_pha, input i_php, input i_pla, input i_plp, input i_rol, input i_ror, input i_rti, input i_rts, input i_sbc, input i_sec, input i_sed, input i_sei, input i_sta, input i_stx , input i_sty, input i_tax, input i_tay, input i_tsx, input i_txa, input i_txs, input i_tya, input [7:0] inst);

wire unknown = !i_adc & !i_and & !i_asl & !i_bcc & !i_bcs & !i_beq & !i_bit & !i_bmi & !i_bne & !i_bpl & !i_brk & !i_bvc & !i_bvs & !i_clc & !i_cld & !i_cli & !i_clv & !i_cmp & !i_cpx & !i_cpy & !i_dec & !i_dex & !i_dey & !i_eor & !i_inc & !i_inx & !i_iny & !i_jmp & !i_jsr & !i_lda & !i_ldx & !i_ldy & !i_lsr & !i_nop & !i_ora & !i_pha & !i_php & !i_pla & !i_plp & !i_rol & !i_ror & !i_rti & !i_rts & !i_sbc & !i_sec & !i_sed & !i_sei & !i_sta & !i_stx & !i_sty & !i_tax & !i_tay & !i_tsx & !i_txa & !i_txs & !i_tya;

always @(posedge clk)begin
if (i_adc)
	$display("i_adc is inst %X", inst);
if (i_and)
	$display("i_and is inst %X", inst);
if (i_asl)
	$display("i_asl is inst %X", inst);
if (i_bcc)
	$display("i_bcc is inst %X", inst);
if (i_bcs)
	$display("i_bcs is inst %X", inst);
if (i_beq)
	$display("i_beq is inst %X", inst);
if (i_bit)
	$display("i_bit is inst %X", inst);
if (i_bmi)
	$display("i_bmi is inst %X", inst);
if (i_bne)
	$display("i_bne is inst %X", inst);
if (i_bpl)
	$display("i_bpl is inst %X", inst);
if (i_brk)
	$display("i_brk is inst %X", inst);
if (i_bvc)
	$display("i_bvc is inst %X", inst);
if (i_bvs)
	$display("i_bvs is inst %X", inst);
if (i_clc)
	$display("i_clc is inst %X", inst);
if (i_cld)
	$display("i_cld is inst %X", inst);
if (i_cli)
	$display("i_cli is inst %X", inst);
if (i_clv)
	$display("i_clv is inst %X", inst);
if (i_cmp)
	$display("i_cmp is inst %X", inst);
if (i_cpx)
	$display("i_cpx is inst %X", inst);
if (i_cpy)
	$display("i_cpy is inst %X", inst);
if (i_dec)
	$display("i_dec is inst %X", inst);
if (i_dex)
	$display("i_dex is inst %X", inst);
if (i_dey)
	$display("i_dey is inst %X", inst);
if (i_eor)
	$display("i_eor is inst %X", inst);
if (i_inc)
	$display("i_inc is inst %X", inst);
if (i_inx)
	$display("i_inx is inst %X", inst);
if (i_iny)
	$display("i_iny is inst %X", inst);
if (i_jmp)
	$display("i_jmp is inst %X", inst);
if (i_jsr)
	$display("i_jsr is inst %X", inst);
if (i_lda)
	$display("i_lda is inst %X", inst);
if (i_ldx)
	$display("i_ldx is inst %X", inst);
if (i_ldy)
	$display("i_ldy is inst %X", inst);
if (i_lsr)
	$display("i_lsr is inst %X", inst);
if (i_nop)
	$display("i_nop is inst %X", inst);
if (i_ora)
	$display("i_ora is inst %X", inst);
if (i_pha)
	$display("i_pha is inst %X", inst);
if (i_php)
	$display("i_php is inst %X", inst);
if (i_pla)
	$display("i_pla is inst %X", inst);
if (i_plp)
	$display("i_plp is inst %X", inst);
if (i_rol)
	$display("i_rol is inst %X", inst);
if (i_ror)
	$display("i_ror is inst %X", inst);
if (i_rti)
	$display("i_rti is inst %X", inst);
if (i_rts)
	$display("i_rts is inst %X", inst);
if (i_sbc)
	$display("i_sbc is inst %X", inst);
if (i_sec)
	$display("i_sec is inst %X", inst);
if (i_sed)
	$display("i_sed is inst %X", inst);
if (i_sei)
	$display("i_sei is inst %X", inst);
if (i_sta)
	$display("i_sta is inst %X", inst);
if (i_stx)
	$display("i_stx is inst %X", inst);
if (i_sty)
	$display("i_sty is inst %X", inst);
if (i_tax)
	$display("i_tax is inst %X", inst);
if (i_tay)
	$display("i_tay is inst %X", inst);
if (i_tsx)
	$display("i_tsx is inst %X", inst);
if (i_txa)
	$display("i_txa is inst %X", inst);
if (i_txs)
	$display("i_txs is inst %X", inst);
if (i_tya)
	$display("i_tya is inst %X", inst);

if (unknown)
	$display("unknown inst %x", inst);

end
endmodule
