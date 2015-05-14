Address  Hexdump   Dissassembly
-------------------------------
$0600    a9 0d     LDA #$0d
$0602    85 f0     STA $f0
$0604    a9 06     LDA #$00
$0606    85 f1     STA $f1
$0608    6c f0 00  JMP ($00f0)
$060b    a9 01     LDA #$01
$060d    a9 02     LDA #$02
$060f    e8        INX 
