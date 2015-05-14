;testing branch taken
Address  Hexdump   Dissassembly
-------------------------------
$0600    a9 01     LDA #$01
$0602    69 01     ADC #$01
$0604    90 02     BCC $0608
$0606    a9 05     LDA #$05
$0608    a9 06     LDA #$06
