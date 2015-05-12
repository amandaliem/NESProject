Address  Hexdump   Dissassembly
-------------------------------
$0600    a9 03     LDA #$03
$0602    8d 00 01  STA $0100
$0605    0a        ASL A
$0606    2d 00 01  AND $0100
$0609    4d 00 01  EOR $0100
$060c    4a        LSR A
$060d    0d 00 01  ORA $0100
$0610    a9 01     LDA #$01
$0612    2a        ROL A
$0613    6a        ROR A
$0614    ed 00 01  SBC $0100
$0617    00        BRK 
