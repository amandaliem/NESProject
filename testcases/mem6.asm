; increment, decrement 

Address  Hexdump   Dissassembly CYCLES
--------------------------------------
$0600    a9 60     LDA #$60       2
$0602    85 50     STA $50        3
$0604    e6 50     INC $50        5
$0606    a6 50     LDX $50        3
$0608    e8        INX            2
$0609    c8        INY            2
$060a    e8        INX            2
$060b    e8        INX            2
$060c    ca        DEX            2
$060d    88        DEY            2
$060e    e6 50     INC $50        5
$0610    e6 50     INC $50        5
$0612    c6 50     DEC $50        5
