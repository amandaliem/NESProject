; absolute and zeropage addressing

Address  Hexdump   Dissassembly
-------------------------------
$0600    a9 30     LDA #$30
$0602    85 40     STA $40
$0604    8d 40 30  STA $3040
$0607    a6 40     LDX $40
$0609    ac 40 30  LDY $3040
$060c    e8        INX 
$060d    c8        INY 
$060e    c8        INY 
$060f    8e 00 10  STX $1000
$0612    84 50     STY $50
$0614    a5 50     LDA $50
$0616    ad 00 10  LDA $1000
