; testing absolute x and y indexed
Address  Hexdump   Dissassembly
-------------------------------
$0600    a9 30     LDA #$30
$0602    8d 04 10  STA $1004
$0605    a9 40     LDA #$40
$0607    8d 05 10  STA $1005
$060a    a2 04     LDX #$04
$060c    a0 05     LDY #$05
$060e    bd 00 10  LDA $1000,X
$0611    79 00 10  ADC $1000,Y
