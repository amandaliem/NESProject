; subroutines!
Address  Hexdump   Dissassembly
-------------------------------
$0600    20 09 00  JSR $0009
$0603    20 0c 00  JSR $000c
$0606    20 12 00  JSR $0012
$0609    a2 00     LDX #$00
$060b    60        RTS 
$060c    e8        INX 
$060d    e0 05     CPX #$05
$060f    d0 fb     BNE $000c
$0611    60        RTS 
$0612    00        BRK
