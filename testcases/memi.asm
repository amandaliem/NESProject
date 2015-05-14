; x zeropage indexing

Address  Hexdump   Dissassembly
-------------------------------
$0600    a2 01     LDX #$01
$0602    a9 aa     LDA #$aa
$0604    95 a0     STA $a0,X
$0606    e8        INX 
$0607    95 a0     STA $a0,X
