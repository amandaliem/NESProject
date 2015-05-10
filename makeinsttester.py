import sys

def caller():
    call = "instprinter ip("

    for line in sys.stdin:
        line = line.strip()
        line = line.lower()
        line = "i_" + line
        call += line + ","

    call = call[:-1]
    call += ");"
    print call

def inputs():
    header = "module instprinter(clk, \n"
    for line in sys.stdin:
        line = line.strip()
        line = line.lower()
        line = "i_" + line
        header += "input " + line + ", "
    header += "input [7:0] inst);"
    print header

def notwire():
    wire = "wire unknown = ";
    for line in sys.stdin:
        line = line.strip()
        line = line.lower()
        line = "i_"+ line
        wire += "!" + line + " & "
    wire = wire[:-3]
    wire += ";"
    print wire

def printer():
    for line in sys.stdin:
        line = line.strip()
        line = line.lower()
        line = "i_" + line
        ifstatement = "if (" + line + ")"
        printer = "\t$display(\"" + line + " is inst %X\", inst);";
        print ifstatement
        print printer



def main():
    #inputs()
    #caller()
    printer()    
    #notwire()
main()
