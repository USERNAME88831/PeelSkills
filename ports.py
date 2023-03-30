from pybricks.parameters import Port, Button


# Motor ports: 
A = Port.A
B = Port.B
C = Port.C
D = Port.D

# Sensor ports
S1 = Port.S1
S2 = Port.S2
S3 = Port.S3
S4 = Port.S4

lookUpTable = ["V", "U", "T", "S", "R", "Q", "P", "O", "N", "M", "L","K", "J", "I", "H", "G", "F", "E", "D", "C", "B","A"]

# Buttons 
CENTERBUTTON = Button.CENTER
RIGHT = Button.RIGHT

def lookUp(str):
    # CONVERTS PEEL SKILLS COORDINATE SYSTEM TO A BASIC XY COORDINATE SYSTEM
    # y starts at V, ends at A
    # x starts at 1 ends at 46

    x_1 = ""
    y_1 = ""
    for i in range(len(str)):
        if i == 0:
            y_1 = str[i]
        else:
            x_1 += str[i]
    y = 0
    for z in range(len(lookUpTable)):
        if y_1 == lookUpTable[z]:
            y = z
    x = int(x_1)


    

    
    return [x, y]