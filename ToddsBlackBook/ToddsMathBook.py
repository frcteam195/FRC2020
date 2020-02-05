import math
#Method One
Avgyro = 10
Aagyro = 10
dplus = 10
Avegyro = 10
Ave = Avgyro + (Aagyro * dplus * (1/2))
Ao = Ave * dplus

#Method Two
Wvl = 10
Wvr = 10
Wr = 5
Wal = 10
War = 10
Wvle = Wvl + (Wal * dplus * (1/2))
Wvre = Wvr + (War * dplus * (1/2))
Dl = Wvle * dplus * (2 * math.pi) * Wr
Dr = Wvre * dplus * (2 * math.pi) * Wr
turningLeft = True
if(turningLeft == True):
    Dlarge = Dr
    Dsmall = Dl
    
else:
    Dlarge = Dl
    Dsmall = Dr
    
Dw = 10
Trlarge = 10
Trsmall = Trlarge - Dw
Dsmall = 2 * math.pi * Trsmall
Dlarge = 2 * math.pi * Trlarge
Dratio = Dsmall/Dlarge
Ad = (Dl/Dlarge) * 2 * math.pi

#Combine
combined = (Ao + Ad)/2

#Pose Displacement
Oo = combined
Angle = Ad
y = math.cos(Angle) * Oo
x = math.sin(Angle) * Oo
Theta = Angle

retval = [x, y, Theta]

def ret():
    print(retval)

ret()

