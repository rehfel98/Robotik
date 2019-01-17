import math
def nearestPoint(x,y):
    x = x*100
    y= y*100
    if(x < 185):
        x = x-185
        y = y - 215
        r = 120
        m = (y)/(x) #Steigung
        b = y - m*x           #Y-Achsenabschnitt
        p = (2*b*m)/(m**2 + 1)
        q = (b-r**2)/(m**2+1)
        x1 = -(p/2)+ math.sqrt((p/2)**2 - q)
        x2 = -(p / 2) - math.sqrt((p / 2) ** 2 - q)

        if(x1 < 0):
            return((x1+185)*0.01, (m*x1+b+215)*0.01)
        else:
            return((x2+185)*0.01,(m*x2+b+215)*0.01)

    elif(x <= 415):
        if(y < 215):
            return(x*0.01, 95*0.01)
        else:
            return(x*0.01, 335*0.01)
    else:
        x = x - 415
        y = y - 215
        r = 120
        m = (y) / (x)  # Steigung
        b = y - m * x  # Y-Achsenabschnitt
        p = (2 * b * m) / (m ** 2 + 1)
        q = (b - r ** 2) / (m ** 2 + 1)
        x1 = -(p / 2) + math.sqrt((p / 2) ** 2 - q)
        x2 = -(p / 2) - math.sqrt((p / 2) ** 2 - q)

        if (x1 > 0):
            return ((x1 + 415)+0.01, (m * x1 + b + 215)*0.01)
        else:
            return ((x2 + 415)*0.01, (m * x2 + b + 215)*0.01)

print(nearestPoint(1,3))