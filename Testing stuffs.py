import turtle
import time
import numpy as np
import math

sc = turtle.Screen()

sc.setup(1000, 1000, startx= 500, starty=500)
sc.bgcolor("black")

l = 55.25
m = 38.4
scale = 3

while(True):
    print("Enter D")
    dis = float(input())
    if(dis == -1):
        break
    
    t1 = math.degrees(np.arccos((-dis**2+l**2+m**2)/(2*l*m)))
    t2 = math.degrees(np.arccos((dis**2+l**2-m**2)/(2*dis*l)))
    
    t3 = 180 - t1 - t2

    print(t1)
    print(t2)
    print(t3)
    turtle.home()
    turtle.clear()
    
    turtle.color('blue') 

    turtle.setheading(270)
    turtle.forward(dis *scale)


    turtle.left(180-t3)
    turtle.forward(m*scale)

    turtle.left(180-t1)
    turtle.forward(l*scale)

