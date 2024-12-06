
from Picasso import Picasso


roboClient=Picasso(
    base_length=11.25,
    a_length=10.5,
    b_length=11.25,
    c_length=11.25,
    d_length=10.5,
    Gear_Ratio_1=4.2,
    Gear_Ratio_2=4.2,
    x0=11.25/2,
    y0=15
)

points=[
    [1,1,False],
    [5,1,True],
    [5,5,True],
    [1,5,True],
    [1,1,True],
    [1,1,False]
        ]

x_offset=3
y_offset=10

points = [[x+x_offset,y+y_offset,state] for x,y,state in points]
print(points)

path_plan=roboClient.InterpolateMoves(points=points,stepsizecm=0.5)
print(path_plan)
                   
roboClient.plot(path_plan)