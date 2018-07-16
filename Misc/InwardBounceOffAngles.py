import numpy as np
from math import *

def distance_between(point1, point2):
  # computes distance between point1 and point2
  x1, y1 = point1
  x2, y2 = point2
  return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

pullData = open("training_data.txt","r").read()
dataArray = pullData.split('\n')
xar = []
yar = []
for eachLine in dataArray:
    if len(eachLine)>1:
        x,y = eachLine.split(',')
        xar.append(int(x))
        yar.append(int(y))


def find_center(w, temp):
    center_point = ()
    if w == "t":
        # TOP
        for i in range(len(temp)):
            # if the previous one is larger than the next one
            if yar[temp[i]-1] > yar[temp[i]]:
                center_point = (xar[temp[i]-1], yar[temp[i]-1])
                # print "got center point for TOP"
                break
        if not center_point:
            center_point = (xar[temp[len(temp)-1]], yar[temp[len(temp)-1]])
        # print "eyeballed center since no pivit took place"
        # print center_point
        # BOTTOM
    elif w == "b":
        for i in range(len(temp)):
            if yar[temp[i]-1] < yar[temp[i]]:
                center_point = (xar[temp[i]-1], yar[temp[i]-1])
                # print "got center point for BOTTOM"
                break
        if not center_point:
            center_point = (xar[temp[len(temp)-1]], yar[temp[len(temp)-1]])
        # print "eyeballed center since no pivit took place"
        # print center_point
    elif w == "r":
        for i in range(len(temp)):
            if xar[temp[i]-1] > xar[temp[i]]:
                center_point = (xar[temp[i]-1], yar[temp[i]-1])
                # print "got center point for RIGHT"
                break
        if not center_point:
            center_point = (xar[temp[len(temp)-1]], yar[temp[len(temp)-1]])
        # print "eyeballed center since no pivit took place"
        # print center_point
    elif w == "l":
        for i in range(len(temp)):
            if xar[temp[i]-1] < xar[temp[i]]:
                center_point = (xar[temp[i]-1], yar[temp[i]-1])
                # print "got center point for LEFT"
                break
        if not center_point:
            center_point = (xar[temp[len(temp)-1]], yar[temp[len(temp)-1]])
        # print "eyeballed center since no pivit took place"
        # print center_point
    elif w == "":
        print "ERROR"
    return center_point


def find_angles (temp, center_point, w):

    # find direction
    if len(temp)==1:
        # print "Error, only one point for bouning"
        return 0, 0

    in_point1 = (xar[temp[0]-1], yar[temp[0]-1])
    out_point2 = (xar[temp[len(temp)-1]+1], yar[temp[len(temp)-1]+1])

    x_v_p1 = center_point[0]-in_point1[0]
    y_v_p1 = center_point[1]-in_point1[1]

    x_v_p2 = out_point2[0]-center_point[0]
    y_v_p2 = out_point2[1]-center_point[1]

    in_angle = atan2(y_v_p1, x_v_p1)
    out_angle = atan2(y_v_p2, x_v_p2)

    return in_angle, out_angle


flag = 0
temp = [] # array for center part walls
temp_b = []
temp_t = []
temp_r = []
temp_l = []
temp_c = [] # array for corner part

angle_in = []
angle_out = []
corners = []
indexes = []

count = 0
w = ""

y_b = 105
y_t = 974
x_l = 240
x_r = 1696

margin = 60
corner_margin = 100


for i in range(1, len(yar)-1):

    # BOUNDARIES FOR BOTTOM AND TOP
    if xar[i] > (x_l+corner_margin) and xar[i] < (x_r-corner_margin) and (flag == 0 or flag == 1 or flag == 2):
        if yar[i] < (y_b+margin):
            # BOTTOM
            # print "bottom"
            # print flag

            if flag == 0:
                # print "hit the bottom one"
                # print i
                flag = 1
                w = "b"
                temp.append(i)
            elif flag == 1:
                # print "hit the bottom two"
                temp.append(i)
            elif flag == 5:
                # calculate how long it stayed there
                # call the FUNCTION THAT CALCULATES CORNER ANGLE AND TIME STUFF

                temp = []
                flag = 0

        elif yar[i] > (y_t-margin):
            # TOP
            # print "top"
            if flag == 0:
                flag = 2
                w = "t"
                temp.append(i)
            elif flag == 2:
                temp.append(i)
            elif flag == 5:
                # calculate how long it stayed there
                # call the FUNCTION THAT CALCULATES CORNER ANGLE AND TIME STUFF

                temp = []
                flag = 0
    elif flag == 1 or flag == 2:
        # print "out"
        #calculate angles here
        # print "I DO GET HERE FROM TIME TO TIME"
        center = find_center(w, temp)
        in_angle, out_angle = find_angles(temp, center, w)

        angles = [degrees(in_angle), degrees(out_angle)]

        if w == "t":
            temp_t.append(angles)
        elif w == "b":
            temp_b.append(angles)
        else:
            print "Error"


        angle_in.append(w)
        angle_in.append(degrees(in_angle))
        angle_out.append(w)
        angle_out.append(degrees(out_angle))

        # print angle_in, angle_out

        # for references
        indexes.append(i)
        temp = []
        flag = 0

    # BOUNDARIES FOR LEFT AND RIGHT
    elif yar[i] > (y_b + corner_margin) and yar[i] < (y_t - corner_margin) and (flag == 0 or flag == 3 or flag == 4):
        if xar[i] < (x_l+margin):
            # print "left"
            # LEFT
            if flag == 0:
                flag = 3
                w = "l"
                temp.append(i)
            elif flag == 3:
                temp.append(i)

            elif flag == 5:
                # calculate how long it stayed there
                # call the FUNCTION THAT CALCULATES CORNER ANGLE AND TIME STUFF

                temp = []
                flag = 0

        elif xar[i] > (x_r-margin):
            # RIGHT
            # print "right"
            # print w
            if flag == 0:
                # print "right one"
                # print i
                flag = 4
                w = "r"
                temp.append(i)
            elif flag == 4:
                # print "right two"
                # print i
                temp.append(i)

            elif flag == 5:
                # calculate how long it stayed there
                # call the FUNCTION THAT CALCULATES CORNER ANGLE AND TIME STUFF

                temp = []
                flag = 0
    elif flag == 3 or flag == 4:
        # print "out"
        # #calculate angles here
        # print "THIS ONE FOR RIGHT AND LEFT"
        # print temp
        # print w
        center = find_center(w, temp)
        in_angle, out_angle = find_angles(temp, center, w)

        angles = [degrees(in_angle), degrees(out_angle)]

        if w == "r":
            temp_r.append(angles)
        elif w == "l":
            temp_l.append(angles)
        else:
            print "Error"

        if angle_in ==0:
            temp = []
            flag = 0
        else:
            angle_in.append(w)
            angle_in.append(degrees(in_angle))
            angle_out.append(w)
            angle_out.append(degrees(out_angle))

            # for references
            indexes.append(i)
            temp = []
            flag = 0


    elif yar[i] <= (y_b + corner_margin):
        if xar[i] <= (x_l + corner_margin):
            # print "BOTTOM LEFT"
            # print flag
            if flag == 0:
                # BOTTOM LEFT
                # clear temp
                w="bl"
                temp = []
                flag = 5
                # At which angle did you enter the corner
                temp_c.append(i)
                count += 1
            elif flag == 5:
                count += 1
                temp_c.append(i)

        elif xar[i] >= (x_r - corner_margin):
            # print "BOTTOM RIGHT"
            # print flag
            if flag == 0:
                # BOTTOM RIGHT
                # clear temp
                w="br"
                temp = []
                flag = 6
                # At which angle did you enter the corner
                temp_c.append(i)
                count += 1
                temp_c.append(i)
            elif flag == 6:
                # print w
                count += 1
                temp_c.append(i)

    elif yar[i] >= (y_t - corner_margin):
        if xar[i] <= (x_l + corner_margin):
            # print "TOP LEFT"
            if flag == 0:
                # TOP LEFT
                # clear temp
                w="tl"
                temp = []
                flag = 7
                # At which angle did you enter the corner
                temp_c.append(i)
                count += 1

            elif flag == 7:
                count += 1
                temp_c.append(i)
        elif xar[i] >= (x_r - corner_margin):
            # print "TOP RIGHT"
            if flag == 0:
                # TOP RIGHT
                # clear temp
                w="tr"
                temp = []
                flag = 8
                # At which angle did you enter the corner
                temp_c.append(i)
                count += 1
            elif flag == 8:
                count += 1
                temp_c.append(i)

    elif flag == 5 or flag == 6 or flag == 7 or flag == 8:

        if len(temp_c)==1:
            w = ""
            flag = 0
            temp_c = []
            count = 0
        else:
            # print temp_c[0], temp_c[1]
            if temp_c[0]==temp_c[1]:
                point_1 = (xar[temp_c[0]], yar[temp_c[0]])
                point_2 = (xar[temp_c[1]+1], yar[temp_c[1]+1])
            else:
                point_1 = (xar[temp_c[0]], yar[temp_c[0]])
                point_2 = (xar[temp_c[1]], yar[temp_c[1]])

            x_val_corn = point_2[0]-point_1[0]
            y_val_corn = point_2[1]-point_1[1]

            corner_angle = atan2(y_val_corn, x_val_corn)

            corners.append(w)
            corners.append(count)
            corners.append(degrees(corner_angle))

            w = ""
            flag = 0
            temp_c = []
            count = 0

temp_t.sort()
temp_b.sort()
temp_r.sort()
temp_l.sort()

data = []
data.append(temp_t)
data.append(temp_b)
data.append(temp_r)
data.append(temp_l)

# Here I splitted Data manually into top, bottom, right and left sections
top_n = []
top_w = []
temp = []
weighted = []

for k in range(4):
    for i in range(72):
        for j in range(len(data[k])):
            if data[k][j][0] >= ((i*5)-180) and data[k][j][0] < ((i+1)*5-180):
                temp.append(data[k][j][1])
        if len(temp)>0:
            avg = reduce(lambda x, y: x + y, temp) / len(temp)
            width = len(temp)
            temp = []
        else:
            avg = 0
            width = 0
            temp = []
        top_n.append(avg)
        top_w.append(width)
        weigt_value = [((i*5)-180), width, avg]
        weighted.append(weigt_value)

print weighted
