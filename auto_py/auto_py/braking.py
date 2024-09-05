#automatic braking based on LiDAR scan of points
#270-deg fov, up to 10m
#DO NOT USE WITH POLAR - MATH ORGINALLY DONE FOR CARTESIAN

import numpy as np
n_points=10 #number of points

def make_points():
    #assume unitsreceive_lidar(self, msg: LaserScan) in centimeters: 1 cm - 500 cm (5m)
    arr1 = np.array([[1,-500]]) #dummy (gets filtered out) to make blank array with x,y -> looking for better way to do this
    count = n_points
    while count >0:
        x = np.random.randint(-500,500)
        y = np.random.randint(-500,500)
        arr2 = np.array([[x,y]])
        arrbig = np.append(arr1,arr2,axis=0)
        count = count-1
        arr1=arrbig
    print("array:",arrbig)
    return(arrbig)

def filter_points(arr,n_points):
    filtered=([])
    for i in range(n_points+1):
        coords=arr[i]
        x = coords[0]
        y = coords[1]
        print("x:",x)
        print("y:",y)
        #--calculate angle from y-axis--
        angle = angle_math(x,y)
        print("angle:",angle)
        #--filters by angles, saves filtered coordinates
        if angle <20:
            if angle > -20:
                filtered.append([x,y])
    print(filtered)
    return(filtered)

def angle_math(x,y):
    angle=np.arctan(x/y)
    angle = np.rad2deg(angle)
    if y<0: #deals with negatives
        if x < 0: #adjusts for quadrant 3
            angle = angle - 180
        if x > 0: #adjusts for quadrant 4
            angle = angle + 180
    return angle

def filter_distance(filtered):
    print('list:',filtered)
    n_coords = len(filtered)
    for i in range (n_coords):
        coords = filtered[i]
        x = coords[0]
        y = coords[1]
        distance = distance_math (x,y)
        if distance < 200:
            return('stop')
    return ('good to go')

def distance_math(x,y):
    print('x: ',x)
    print('y: ',y)
    x = x ** 2
    y = y ** 2
    distance = np.sqrt(x+y)
    print('distance:',distance)
    return distance

def make_decisions(filtered_points):
    result = filter_distance(filtered_points)
    if result == 'stop':
        stop()
    elif result == 'good to go':
        keep_going()

def stop():
    print('stop')
    
def keep_going():
    print('keep going')

def main():
    arr = make_points()
    filtered_points = filter_points(arr,n_points)
    make_decisions(filtered_points)

if __name__ == '__main__':
    main()