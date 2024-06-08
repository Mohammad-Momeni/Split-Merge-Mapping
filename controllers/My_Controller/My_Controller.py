from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
import math

# Getting data with Distance Sensor functions
def inverseKinematics(v, omega, l):
    r = 20.5 / 1000
    phi1 = (v + ((l * omega) / 2)) / r
    phi2 = (v - ((l * omega) / 2)) / r
    
    wheelsVelocity = np.array([phi1,
                               phi2])
    return wheelsVelocity

def calculatePosition(start, distance, angle):
    radiants = math.radians(angle)
    x, y = start
    x += distance * math.cos(radiants)
    y += distance * math.sin(radiants)
    return x, y

# Get robot's heading in degree based on compass values
def getRobotHeading(compassValue):
    rad = math.atan2(compassValue[1], compassValue[0])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0
    
    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading

def tableToReal(lookupTable, output):
    measured = lookupTable[:, 1]
    noMeasured = len(measured)
    index = 0

    for i in range(noMeasured):
        if measured[i] <= output:
            index = i
            break

    if index == 0:
        if output <= measured[noMeasured - 1]:
            return lookupTable[:, 0][noMeasured - 1]
        return 0

    firstDotY = measured[index - 1]
    firstDotX = lookupTable[:, 0][index - 1]
    secondDotY = measured[index]
    secondDotX = lookupTable[:, 0][index]
    slope = (firstDotY - secondDotY) / (firstDotX - secondDotX)
    intercept = firstDotY - (slope * firstDotX)
    realDistance = (output - intercept) / slope
    return realDistance

# Split and Merge functions
def toTuple(arr1, arr2):
    result = []
    for i in range(0, len(arr1)):
        result.append((arr1[i], arr2[i]))
    return result

def distanceFromLine(node1, node2, node3): # distance of node3 from line of node1-node2
    x1 = node1[0]
    y1 = node1[1]
    x2 = node2[0]
    y2 = node2[1]
    x3 = node3[0]
    y3 = node3[1]
    return abs((y2 - y1) * x3 - (x2 - x1) * y3 + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

def distanceBetweenNodes(node1, node2): # distance between node1 and node2
    x1 = node1[0]
    y1 = node1[1]
    x2 = node2[0]
    y2 = node2[1]
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def needToSplit(node1, node2, node3, threshold):
    distance = distanceFromLine(node1, node2, node3)
    return distance > threshold

def maxDistanceFromLine(arr, threshold): # find the point with maximum distance from line
    maxDistance = 0
    maxIndex = 0
    for i in range(1, len(arr) - 1):
        if (needToSplit(arr[0], arr[len(arr) - 1], arr[i], threshold)):
            if (distanceFromLine(arr[0], arr[len(arr) - 1], arr[i]) > maxDistance):
                maxDistance = distanceFromLine(arr[0], arr[len(arr) - 1], arr[i])
            maxIndex = i
    return maxDistance, maxIndex

def Splits(arr, threshold, result , redundant):
    maxDistance, maxIndex = maxDistanceFromLine(arr, threshold)
    if (maxDistance == 0):
        return
    if (len(result) > 0):
        result.pop()
        redundant.pop()

    # merge arrays
    arr1 = []
    arr2 = []
    for i in range(0, maxIndex + 1):
        arr1.append(arr[i])

    for i in range(maxIndex, len(arr)):
        arr2.append(arr[i])

    result.append([arr1[0], arr1[len(arr1) - 1]])
    redundant.append(arr1)
    Splits(arr1, threshold, result, redundant )
    
    result.append([arr2[0], arr2[len(arr2) - 1]])
    redundant.append(arr2)
    Splits(arr2, threshold, result, redundant)

    return result, redundant

def mergeArrays(arr1, arr2):
    result = []
    for i in range(0, len(arr1)):
        result.append(arr1[i])
    for i in range(1, len(arr2)):
        result.append(arr2[i])
    return result

def MergeSplits(result , threshold):
    merge = []
    merge.append(result[0])
    k = 0
    i = 1
    while (i < len(result)):
        arr1 = merge[k]
        maxDistance, maxIndex = maxDistanceFromLine(arr1, threshold)
        arr2 = result[i]
        maxDistance2, maxIndex2 = maxDistanceFromLine(arr2, threshold)
        arr3 = mergeArrays(arr1, arr2)
        maxDistance3, maxIndex3 = maxDistanceFromLine(arr3, threshold)
        if (maxDistance3/distanceBetweenNodes(arr3[0], arr3[len(arr3) - 1]) < maxDistance/distanceBetweenNodes(arr1[0], arr1[len(arr1) - 1]) 
                + maxDistance2/distanceBetweenNodes(arr2[0], arr2[len(arr2) - 1]) and len(arr3) <= 8):
            merge[k] = arr3
        else:
            merge.append(arr2)
            k += 1
        i += 1
    return merge

def SplitAndMerge(arrX, arrY, threshold):
    result = []
    redundant = []
    arr = toTuple(arrX, arrY)
    result.append([arr[0], arr[len(arr) - 1]])
    redundant.append(arr)
    Splits(arr, threshold, result , redundant)
    Merged = MergeSplits(redundant, 0)

    splits = []
    for i in range(0, len(result)):
        if (distanceBetweenNodes(result[i][0], result[i][1]) < 10 * threshold):
            splits.append(result[i])
    
    merge = []
    for i in range(0, len(Merged)):
        if (distanceBetweenNodes(Merged[i][0], Merged[i][len(Merged[i]) - 1]) < 10 * threshold):
            merge.append([Merged[i][0], Merged[i][len(Merged[i]) - 1]])

    return splits, merge

# Create an instance of robot
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

# Load Devices such as sensors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
distanceSensor2 = robot.getDevice('ps2')

# Enables the devices
distanceSensor2.enable(TIME_STEP)
gps.enable(TIME_STEP)
compass.enable(TIME_STEP)

robot.step(TIME_STEP) # take some dummy steps in environment for safe initialization

initialHeading = getRobotHeading(compass.getValues())
initialLocation = gps.getValues()

spin = inverseKinematics(0, (2 * math.pi) / (360 * (TIME_STEP / 1000)), 52/1000)

# Set the motors to rotate for ever
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(spin[1])
rightMotor.setVelocity(spin[0])

# General parameters
heading = 361
firstTime = True
xValues = []
yValues = []
lookupTable = distanceSensor2.getLookupTable()
lookupTable = np.array(lookupTable)
lookupTable = lookupTable.reshape(int(lookupTable.size/3), 3)

while ((initialHeading < heading - 0.5) or (initialHeading > heading + 0.5)) or firstTime:
    firstTime = False
    robot.step(TIME_STEP)
    value = distanceSensor2.getValue()
    realDistance = tableToReal(lookupTable, value)
    heading = getRobotHeading(compass.getValues())
    x, y = calculatePosition(initialLocation[:2], realDistance, heading - 90)
    xValues.append(x)
    yValues.append(y)

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

plt.figure()
plt.scatter(xValues, yValues)
plt.plot(initialLocation[0], initialLocation[1], 'r', marker='*', ms=8)

threshold = 0.0185
splits, merge = SplitAndMerge(xValues, yValues, threshold)

plt.figure()
for i in range(0 , len(splits)):
    plt.plot([splits[i][0][0], splits[i][1][0]], [splits[i][0][1], splits[i][1][1]], 'b', linewidth=2.0)
plt.plot(initialLocation[0], initialLocation[1], 'r', marker='*', ms=8)

plt.figure()
for i in range(0 , len(merge)):
    x = []
    y = []
    for j in range(0 , len(merge[i])):
        x.append(merge[i][j][0])
        y.append(merge[i][j][1])
    plt.plot(x, y, 'g')
plt.plot(initialLocation[0], initialLocation[1], 'r', marker='*', ms=8)

plt.show()