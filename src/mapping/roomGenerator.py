import random
import turtle
import copy
from pgmGenerator import pgm
from objectGenerator import ItemGenerator


# bedroom
# ensuite
# bathroom
# main living space
# living room
# kitchen diner combo
# kitchen
# dining room
# corridor
class Room:
    def __init__(self, roomType, canBeConnectedTo, sizeConstraint, constraint, weight):
        self.type = roomType
        self.canBeConnectedTo = canBeConnectedTo
        self.size = random.randint(sizeConstraint[0], sizeConstraint[1])
        self.rotation = random.choice([True, False])
        self.length = (random.randint(constraint[0] * 1000, constraint[1] * 1000) / 1000)
        self.width = self.size / self.length
        if self.rotation:
            temp = self.length
            self.length = self.width
            self.width = temp
        self.corners = None
        self.weight = weight

    def setCorners(self, corners):
        self.corners = corners

    def getWidth(self):
        return self.width

    def getLength(self):
        return self.length

    def getRotatedIdiot(self):
        return self.rotation

    def getSize(self):
        return self.size

    def getType(self):
        return self.type

    def getConnections(self):
        return self.canBeConnectedTo

    def getCorners(self):
        return self.corners

    def getWeight(self):
        return self.weight


class Corner:
    def __init__(self, coords, occupied):
        self.coords = coords
        self.occupied = occupied

    def setCoords(self, coords):
        self.coords = coords

    def setOccupied(self, occupied):
        self.occupied = occupied

    def getCoords(self):
        return self.coords

    def getOccupied(self):
        return self.occupied


def drawBox(corners, name):
    t.penup()
    t.setposition(corners[0][0] * 20, corners[0][1] * 20)
    t.pendown()
    for i in range(len(corners)):
        t.setposition(corners[len(corners) - i - 1][0] * 20, corners[len(corners) - i - 1][1] * 20)

    midX = round((corners[0][0] + corners[2][0]) / 2)
    midY = round((corners[0][1] + corners[2][1]) / 2)
    t.penup()
    t.setposition(midX * 20, midY * 20)
    t.pendown()
    t.write(name, font=("Verdana", 5, "normal"), align="center")


def getHighestWeightedRoom(rooms):
    biggest = 0
    current = 0
    for i in range(len(rooms)):
        if rooms[i].getWeight() > biggest:
            biggest = rooms[i].getWeight()
            current = i

    return current


def equation(x, y, a1, b1, p1, q1, a2, b2, p2, q2):
    value1 = ((abs((x - a2) / p2) + ((y - b2) / q2) + abs((x - a2) / p2) - ((y - b2) / q2) - 1) +
              (abs((x - a1) / p1) + ((y - b1) / q1) + abs((x - a1) / p1) - ((y - b1) / q1) - 1))

    value2 = ((abs((x - a2) / p2) + ((y - b2) / q2) + abs((x - a2) / p2) - ((y - b2) / q2) - 1) *
              (abs((x - a1) / p1) + ((y - b1) / q1) + abs((x - a1) / p1) - ((y - b1) / q1) - 1))

    if value1 < 0 == value2:

        return True
    else:
        return False


def checkOverlap(corners, a1, b1, p1, q1, a2, b2, p2, q2):
    variance = 0.000
    overlap = False
    offsets = [[-variance, -variance], [-variance, variance], [variance, variance], [variance, -variance]]
    for i in range(len(corners)):

        value = equation(corners[i][0] + offsets[i][0], corners[i][1] + offsets[i][1], a1, b1, p1, q1, a2, b2, p2, q2)
        if value:
            overlap = True
        if overlap:
            t.pencolor("blue")
            t.penup()
            t.setposition(corners[i][0] * 20, corners[i][1] * 20)
            t.pendown()
            t.pensize(5)
            t.forward(1)
            t.pensize(1)
    return overlap


def generateRoomEquationData(room):
    corners = []
    for corner in room.corners:
        corners.append(corner.coords)

    p = room.length
    q = room.width
    a = (room.corners[0].coords[0] + room.corners[2].coords[0]) / 2
    b = (room.corners[0].coords[1] + room.corners[2].coords[1]) / 2

    return corners, a, b, p, q


rooms = []

size = random.choice(["small", "medium", "large"])

style = random.choice(["open plan", "not open plan"])

if size == "small":
    bedrooms = random.randint(1, 2)
    ensuite = True
elif size == "medium":
    bedrooms = random.randint(2, 3)
    ensuite = random.choice([True, False])
else:
    bedrooms = random.randint(4, 5)
    ensuite = False

if not ensuite:
    bathrooms = random.randint(1, bedrooms - 1)
else:
    bathrooms = 0

for _ in range(bedrooms):
    rooms.append("bedroom")
    if ensuite:
        rooms.append("ensuite")

for _ in range(bathrooms):
    rooms.append("bathroom")

if style == "open plan":
    rooms.append("main living space")
else:
    rooms.append("living room")
    if random.choice([True, False]):
        rooms.append("kitchen diner combo")
    else:
        rooms.append("kitchen")
        rooms.append("dining room")
if bedrooms > 2:
    rooms.append("corridor")

for i in range(len(rooms)):
    if rooms[i] == "bedroom":
        connections = ["corridor", "main living space", "living room"]
        size = [8, 20]
        constraint = [2, 6]
        weight = 1

    elif rooms[i] == "ensuite":
        connections = ["bedroom"]
        size = [6, 7]
        constraint = [2, 3]
        weight = 0

    elif rooms[i] == "bathroom":
        connections = ["corridor", "kitchen", "living room", "main living space", "dining room"]
        size = [6, 8]
        constraint = [3, 4]
        weight = 1

    elif rooms[i] == "main living space":
        connections = None
        size = [35, 60]
        constraint = [5, 10]
        weight = 5

    elif rooms[i] == "living room":
        connections = None
        size = [14, 22]
        constraint = [3, 6]
        weight = 5

    elif rooms[i] == "kitchen diner combo":
        connections = ["corridor", "living room"]
        size = [20, 30]
        constraint = [4, 7]
        weight = 3

    elif rooms[i] == "kitchen":
        connections = ["living room", "dining room", "corridor"]
        size = [10, 20]
        constraint = [2, 4]
        weight = 3

    elif rooms[i] == "dining room":
        connections = ["living room", "kitchen", "corridor"]
        size = [10, 20]
        constraint = [2, 4]
        weight = 3
    else:
        connections = ["main living space", "living room", "kitchen", "dining room"]
        size = [4, 10]
        constraint = [2, 3]
        weight = 4

    roomObj = Room(rooms[i], connections, size, constraint, weight)
    rooms[i] = roomObj

t = turtle.Turtle()
t.hideturtle()
t.speed(0)
for _ in range(4):
    t.forward(1000)
    t.backward(1000)
    t.left(90)
# BOOM that's a lot of rooms, but we still need to connect them together

current = getHighestWeightedRoom(rooms)
currentRoom = rooms[current]
# generate coordinates
# make corners based on those coordinates
# place new room and update/delete/generate new coordinates
# repeat until all rooms are placed or no solution is found (restart)

upperLeftCoord = [0, 0]
upperRightCoord = [upperLeftCoord[0] + currentRoom.length, upperLeftCoord[1]]
lowerRightCoord = [upperLeftCoord[0] + currentRoom.length, upperLeftCoord[1] - currentRoom.width]
lowerLeftCoord = [upperLeftCoord[0], upperLeftCoord[1] - currentRoom.width]

lowerLeftCorner = Corner(lowerLeftCoord, [1, 0, 0, 0])
upperLeftCorner = Corner(upperLeftCoord, [0, 1, 0, 0])
upperRightCorner = Corner(upperRightCoord, [0, 0, 1, 0])
lowerRightCorner = Corner(lowerRightCoord, [0, 0, 0, 1])

currentRoom.setCorners([upperRightCorner, lowerRightCorner, lowerLeftCorner, upperLeftCorner])
cloneRoom = currentRoom
placedRooms = [currentRoom]

del rooms[current]

doors = []

weightedRooms = []
for _ in range(len(rooms)):
    index = getHighestWeightedRoom(rooms)
    weightedRooms.append((rooms[index]))
    del rooms[index]

while len(weightedRooms) > 0:
    index = 0
    availableRooms = []
    while True:
        found = False
        for i in range(len(placedRooms)):
            if placedRooms[i].type in weightedRooms[index].canBeConnectedTo:
                availableRooms.append(i)
                found = True

        if found:
            break
        index += 1
    currentRoom = weightedRooms[index]
    failure = 0
    available = []
    while failure < 100:
        locationToPlace = random.choice(availableRooms)

        try:
            if len(locationToPlace) == 1:
                locationToPlace = locationToPlace[0]
        except:
            pass
        cornerToPlace = random.choice(placedRooms[locationToPlace].corners)

        # pick a direction
        available = []
        for i in range(len(cornerToPlace.occupied)):
            lower = cornerToPlace.occupied[i - 1]
            try:
                upper = cornerToPlace.occupied[i + 1]
            except:
                upper = cornerToPlace.occupied[0]
            if lower == 1 or upper == 1:
                if cornerToPlace.occupied[i] != 2:
                    available.append(i)

        if len(available) > 0:
            break
        else:
            failure += 1

    if len(available) == 0:
        break

    elif len(available) == 1:
        desiredDirection = available[0]
    else:
        desiredDirection = random.choice(available)

    # 0 = upperRight, 1 = lowerRight, 2 = lowerLeft 3 = upperLeft
    cornerIndex = placedRooms[locationToPlace].corners.index(cornerToPlace)

    backup = copy.deepcopy(placedRooms[locationToPlace].corners[cornerIndex].coords)

    upperLeftCoord = cornerToPlace.coords

    if cornerIndex == 0:
        if desiredDirection == 3:
            upperLeftCoord[0] = upperLeftCoord[0] - currentRoom.length
            upperLeftCoord[1] = upperLeftCoord[1] + currentRoom.width
            doors.append([backup[0] - 1, backup[1]])
        else:
            doors.append([backup[0], backup[1] - 1])
    elif cornerIndex == 1:
        if desiredDirection == 2:
            upperLeftCoord[0] = upperLeftCoord[0] - currentRoom.length
            doors.append([backup[0] - 1, backup[1]])

        else:
            upperLeftCoord[1] = upperLeftCoord[1] + currentRoom.width
            doors.append([backup[0], backup[1] + 1])

    elif cornerIndex == 2:
        if desiredDirection == 1:
            upperLeftCoord[0] = upperLeftCoord[0] - currentRoom.length
            upperLeftCoord[1] = upperLeftCoord[1] + currentRoom.width
            doors.append([backup[0], backup[1] + 1])

        else:
            doors.append([backup[0] + 1, backup[1]])

    elif cornerIndex == 3:
        if desiredDirection == 0:
            upperLeftCoord[1] = upperLeftCoord[1] + currentRoom.width
            doors.append([backup[0] + 1, backup[1]])
        else:
            upperLeftCoord[0] = upperLeftCoord[0] - currentRoom.length
            doors.append([backup[0], backup[1] - 1])

    placedRooms[locationToPlace].corners[cornerIndex].coords = backup

    upperRightCoord = [upperLeftCoord[0] + currentRoom.length, upperLeftCoord[1]]
    lowerRightCoord = [upperLeftCoord[0] + currentRoom.length, upperLeftCoord[1] - currentRoom.width]
    lowerLeftCoord = [upperLeftCoord[0], upperLeftCoord[1] - currentRoom.width]

    lowerLeftCorner = Corner(lowerLeftCoord, [1, 0, 0, 0])
    upperLeftCorner = Corner(upperLeftCoord, [0, 1, 0, 0])
    upperRightCorner = Corner(upperRightCoord, [0, 0, 1, 0])
    lowerRightCorner = Corner(lowerRightCoord, [0, 0, 0, 1])
    #####################################
    # update corners based on the stupid diagrams I made
    if cornerToPlace.occupied[0] == 1:
        upperRightCorner.occupied = cornerToPlace.occupied
    if cornerToPlace.occupied[1] == 1:
        lowerRightCorner.occupied = cornerToPlace.occupied
    if cornerToPlace.occupied[2] == 1:
        lowerLeftCorner.occupied = cornerToPlace.occupied
    if cornerToPlace.occupied[3] == 1:
        upperLeftCorner.occupied = cornerToPlace.occupied
    #####################################
    corners = [lowerLeftCorner, upperLeftCorner, upperRightCorner, lowerRightCorner]
    corners[(desiredDirection + 2) % 4].occupied = cornerToPlace.occupied
    corners[(desiredDirection + 2) % 4].occupied[desiredDirection] = 1
    corners[(desiredDirection + 2) % 4].occupied[(desiredDirection + 2) % 4] = 2

    #####################################
    # baseline coords generated, now they need to be modified such that they don't allow for incorrect rooms
    placedRooms[locationToPlace].corners[cornerIndex].occupied[desiredDirection] = 2
    currentRoom.setCorners([upperRightCorner, lowerRightCorner, lowerLeftCorner, upperLeftCorner])

    placedRooms[locationToPlace].corners[cornerIndex].occupied[desiredDirection] = 2

    # preventing bedrooms from having 2 ensuites
    if currentRoom.type == "ensuite":
        for i in range(4):
            currentRoom.corners[i].occupied = [2, 2, 2, 2]
        if placedRooms[locationToPlace].type == "bedroom":
            for i in range(4):
                placedRooms[locationToPlace].corners[i].occupied = [2, 2, 2, 2]

    if not ensuite:
        if currentRoom.type == "bedroom":
            for i in range(4):
                currentRoom.corners[i].occupied = [2, 2, 2, 2]

    if currentRoom.type == "bathroom":
        for i in range(4):
            currentRoom.corners[i].occupied = [2, 2, 2, 2]
    placedRooms.append(currentRoom)
    del weightedRooms[index]

placedRooms[0] = cloneRoom

t.pencolor("red")
for room in placedRooms:
    upperRightCoord = room.corners[0].coords
    upperLeftCoord = room.corners[3].coords
    lowerRightCoord = room.corners[1].coords
    lowerLeftCoord = room.corners[2].coords
    drawBox([upperRightCoord, lowerRightCoord, lowerLeftCoord, upperLeftCoord], room.type)

if len(weightedRooms) == 0:
    # ok so theoretically this should only happen if it actually placed all the rooms,
    # but we still need to check that none of the rooms overlap
    # I derived a looooong equation to do that for all rooms,
    # so we check no rooms overlap next
    failure = False
    for i in range(len(placedRooms)):
        corners, a1, b1, p1, q1 = generateRoomEquationData(placedRooms[i])
        for j in range(len(placedRooms)):
            useless, a2, b2, p2, q2 = generateRoomEquationData(placedRooms[j])
            if i != j:
                if checkOverlap(corners, a1, b1, p1, q1, a2, b2, p2, q2):
                    failure = True

    print(failure)

newMap = pgm()

smallestX = 1000
smallestY = 1000
largestX = 0
largestY = 0

for room in placedRooms:
    for corner in room.corners:
        if corner.coords[0] > largestX:
            largestX = corner.coords[0]
        elif corner.coords[0] < smallestX:
            smallestX = corner.coords[0]
        if corner.coords[1] > largestY:
            largestY = corner.coords[1]
        elif corner.coords[1] < smallestY:
            smallestY = corner.coords[1]

averageX = round((smallestX + largestX) / 2)
averageY = round((smallestY + largestY) / 2)

offsetX = 250 + averageX * -20

offsetY = 250 + averageY * -20

for room in placedRooms:
    topLeft = room.corners[3].coords
    bottomRight = room.corners[1].coords
    xRange = [round(topLeft[0] * 20) + offsetX, round(bottomRight[0] * 20) + offsetX]
    yRange = [round(bottomRight[1] * 20) + offsetY, round(topLeft[1] * 20) + offsetY]

    newMap.addRoom(xRange, yRange)

#print(len(doors))
for door in doors:
    #print(door)
    newMap.addDoor([round(door[0] * 20) + offsetX, round(door[1] * 20) + offsetY])

#print(averageX, averageY)

# make some objects idk
print(random.randint(0,0))
print(placedRooms)
objList = ItemGenerator()
objList.generateObjects(placedRooms)
allItems = copy.deepcopy(objList.allItems)
t.pensize(1)
t.pencolor("black")
for i in range(len(allItems)):
    t.penup()
    t.setposition(allItems[i][1]*20,allItems[i][2]*20)
    t.pendown()
    t.forward(1)
    allItems[i] = (allItems[i][0],(round(allItems[i][1] * 20) + offsetX),(round(allItems[i][2] * 20) + offsetY))

objList.allItems = allItems
objList.saveToFile()


# do some rotating to bud!


# make image noisy

newMap.generatePGM()

turtle.mainloop()
