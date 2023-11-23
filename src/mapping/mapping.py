import copy
from pgmGenerator import pgm
from objectGenerator import ItemGenerator
from roomGenerator import *
from svgGenerator import PogTurtle
import os.path


# generate turtle and make the map white


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
        rooms[i] = Room(rooms[i], ["corridor", "main living space", "living room"], [8, 20], [2, 6], 1)

    elif rooms[i] == "ensuite":
        rooms[i] = Room(rooms[i], ["bedroom"], [6, 7], [2, 3], 0)

    elif rooms[i] == "bathroom":
        rooms[i] = Room(rooms[i], ["corridor", "kitchen", "living room", "main living space"], [6, 8], [3, 4], 1)

    elif rooms[i] == "main living space":
        rooms[i] = Room(rooms[i], None, [35, 60], [5, 10], 5)

    elif rooms[i] == "living room":
        rooms[i] = Room(rooms[i], None, [14, 22], [3, 6], 5)

    elif rooms[i] == "kitchen diner combo":
        rooms[i] = Room(rooms[i], ["corridor", "living room"], [20, 30], [4, 7], 3)

    elif rooms[i] == "kitchen":
        rooms[i] = Room(rooms[i], ["living room", "dining room", "corridor"], [10, 20], [2, 4], 3)

    elif rooms[i] == "dining room":
        rooms[i] = Room(rooms[i], ["living room", "kitchen", "corridor"], [10, 20], [2, 4], 3)

    else:
        rooms[i] = Room(rooms[i], ["main living space", "living room", "kitchen", "dining room"], [4, 10], [2, 3], 4)

current = getHighestWeightedRoom(rooms)
currentRoom = rooms[current]

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

tortoise = PogTurtle()
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

tortoise.t.pencolor("red")
for room in placedRooms:
    tortoise.drawRoom([room.corners[0].coords, room.corners[1].coords,
                       room.corners[2].coords, room.corners[3].coords], room.type)

if len(weightedRooms) == 0:
    failure = False
    for i in range(len(placedRooms)):
        for j in range(len(placedRooms)):
            if i != j:
                if checkOverlap(placedRooms[i].corners, placedRooms[j].corners):
                    for corner in placedRooms[i].corners:
                        tortoise.drawOverlap(corner)
                    failure = True

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

width = largestX - smallestX
height = largestY - smallestY

worldPath = os.path.dirname(__file__)
worldPath = os.path.join(worldPath, "../mapping/out/world.world")

text = "  size [" + str(width) + " " + str(height) + " 0.5]\n"

lines = open(worldPath, 'r').readlines()
lines[63] = text
out = open(worldPath, 'w')
out.writelines(lines)
out.close()

for door in doors:
    newMap.addDoor([round(door[0] * 20) + offsetX, round(door[1] * 20) + offsetY])

# make some objects idk
itemList = ItemGenerator()
itemList.generateObjects(placedRooms)
allItems = copy.deepcopy(itemList.allItems)
tortoise.drawItems(allItems,offsetX,offsetY)

itemList.allItems = allItems
itemList.saveToFile()

newMap.generatePGM()

tortoise.saveSVG()