from pgmGenerator import PGM
from itemGenerator import ItemGenerator
from svgGenerator import PogTurtle
from cornerGenerator import *
import os.path


def errorChecker(PlacedRooms):
    failure = False
    for i in range(len(placedRooms)):
        for j in range(len(placedRooms)):
            if i != j:
                if checkOverlap(tortoise, placedRooms[i].corners, placedRooms[j].corners):
                    # corner is inside of room
                    failure = True

    x_lines = []
    y_lines = []
    for room in placedRooms:
        for i in range(len(room.corners)):
            corner1 = room.corners[(i) % 4].coords
            corner2 = room.corners[(i + 1) % 4].coords
            if (i % 2) == 0:
                y_lines.append([corner1[0], corner1[1], corner2[1]])
            else:
                x_lines.append([corner1[1], corner1[0], corner2[0]])
    for x_line in x_lines:
        for y_line in y_lines:
            if y_line[1] < x_line[0] < y_line[2] and x_line[1] < y_line[0] < x_line[2]:
                # line overlap
                failure = True

    return failure


tortoise = PogTurtle()

containsErrors = True
while containsErrors:
    tortoise.t.clear()
    rooms, ensuite = generateInitialRooms()
    room_objects = generateRoomObjects(rooms)

    placedRooms, doors= generateCorners(room_objects,ensuite)

    containsErrors = errorChecker(placedRooms)


for room in placedRooms:
    tortoise.drawRoom([room.corners[0].coords, room.corners[1].coords,
                       room.corners[2].coords, room.corners[3].coords], room.type)





newMap = PGM()

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
tortoise.drawItems(allItems, offsetX, offsetY)

itemList.allItems = allItems
itemList.saveToFile()

newMap.generatePGM()

tortoise.saveSVG()
