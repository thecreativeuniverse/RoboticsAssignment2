from roomGenerator import *
import copy
def generateCorners(room_objects,ensuite):
    current = getHighestWeightedRoom(room_objects)
    currentRoom = room_objects[current]

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

    del room_objects[current]

    doors = []

    weightedRooms = []
    for _ in range(len(room_objects)):
        index = getHighestWeightedRoom(room_objects)
        weightedRooms.append((room_objects[index]))
        del room_objects[index]

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
        currentRoom.setCorners([upperRightCorner, lowerRightCorner, lowerLeftCorner, upperLeftCorner])

        placedRooms[locationToPlace].corners[cornerIndex].occupied[desiredDirection] = 2
        placedRooms[locationToPlace].corners[desiredDirection].occupied[cornerIndex] = 2

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

    return placedRooms,doors