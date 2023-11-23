import random


class Room:
    def __init__(self, room_type, can_be_connected_to, size_constraint, length_constraint, weight):
        self.type = room_type
        self.canBeConnectedTo = can_be_connected_to
        self.size = random.randint(size_constraint[0], size_constraint[1])
        self.rotation = random.choice([True, False])
        self.length = (random.randint(length_constraint[0] * 1000, length_constraint[1] * 1000) / 1000)
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


def getHighestWeightedRoom(rooms):
    biggest = 0
    current = 0
    for i in range(len(rooms)):
        if rooms[i].getWeight() > biggest:
            biggest = rooms[i].getWeight()
            current = i

    return current


def checkOverlap(corners1, corners2):
    overlap = False
    upper_left_corner = corners2[3]
    lower_right_corner = corners2[1]
    lower_x = lower_right_corner.coords[1]
    upper_x = upper_left_corner.coords[1]
    lower_y = upper_left_corner.coords[0]
    upper_y = lower_right_corner.coords[0]
    for corner in corners1:
        coords = corner.coords
        print(lower_x, lower_y, upper_x, upper_y)
        if lower_x < coords[0] < upper_x and lower_y < coords[1] < upper_y:
            print("overlap")
            overlap = True
            break
        else:
            print("no overlap")
    # check if corners1 are in corners2
    return overlap
