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


def checkOverlap(tortoise,corners1, corners2):
    overlap = False
    upper_left_corner = corners2[3]
    lower_right_corner = corners2[1]
    lower_x = upper_left_corner.coords[0]
    upper_x = lower_right_corner.coords[0]
    lower_y = upper_left_corner.coords[1]
    upper_y = lower_right_corner.coords[1]
    for corner in corners1:

        coords = corner.coords
        print((coords[0],coords[1]*-1),(lower_x,-1*lower_y),(upper_x,-1*upper_y))
        if lower_x < coords[0] < upper_x and lower_y*-1 < coords[1]*-1 < upper_y*-1:
            if (round(lower_x, 3) < round(coords[0], 3) < round(upper_x, 3) and
                    round(lower_y * -1, 3) < round(coords[1] * -1, 3) < round(upper_y * -1, 3)):
                print("overlap")
                tortoise.drawOverlap(corner)
                tortoise.t.pencolor("black")
                tortoise.t.penup()
                tortoise.t.setposition(lower_right_corner.coords[0] * 20, lower_right_corner.coords[1] * -20)
                tortoise.t.pendown()
                tortoise.t.setposition(corner.coords[0] * 20, corner.coords[1] * -20)
                tortoise.t.setposition(upper_left_corner.coords[0] * 20, upper_left_corner.coords[1] * -20)

                overlap = True
                break
            else:
                print("no overlap")
        else:
            print("no overlap")

    # check if corners1 are in corners2
    return overlap

def generateInitialRooms():
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
    return rooms, ensuite

def generateRoomObjects(rooms):
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
            rooms[i] = Room(rooms[i], ["main living space", "living room", "kitchen", "dining room"], [4, 10], [2, 3],4)

    return rooms