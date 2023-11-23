import copy
import csv
import os
import random


def roomExists(itemLocation, roomList):
    locations = []
    for i in range(len(roomList)):
        if itemLocation == roomList[i].type:
            locations.append(i)

    return locations


class Item:
    def __init__(self):
        self.name = None
        self.rooms = []
        self.quantities = []
        self.isNextTo = []
        self.isOnTopOf = []

    def setName(self, name):
        self.name = name

    def addLocation(self, room, quantity):
        self.rooms.append(room)
        self.quantities.append(quantity)

    def setIsNextTo(self, isNextTo):
        self.isNextTo = isNextTo

    def setISOnTopOf(self, isOnTopOf):
        self.isOnTopOf = isOnTopOf


class ItemGenerator:
    def __init__(self):
        current_file = os.path.dirname(__file__)
        with open(os.path.join(current_file, 'ObjectList.csv')) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            data = []
            for row in csv_reader:
                if line_count == 0:
                    types = row
                    line_count += 1
                else:
                    data.append(row)

        self.itemRooms = []
        self.itemObjList = []
        for i in range(len(data)):
            for j in range(len(types)):
                if types[j] == "Item":
                    newItem = Item()
                    newItem.setName(data[i][j])
                    self.itemObjList.append(newItem)
                elif types[j] == "isNextTo":
                    info = data[i][j].split(",")
                    self.itemObjList[-1].setIsNextTo(info)

                elif types[j] == "isOnTopOf":
                    info = data[i][j].split(",")
                    self.itemObjList[-1].setISOnTopOf(info)
                else:
                    info = data[i][j].split("-")
                    info = list(map(int, info))
                    if len(info) == 1:
                        info.append(0)
                    self.itemObjList[-1].addLocation(types[j], info)

    def generateObjects(self, roomList):
        #################################
        for room in roomList:
            print(room.type)
        for item in self.itemObjList:
            print(vars(item))
        #################################

        for item in self.itemObjList:
            for i in range(len(item.rooms)):
                if item.quantities[i][1] != 0:

                    roomIndexes = roomExists(item.rooms[i], roomList)
                    data = [item.name, item.quantities[i], roomIndexes]
                    if len(roomIndexes) > 0 and roomIndexes != [0]:
                        self.itemRooms.append(data)

        # iterate over each room
        # check if item is able/supposed to exist there
        # randomly generate the quantity if necessary
        # place items based on whether they need to be next to a wall etc
        # save all placed items into array per room
        # combine arrays
        # have list YAY

        wallGap = 0.3
        self.allItems = []
        for i in range(len(roomList)):
            roomItems = []
            for potentialItem in self.itemRooms:
                if i in potentialItem[2]:
                    itemQuantity = random.randint(potentialItem[1][0], potentialItem[1][1])
                    topLeftCorner = copy.deepcopy(roomList[i].corners[3].coords)
                    bottomRightCorner = copy.deepcopy(roomList[i].corners[1].coords)
                    topLeftCorner = [topLeftCorner[0] + wallGap, topLeftCorner[1] - wallGap]
                    bottomRightCorner = [bottomRightCorner[0] - wallGap, bottomRightCorner[1] + wallGap]
                    print(topLeftCorner)
                    print(bottomRightCorner)
                    for k in range(itemQuantity):
                        xCoord = random.randint(round(topLeftCorner[0]) * 1000,
                                                round(bottomRightCorner[0]) * 1000) / 1000
                        yCoord = random.randint(round(bottomRightCorner[1]) * 1000,
                                                round(topLeftCorner[1]) * 1000) / 1000
                        roomItems.append((potentialItem[0], xCoord, yCoord))
                        self.allItems.append((potentialItem[0], xCoord, yCoord))

    def saveToFile(self):
        output_dir = os.path.join(os.path.dirname(__file__), "out")
        os.makedirs(output_dir, exist_ok=True)
        f = open(os.path.join(output_dir, "itemList.txt"), "a")
        f.truncate(0)
        for item in self.allItems:
            line = "("
            for i in range(len(item)):
                if i ==0:
                    line =line + '"'+str(item[i]) + '",'
                else:
                    line = line + str(item[i]) + ","
            line = line[:-1]
            line = line + ")"

            f.writelines(line)

            f.writelines("\n")
        f.close()
