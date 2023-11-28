import copy
import csv
import os
import random


def roomExists(item_location, room_list):
    locations = []
    for i in range(len(room_list)):
        if item_location == room_list[i].type:
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

    def setIsNextTo(self, is_next_to):
        self.isNextTo = is_next_to

    def setISOnTopOf(self, is_on_top_of):
        self.isOnTopOf = is_on_top_of


class ItemGenerator:
    def __init__(self):
        self.allItems = []
        self.itemRooms = []
        self.itemObjList = []

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

        for i in range(len(data)):
            for j in range(len(types)):
                if types[j] == "Item":
                    new_item = Item()
                    new_item.setName(data[i][j])
                    self.itemObjList.append(new_item)
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

    def generateObjects(self, room_list):
        for item in self.itemObjList:
            for i in range(len(item.rooms)):
                if item.quantities[i][1] != 0:

                    room_indexes = roomExists(item.rooms[i], room_list)
                    data = [item.name, item.quantities[i], room_indexes]
                    if len(room_indexes) > 0 :
                        self.itemRooms.append(data)

        wall_gap = 0.5
        for i in range(len(room_list)):
            room_items = []
            for potentialItem in self.itemRooms:
                if i in potentialItem[2]:
                    item_quantity = random.randint(potentialItem[1][0], potentialItem[1][1])
                    top_left_corner = copy.deepcopy(room_list[i].corners[3].coords)
                    bottom_right_corner = copy.deepcopy(room_list[i].corners[1].coords)
                    top_left_corner = [top_left_corner[0] + wall_gap, top_left_corner[1] - wall_gap]
                    bottom_right_corner = [bottom_right_corner[0] - wall_gap, bottom_right_corner[1] + wall_gap]
                    for k in range(item_quantity):
                        x_coord = random.randint(round(top_left_corner[0] * 1000), round(bottom_right_corner[0] * 1000)) / 1000
                        y_coord = random.randint(round(bottom_right_corner[1] * 1000), round(top_left_corner[1] * 1000)) / 1000
                        room_items.append((potentialItem[0], x_coord, y_coord))
                        self.allItems.append((potentialItem[0], x_coord, y_coord))

    def saveToFile(self):
        output_dir = os.path.join(os.path.dirname(__file__), "out")
        os.makedirs(output_dir, exist_ok=True)
        f = open(os.path.join(output_dir, "itemList.txt"), "a")
        f.truncate(0)
        for item in self.allItems:
            line = "("
            for i in range(len(item)):
                if i == 0:
                    line = line + '"' + str(item[i]) + '",'
                else:
                    line = line + str(item[i]) + ","
            line = line[:-1]
            line = line + ")"

            f.writelines(line)

            f.writelines("\n")
        f.close()
