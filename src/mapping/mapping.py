from pgmGenerator import PGM
from itemGenerator import ItemGenerator
from svgGenerator import PogTurtle
from cornerGenerator import *
import os.path


def errorChecker(placed_rooms, tortoise):
    # checks if a room doesn't work in its current location
    failure = False
    #checks if corner is inside of room
    for i in range(len(placed_rooms)):
        for j in range(len(placed_rooms)):
            if i != j:
                if checkOverlap(tortoise, placed_rooms[i].corners, placed_rooms[j].corners):
                    # corner is inside of room
                    failure = True

    # checks if any lines overlap
    x_lines = []
    y_lines = []
    for room in placed_rooms:
        for i in range(len(room.corners)):
            corner1 = room.corners[i % 4].coords
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


def generate_maps():
    #tortoise is used to draw the svg file
    tortoise = PogTurtle()

    contains_errors = True
    placed_rooms, doors = None, None
    # keeps generating rooms and doors until a valid layout is generated
    while contains_errors:
        tortoise.t.clear()
        tortoise.startup()
        rooms, ensuite = generateInitialRooms()
        room_objects = generateRoomObjects(rooms)

        placed_rooms, doors = generateCorners(copy.deepcopy(room_objects), ensuite)

        contains_errors = errorChecker(placed_rooms, tortoise)
        if len(room_objects) != len(placed_rooms):
            contains_errors = True

    for room in placed_rooms:
        tortoise.drawRoom([room.corners[0].coords, room.corners[1].coords,
                           room.corners[2].coords, room.corners[3].coords], room.type)

    # forms the pgm map
    new_map = PGM()

    smallest_x = 1000
    smallest_y = 1000
    largest_x = 0
    largest_y = 0

    # finds the center of the map
    for room in placed_rooms:
        for corner in room.corners:
            if corner.coords[0] > largest_x:
                largest_x = corner.coords[0]
            elif corner.coords[0] < smallest_x:
                smallest_x = corner.coords[0]
            if corner.coords[1] > largest_y:
                largest_y = corner.coords[1]
            elif corner.coords[1] < smallest_y:
                smallest_y = corner.coords[1]

    average_x = round((smallest_x + largest_x) / 2)
    average_y = round((smallest_y + largest_y) / 2)

    offset_x = 250 + average_x * -20
    offset_y = 250 + average_y * -20

    for room in placed_rooms:
        top_left = room.corners[3].coords
        bottom_right = room.corners[1].coords
        x_range = [round(top_left[0] * 20) + offset_x, round(bottom_right[0] * 20) + offset_x]
        y_range = [round(bottom_right[1] * 20) + offset_y, round(top_left[1] * 20) + offset_y]

        new_map.addRoom(x_range, y_range)

    width = largest_x - smallest_x
    height = largest_y - smallest_y

    room_index = random.randint(0, len(placed_rooms) - 1)
    room = placed_rooms[room_index]

    top_left = room.corners[3].coords
    bottom_right = room.corners[1].coords
    x_center = round((top_left[0] + bottom_right[0]) / 2 - average_x, 2)
    y_center = round((-1 * (top_left[1] + round(bottom_right[1]))) / 2 + average_y, 2)
    robot_pos = 'pioneer( pose [ ' + str(x_center) + ' ' + str(y_center) + ' 0 0 ] name "robot" color "blue")\n'

    world_path = os.path.dirname(__file__)
    world_path = os.path.join(world_path, "../mapping/out/world.world")

    text = "  size [" + str(width) + " " + str(height) + " 0.5]\n"

    lines = open(world_path, 'r').readlines()
    lines[63] = text
    lines[66] = robot_pos
    out = open(world_path, 'w')
    out.writelines(lines)
    out.close()

    for door in doors:
        new_map.addDoor([round(door[0] * 20) + offset_x, round(door[1] * 20) + offset_y])

    # make some objects idk
    item_list = ItemGenerator()
    item_list.generateObjects(placed_rooms)
    all_items = copy.deepcopy(item_list.allItems)
    tortoise.drawItems(all_items, offset_x, offset_y)

    item_list.allItems = all_items
    item_list.saveToFile()

    new_map.generatePGM()

    tortoise.saveSVG()


if __name__ == '__main__':
    generate_maps()
