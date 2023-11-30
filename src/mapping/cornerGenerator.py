from roomGenerator import *
import copy

"""
The code below is designed to generate the coordinates for each of the 4 corners of each room.
The design allows the corners to know if adjacent rooms can be places next to them to ensure
that as few rooms are generated with errors.
"""


def generateCorners(room_objects, ensuite):
    # begin by finding the main room of the house
    current = getHighestWeightedRoom(room_objects)
    current_room = room_objects[current]

    # form the corners of said room (no limits on where rooms can be placed yet
    upper_left_coord = [0, 0]
    upper_right_coord = [upper_left_coord[0] + current_room.length, upper_left_coord[1]]
    lower_right_coord = [upper_left_coord[0] + current_room.length, upper_left_coord[1] - current_room.width]
    lower_left_coord = [upper_left_coord[0], upper_left_coord[1] - current_room.width]

    lower_left_corner = Corner(lower_left_coord, [1, 0, 0, 0])
    upper_left_corner = Corner(upper_left_coord, [0, 1, 0, 0])
    upper_right_corner = Corner(upper_right_coord, [0, 0, 1, 0])
    lower_right_corner = Corner(lower_right_coord, [0, 0, 0, 1])

    current_room.setCorners([upper_right_corner, lower_right_corner, lower_left_corner, upper_left_corner])
    clone_room = current_room
    placed_rooms = [current_room]

    # remove room from list of unplaced rooms since it has been placed
    del room_objects[current]

    doors = []
    weighted_rooms = []

    # order the rooms by weight
    for _ in range(len(room_objects)):
        index = getHighestWeightedRoom(room_objects)
        weighted_rooms.append((room_objects[index]))
        del room_objects[index]

    # while loop until all rooms have been placed, or we can't place any more rooms
    while len(weighted_rooms) > 0:
        index = 0
        available_rooms = []

        # here we check if the next room in the list can be placed. If not we can skip it for layer
        while True:
            found = False
            for i in range(len(placed_rooms)):
                if placed_rooms[i].type in weighted_rooms[index].canBeConnectedTo:
                    available_rooms.append(i)
                    found = True

            if found:
                break
            index += 1
        current_room = weighted_rooms[index]
        failure = 0
        available = []
        location_to_place = None
        corner_to_place = None
        # assuming we have a room that can be placed we try to place it 100 times until it has either been placed
        # correctly or we exit (there isn't a valid location for the room
        while failure < 100:
            location_to_place = random.choice(available_rooms)

            try:
                if len(location_to_place) == 1:
                    location_to_place = location_to_place[0]
            except:
                pass

            # here we identify the corner of the room we are going to place from
            corner_to_place = random.choice(placed_rooms[location_to_place].corners)

            # pick a direction
            available = []
            for i in range(len(corner_to_place.occupied)):
                lower = corner_to_place.occupied[i - 1]
                try:
                    upper = corner_to_place.occupied[i + 1]
                except:
                    upper = corner_to_place.occupied[0]
                if lower == 1 or upper == 1:
                    if corner_to_place.occupied[i] != 2:
                        available.append(i)

            if len(available) > 0:
                break
            else:
                failure += 1

        # desired_direction is the direction from the corner we can go.
        # e.g. the top left corner can go up or left
        if len(available) == 0:
            break
        elif len(available) == 1:
            desired_direction = available[0]
        else:
            desired_direction = random.choice(available)

        # 0 = upperRight, 1 = lowerRight, 2 = lowerLeft 3 = upperLeft
        corner_index = placed_rooms[location_to_place].corners.index(corner_to_place)

        backup = copy.deepcopy(placed_rooms[location_to_place].corners[corner_index].coords)

        upper_left_coord = corner_to_place.coords
        # here we generate the corners for the new room based on the corner we built the room from
        if corner_index == 0:
            if desired_direction == 3:
                upper_left_coord[0] = upper_left_coord[0] - current_room.length
                upper_left_coord[1] = upper_left_coord[1] + current_room.width
                doors.append([backup[0] - 1, backup[1]])
            else:
                doors.append([backup[0], backup[1] - 1])
        elif corner_index == 1:
            if desired_direction == 2:
                upper_left_coord[0] = upper_left_coord[0] - current_room.length
                doors.append([backup[0] - 1, backup[1]])

            else:
                upper_left_coord[1] = upper_left_coord[1] + current_room.width
                doors.append([backup[0], backup[1] + 1])

        elif corner_index == 2:
            if desired_direction == 1:
                upper_left_coord[0] = upper_left_coord[0] - current_room.length
                upper_left_coord[1] = upper_left_coord[1] + current_room.width
                doors.append([backup[0], backup[1] + 1])

            else:
                doors.append([backup[0] + 1, backup[1]])

        elif corner_index == 3:
            if desired_direction == 0:
                upper_left_coord[1] = upper_left_coord[1] + current_room.width
                doors.append([backup[0] + 1, backup[1]])
            else:
                upper_left_coord[0] = upper_left_coord[0] - current_room.length
                doors.append([backup[0], backup[1] - 1])

        placed_rooms[location_to_place].corners[corner_index].coords = backup

        upper_right_coord = [upper_left_coord[0] + current_room.length, upper_left_coord[1]]
        lower_right_coord = [upper_left_coord[0] + current_room.length, upper_left_coord[1] - current_room.width]
        lower_left_coord = [upper_left_coord[0], upper_left_coord[1] - current_room.width]

        lower_left_corner = Corner(lower_left_coord, [1, 0, 0, 0])
        upper_left_corner = Corner(upper_left_coord, [0, 1, 0, 0])
        upper_right_corner = Corner(upper_right_coord, [0, 0, 1, 0])
        lower_right_corner = Corner(lower_right_coord, [0, 0, 0, 1])
        #####################################
        # update corners based on the stupid diagrams I made
        if corner_to_place.occupied[0] == 1:
            upper_right_corner.occupied = corner_to_place.occupied
        if corner_to_place.occupied[1] == 1:
            lower_right_corner.occupied = corner_to_place.occupied
        if corner_to_place.occupied[2] == 1:
            lower_left_corner.occupied = corner_to_place.occupied
        if corner_to_place.occupied[3] == 1:
            upper_left_corner.occupied = corner_to_place.occupied
        #####################################
        corners = [lower_left_corner, upper_left_corner, upper_right_corner, lower_right_corner]
        corners[(desired_direction + 2) % 4].occupied = corner_to_place.occupied
        corners[(desired_direction + 2) % 4].occupied[desired_direction] = 1
        corners[(desired_direction + 2) % 4].occupied[(desired_direction + 2) % 4] = 2

        #####################################
        # baseline coords generated, now they need to be modified such that they don't allow for incorrect rooms
        current_room.setCorners([upper_right_corner, lower_right_corner, lower_left_corner, upper_left_corner])

        placed_rooms[location_to_place].corners[corner_index].occupied[desired_direction] = 2
        placed_rooms[location_to_place].corners[desired_direction].occupied[corner_index] = 2

        # preventing bedrooms from having 2 ensuites
        if current_room.type == "ensuite":
            for i in range(4):
                current_room.corners[i].occupied = [2, 2, 2, 2]
            if placed_rooms[location_to_place].type == "bedroom":
                for i in range(4):
                    placed_rooms[location_to_place].corners[i].occupied = [2, 2, 2, 2]

        if not ensuite:
            if current_room.type == "bedroom":
                for i in range(4):
                    current_room.corners[i].occupied = [2, 2, 2, 2]

        if current_room.type == "bathroom":
            for i in range(4):
                current_room.corners[i].occupied = [2, 2, 2, 2]
        placed_rooms.append(current_room)
        del weighted_rooms[index]

    placed_rooms[0] = clone_room
    # return all the placed room objects and their corresponding doors
    return placed_rooms, doors
