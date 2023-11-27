import numpy as np
import os


class PGM:
    def __init__(self):
        self.width = 500
        self.height = 500
        self.p_num = self.width * self.height
        self.arr = np.full(self.p_num, 205)

    def addRoom(self, x_range, y_range):

        for y in range(self.height):
            for x in range(self.width):
                if x_range[0] < x < x_range[1]:
                    if y_range[0] < y < y_range[1]:
                        self.arr[(y * self.width) + x] = 254

                if x == x_range[0] or x == x_range[1]:
                    if y_range[0] <= y <= y_range[1]:
                        self.arr[(y * self.width) + x] = 1
                if y == y_range[0] or y == y_range[1]:
                    if x_range[0] <= x <= x_range[1]:
                        self.arr[(y * self.width) + x] = 1

    def addDoor(self, coords):
        #maybe bigger doors? Setting the 10s to 15 seemed to make it better. Robot is struggling with the smaller doorways.
        x_range = [coords[0] - 10, coords[0] + 10]
        y_range = [coords[1] - 10, coords[1] + 10]
        for y in range(self.height):
            for x in range(self.width):
                if x_range[0] < x < x_range[1]:
                    if y_range[0] < y < y_range[1]:
                        self.arr[(y * self.width) + x] = 254

    def generatePGM(self):
        # open file for writing
        output_dir = os.path.join(os.path.dirname(__file__), "out")
        os.makedirs(output_dir, exist_ok=True)
        filename = 'world.pgm'
        file = open(os.path.join(output_dir, filename), 'wb')

        # define PGM Header
        pgm_header = 'P5' + ' ' + str(self.width) + ' ' + str(self.height) + ' ' + str(255) + '\n'

        pgm_header_byte = bytearray(pgm_header, 'utf-8')

        # write the header to the file
        file.write(pgm_header_byte)

        # write the data to the file
        img = np.reshape(self.arr, (self.height, self.width))

        for j in range(self.height):
            bnd = list(img[j, :])
            file.write(bytearray(bnd))  # for 8-bit data only

        file.close()
