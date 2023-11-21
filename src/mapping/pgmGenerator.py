import uuid

import numpy as np
import os
class pgm:
    def __init__(self):
        self.width = 500
        self.height = 500
        self.p_num = self.width * self.height
        self.arr = np.full(self.p_num,205)
    def addRoom(self,xRange,yRange):

        for y in range(self.height):
            for x in range(self.width):
                if xRange[0] < x < xRange[1]:
                    if yRange[0] < y <yRange[1]:
                        self.arr[(y*self.width)+x] = 254

                if x ==xRange[0] or x == xRange[1]:
                    if yRange[0] <= y <=yRange[1]:
                        self.arr[(y * self.width) + x] = 1
                if y ==yRange[0] or y == yRange[1]:
                    if xRange[0] <= x <= xRange[1]:
                        self.arr[(y * self.width) + x] = 1

    def addDoor(self,coords):
        xRange = [coords[0]-10,coords[0]+10]
        yRange = [coords[1] - 10, coords[1] + 10]
        for y in range(self.height):
            for x in range(self.width):
                if xRange[0] < x < xRange[1]:
                    if yRange[0] < y < yRange[1]:
                        self.arr[(y * self.width) + x] = 254
    def generatePGM(self):
        # open file for writing
        output_dir = os.path.join(os.path.dirname(__file__), "out")
        os.makedirs(output_dir, exist_ok=True)
        filename = 'world.pgm'
        fout=open(os.path.join(output_dir, filename), 'wb')

        # define PGM Header
        pgmHeader = 'P5' + ' ' + str(self.width) + ' ' + str(self.height) + ' ' + str(255) +  '\n'

        pgmHeader_byte = bytearray(pgmHeader,'utf-8')

        # write the header to the file
        fout.write(pgmHeader_byte)

        # write the data to the file
        img = np.reshape(self.arr,(self.height,self.width))

        for j in range(self.height):
            bnd = list(img[j,:])
            fout.write(bytearray(bnd)) # for 8-bit data only

        fout.close()

test = pgm()
test.addRoom([120,150],[200,220])
test.addDoor([135,200])
test.generatePGM()