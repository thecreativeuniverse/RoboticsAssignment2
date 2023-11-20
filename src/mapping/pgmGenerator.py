import numpy as np
import random

#empty = 205
#movable space =254
#wall =0



# define the width  (columns) and height (rows) of your image
width = 600
height = 600

p_num = width * height
arr = np.random.randint(0,255,p_num)

for i in range(len(arr)):
    arr[i] = [205,254,0][i%3]

# open file for writing
filename = 'test.pgm'
fout=open(filename, 'wb')

# define PGM Header
pgmHeader = 'P5' + ' ' + str(width) + ' ' + str(height) + ' ' + str(255) +  '\n'

pgmHeader_byte = bytearray(pgmHeader,'utf-8')

# write the header to the file
fout.write(pgmHeader_byte)

# write the data to the file
img = np.reshape(arr,(height,width))

for j in range(height):
    bnd = list(img[j,:])
    fout.write(bytearray(bnd)) # for 8-bit data only


fout.close()
