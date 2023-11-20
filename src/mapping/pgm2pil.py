from PIL import Image
import numpy

def pgm2pil(filename):

    try:
        inFile = open(filename)

        header = None
        size = None
        maxGray = None
        data = []

        for line in inFile:
            stripped = line.strip()

            if stripped[0] == '#':
                continue
            elif header == None:
                if stripped != 'P2': return None
                header = stripped
            elif size == None:
                size = map(int, stripped.split())
            elif maxGray == None:
                maxGray = int(stripped)
            else:
                for item in stripped.split():
                    data.append(int(item.strip()))

        data = numpy.reshape(data, (size[1],size[0]))/float(maxGray)*255
        return numpy.flipud(data)

    except:
        pass

    return None

def imageOpenWrapper(fname):
    pgm = pgm2pil(fname)
    if pgm is not None:
        return Image.fromarray(pgm)
    return origImageOpen(fname)

origImageOpen = Image.open
Image.open = imageOpenWrapper
