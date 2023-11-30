import os
import sys
import numpy as np

sys.path.insert(1, '/home/shrimpton/Documents/RoboticsAssignment2/src/ops')
sys.path.insert(1, '/home/shrimpton/Documents/RoboticsAssignment2/src/mapping')
import srg
import mapping

if __name__ == '__main__':
    num_to_train = 0 if len(sys.argv) <= 1 else int(sys.argv[1])
    training_srg = srg.TrainingSRG()
    for i in range(num_to_train):
        mapping.generate_maps()
        path = os.path.dirname(__file__)
        path = os.path.join(path, "../mapping/out/itemList.txt")
        with open(path, 'r') as file1:
            lines = file1.readlines()

        all_list = []
        for item in lines:
            obj, x, y = eval(item)
            all_list.append((obj, (x,y)))

        for obj1, (x1, y1) in all_list:
            for obj2, (x2, y2) in all_list:
                if obj1 == obj2:
                    continue
                distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                training_srg.update_weights(obj1, obj2, distance)
    training_srg.save_in_file()
