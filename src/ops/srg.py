import math
import os
import json
import pandas


class SRG:

    # dummy srg for now; will need a way to serialise/deserialise this later on.
    def __init__(self, filename=None):
        if filename is None:
            current_file = os.path.dirname(__file__)
            df = pandas.read_csv(os.path.join(current_file, '../mapping/ObjectList.csv'))
            objects = df.Item.to_list()
            self._srg = {}
            dist, var, n = 5, 2, 1
            for obj in objects:
                object_dict = {}
                for temp in objects:
                    if temp == obj:
                        object_dict.update({temp: (0, 10, 0)})
                    elif temp in self._srg:
                        object_dict.update({temp: self._srg.get(temp).get(obj)})
                    else:
                        object_dict.update({temp: (dist, var, n)})
                self._srg.update({obj: object_dict})
        else:
            try:
                path = os.path.dirname(__file__)
                file = open(os.path.join(path, filename), "r")
                self._srg = json.load(file)
            except IOError:
                "Error: Could not find file"

    def get_distance(self, obj, target):
        return self._srg.get(obj).get(target)[0]

    def get_target_distribution(self, target):
        return self._srg.get(target)

    def save_in_file(self):
        try:
            output_dir = os.path.join(os.path.dirname(__file__), "out")
            os.makedirs(output_dir, exist_ok=True)
            filename = 'srg.json'
            # file = open(os.path.join(output_dir, filename), 'wb')

            with open(os.path.join(output_dir, filename), 'w') as file:
                json.dump(self._srg, file)

            # json.dump(self._srg, file)
        except IOError:
            print("Error: could ot create file")


class TrainingSRG(SRG):

    def __init__(self, filename=None):
        SRG.__init__(self, filename=filename)

    def update_weights(self, obj1, obj2, distance):
        print(f"{obj1} {obj2} ")
        mean, var, n = self._srg.get(obj1).get(obj2)

        if math.isnan(distance):
            return

        var = (n / (n + 1)) * (var + (((mean - distance) ** 2) / n + 1))
        mean = ((mean * n) + distance) / (n + 1)
        srg = self._srg
        obj1_dict = srg.get(obj1)
        obj1_dict.update({obj2: (mean, var, n + 1)})
        srg.update({obj1: obj1_dict})
        obj2_dict = self._srg.get(obj2)
        obj2_dict.update({obj1: (mean, var, n + 1)})
        srg.update({obj2: obj2_dict})
        self._srg = srg


if __name__ == "__main__":
    srg = SRG()
    srg2 = SRG("../out/srg.json")
