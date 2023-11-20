import numpy as np


class SRG:

    # dummy srg for now; will need a way to serialise/deserialise this later on.
    def __init__(self):
        objects = ["oven", "kettle", "toaster", "cushion", "sofa", "bed"]
        self._srg = {}
        dist, var, n = 100, 10, 1
        for obj in objects:
            object_dict = {}
            for temp in objects:
                if temp == obj:
                    object_dict.update({temp: (0, 0, 0)})
                elif temp in self._srg:
                    object_dict.update({temp: self._srg.get(temp).get(obj)})
                else:
                    object_dict.update({temp: (dist, var, n)})
            self._srg.update({obj: object_dict})

    def get_distance(self, obj, target):
        return self._srg.get(obj).get(target)[0]

    def get_target_distribution(self, target):
        return self._srg.get(target)


class TrainingSRG(SRG):

    def __init__(self):
        SRG.__init__(self)

    def update_weights(self, obj1, obj2, distance):
        mean, var, n = self._srg.get(obj1).get(obj2)
        print("old mean", mean, "old var", var, "distance", distance)
        var = (n/(n+1)) * (var + (((mean-distance)**2)/n+1))
        mean = ((mean * n) + distance) / (n + 1)
        print("new mean", mean, "new var", var)
        srg = self._srg
        obj1_dict = srg.get(obj1)
        obj1_dict.update({obj2: (mean, var, n + 1)})
        srg.update({obj1: obj1_dict})
        obj2_dict = self._srg.get(obj2)
        obj2_dict.update({obj1: (mean, var, n + 1)})
        srg.update({obj2: obj2_dict})
        self._srg = srg
