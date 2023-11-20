import numpy as np


class SRG:

    # dummy srg for now; will need a way to serialise/deserialise this later on.
    def __init__(self):
        objects = ["oven", "kettle", "toaster", "cushion", "sofa", "bed"]
        self._srg = {}
        for obj in objects:
            object_dict = {}
            for temp in objects:
                if temp == obj:
                    object_dict.update({temp: 1})
                elif temp in self._srg:
                    object_dict.update({temp: self._srg.get(temp).get(obj)})
                else:
                    object_dict.update({temp: np.random.uniform(0, 250)})
            self._srg.update({obj: object_dict})

    def get_distance(self, obj, target):
        return self._srg.get(obj).get(target)


class TrainingSRG(SRG):

    def __init__(self):
        SRG.__init__(self)


    def update_weights(self, data_idk):
