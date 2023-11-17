import numpy as np


class SRG:

    # dummy srg for now; will need a way to serialise/deserialise this later on.
    def __init__(self):
        objects = ["oven", "kettle", "toaster", "cushion", "sofa", "bed"]
        self.srg = {}
        for obj in objects:
            object_dict = {}
            for temp in objects:
                if temp == obj:
                    object_dict.update({temp: 0})
                elif temp in self.srg:
                    object_dict.update({temp: self.srg.get(temp).get(obj)})
                else:
                    object_dict.update({temp: np.random.uniform(0, 500)})
            self.srg.update({obj: object_dict})

        for item in self.srg:
            print(item, self.srg[item])

    def get_distance(self, obj, target):
        return self.srg.get(obj).get(target)
