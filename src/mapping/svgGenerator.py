from svg_turtle import SvgTurtle
import os.path


class PogTurtle:
    def __init__(self):
        self.t = SvgTurtle(1000, 1000)
        self.t.hideturtle()
        self.t.speed(0)
        self.startup()

    def startup(self):
        self.t.pencolor("pink")
        for _ in range(4):
            self.t.forward(500)
            self.t.left(90)
        self.t.color("white")
        self.t.setposition(-500, -500)
        self.t.begin_fill()
        for x in range(4):
            self.t.forward(1000)
            self.t.left(90)
        self.t.end_fill()
        self.t.setposition(-490, -490)

        self.t.pencolor("black")
        self.t.write("TOMISPOG", font=("Comic Sans MS", 20, "normal"), align="left")


    def drawRoom(self, corners, name):
        self.t.pencolor("red")
        self.t.penup()
        self.t.setposition(corners[0][0] * 20, corners[0][1] * -20)
        self.t.pendown()
        for corner_index in range(len(corners)):
            self.t.setposition(corners[len(corners) - corner_index - 1][0] * 20,
                               corners[len(corners) - corner_index - 1][1] * -20)


        mid_x = (corners[1][0] + corners[3][0]) / 2
        mid_y = (corners[1][1] + corners[3][1]) / 2
        print(corners[3],corners[1],(mid_x,mid_y))
        self.t.penup()
        self.t.setposition(mid_x * 20, mid_y * -20)
        self.t.pendown()
        self.t.write(name, font=("Verdana", 5, "normal"), align="center")

    def drawOverlap(self, corner):
        self.t.pencolor("blue")
        self.t.penup()
        self.t.setposition(corner.coords[0] * 20, corner.coords[1] * -20)
        self.t.pendown()
        self.t.pensize(5)
        self.t.forward(1)
        self.t.pensize(1)

    def drawItems(self, all_items, offset_x, offset_y):
        self.t.pensize(1)
        self.t.pencolor("black")
        for i in range(len(all_items)):
            self.t.penup()
            self.t.setposition(all_items[i][1] * 20, all_items[i][2] * -20)
            self.t.pendown()
            self.t.forward(1)
            all_items[i] = (all_items[i][0], (round(all_items[i][1] * 20) + offset_x),
                            (round(all_items[i][2] * 20) + offset_y))

    def saveSVG(self):
        output_dir = os.path.join(os.path.dirname(__file__), "out")
        os.makedirs(output_dir, exist_ok=True)
        filename = output_dir + '/turtleMap.svg'
        self.t.save_as(filename)
