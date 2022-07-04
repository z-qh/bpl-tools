import cv2 as cv


class point_2d:
    x = float(0)
    y = float(0)

    def __init__(self, x_, y_):
        self.x = x_
        self.y = y_
    def __str__(self):
        return "({:.2f},{:.2f})".format(self.x, self.y)

class ploygon_shape:
    points = []
    point_size = 0

    def __init__(self):
        self.points.clear()
        self.size = 0

    def __str__(self):
        ploygon_shape_str = ""
        for point in self.points:
            ploygon_shape_str += str(point)
            ploygon_shape_str += ", "
        return ploygon_shape_str


class frame:
    shapes = []
    shape_size = 0

    def __init__(self):
        self.shapes.clear()
        self.shape_size = 0

    def __str__(self):
        frame_str = ""
        for shape in self.shapes:
            frame_str += str(shape)
            frame_str += "\n"
        return frame_str


def load_frame_shape(filename: str):
    this_frame = frame()
    with open(filename) as file:
        lines = file.readlines()
        index = 0
        shape_size = int(lines[index].strip("\n").split(" ")[0])
        index += 1
        for i in range(shape_size):
            this_shape = ploygon_shape()
            this_shape.point_size = int(lines[index].strip("\n").split(" ")[0])
            index += 1
            for j in range(this_shape.point_size):
                p = point_2d( float(lines[index].strip("\n").split(' ')[0]), float(lines[index].strip("\n").split(' ')[1]) )
                print(p.x, p.y)
                index+=1
                this_shape.points.append(p)
            this_frame.shape_size += 1
            this_frame.shapes.append(this_shape)
        return this_frame

a1111 = load_frame_shape("/home/qh/ins_map_temp/match_graph/0.inss")

print(a1111)
print("123")