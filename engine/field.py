from vector import Vector

EPSILON = 1e-3

FIELD_CENTERED = True

class Field(object):
    '''This class is responsible for conversion between physical and on-screen coords'''


    def __init__(self, size, resolution):
        """
        size - tuple containing physical size of field
        resolution - size of the drawn image
        """
        self.size = size
        self.resolution = resolution
        if FIELD_CENTERED:
            self.left = -0.5 * size[0]
            self.bottom = -0.5 * size[1]
            self.right = 0.5 * size[0]
            self.top = 0.5 * size[1]
        else:
            self.left = 0
            self.bottom = 0
            self.right = size[0]
            self.top = size[1]
        self.width = self.right - self.left
        self.height = self.top - self.bottom


    def fit_on_screen(self, pos):
        rl = 0
        rw = self.resolution[0]
        rb = 0
        rh = self.resolution[1]
        return Vector(int(rl + rw * (pos.x - self.left) / self.width),
                      int(rb + rh * (1.0 - (pos.y - self.bottom) / self.height)))


    def screen_to_field(self, pos):
        return Vector(self.left + self.width * pos[0] / self.resolution[0],
                      self.top - self.height * pos[1] / self.resolution[1])


    def scale(self, dist):
        #if abs(self.resolution[0] / self.width - self.resolution[1] / self.height) > EPSILON:
        #    raise RuntimeError("Field has non-uniform scale")
        return int(dist * (self.resolution[0] / self.width))


    def resize_to_contain(self, points):
        """ Uniformly scale the field up so that all points are inside it."""

        cleft   = self.left
        cright  = self.right
        cbottom = self.bottom
        ctop    = self.top
        for p in points:
            cleft   = min(cleft,   p.x)
            cright  = max(cright,  p.x)
            cbottom = min(cbottom, p.y)
            ctop    = max(ctop,    p.y)
        scale_w = (cright - cleft) / self.width
        scale_h = (ctop - cbottom) / self.height
        scale = max(scale_w, scale_h)
        self.width *= scale
        self.height *= scale
        self.left *= scale
        self.right *= scale
        self.bottom *= scale
        self.top *= scale
        #field.width = field.right - field.left
        #field.height = field.top - field.bottom
