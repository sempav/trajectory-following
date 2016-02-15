import unittest
from engine.vector import Point, Vector
from engine.shapes import Ray, Circle, Segment, Polygon


class CircleTestCase(unittest.TestCase):
    def testTwoRayIntersections(self):
        c = Circle(Point(0.0, 0.0), 1.0)

        p = Ray(Point(-2.0, 0.0), Vector(1.0, 0.0)).intersect(c)
        self.assertEqual(p, Point(-1.0, 0.0))

        c = Circle(Point(0.0, 0.0), 2.0)
        p = Ray(Point(100.0, 100.0), Vector(-1.0, -1.0)).intersect(c)
        self.assertEqual(p, Point(2**0.5, 2**0.5))

        c = Circle(Point(10.0, 10.0), 4.0)
        p = Ray(Point(0.0, 0.0), Vector(1.0, 1.0)).intersect(c)
        self.assertEqual(p, Point(10.0 - 2 * 2**0.5, 10.0 - 2 * 2**0.5))


    def testOneRayIntersectionWithOrigInsideCircle(self):
        c = Circle(Point(0.0, 0.0), 1.0)

        p = Ray(Point(0.0, 0.0), Vector(1.0, 0.0)).intersect(c)
        self.assertEqual(p, Point(1.0, 0.0))

        p = Ray(Point(0.0, 0.0), Vector(1.0, 1.0)).intersect(c)
        self.assertEqual(p, Point(0.5 * 2**0.5, 0.5 * 2**0.5))


    def testOneRayIntersectionWithIncidentRay(self):
        c = Circle(Point(0.0, 0.0), 1.0)

        p = Ray(Point(-10.0, 1.0), Vector(1.0, 0.0)).intersect(c)
        self.assertEqual(p, Point(0.0, 1.0))
        
        p = Ray(Point(-10.0 - 0.5 * 2**0.5, -10.0 + 0.5 * 2**0.5), Vector(1.0, 1.0)).intersect(c)
        self.assertEqual(p, Point(-0.5 * 2**0.5, 0.5 * 2**0.5))


    def testNoRayIntersection(self):
        c = Circle(Point(10.0, 10.0), 1.0)

        p = Ray(Point(0.0, 0.0), Vector(1.0, 0.0)).intersect(c)
        self.assertIsNone(p)

        p = Ray(Point(0.0, 0.0), Vector(-1.0, -1.0)).intersect(c)
        self.assertIsNone(p)


    def testOneCircleIntersection(self):
        c = Circle(Point(0.0, 0.0), 1.0)
        f = Circle(Point(2.0, 0.0), 1.0).intersect(c)
        self.assertTrue(f)
        f = Circle(Point(2**0.5, 2**0.5), 1.0).intersect(c)
        self.assertTrue(f)


    def testTwoCircleIntersections(self):
        c = Circle(Point(0.0, 0.0), 1.0)
        f = Circle(Point(1.0, 0.0), 1.0).intersect(c)
        self.assertTrue(f)
        f = Circle(Point(1.0, 1.0), 1.0).intersect(c)
        self.assertTrue(f)


    def testSameCircleIntersection(self):
        c = Circle(Point(0.0, 0.0), 1.0)
        f = Circle(Point(0.0, 0.0), 1.0).intersect(c)
        self.assertTrue(f)


    def testNoCircleIntersection(self):
        c = Circle(Point(0.0, 0.0), 1.0)
        f = Circle(Point(100.0, 0.0), 1.0).intersect(c)
        self.assertIsNone(f)


class SegmentTestCase(unittest.TestCase):
    def testMiddleRayIntersection(self):
        s = Segment(Point(-1.0, 0.0), Point(1.0, 0.0))

        p = Ray(Point(-1.0, -1.0), Vector(1.0, 1.0)).intersect(s)
        self.assertEqual(p, Point(0.0, 0.0))

        p = Ray(Point(-2.0, -1.0), Vector(2.75, 1.0)).intersect(s)
        self.assertEqual(p, Point(0.75, 0.0))


    def testBoundsRayIntersection(self):
        s = Segment(Point(-1.0, 0.0), Point(1.0, 0.0))

        p = Ray(Point(-1.0, -1.0), Vector(0.0, 1.0)).intersect(s)
        self.assertEqual(p, Point(-1.0, 0.0))

        p = Ray(Point(-2.0, -2.0), Vector(3.0, 2.0)).intersect(s)
        self.assertEqual(p, Point(1.0, 0.0))

    def testNoRayIntersection(self):
        s = Segment(Point(-1.0, 0.0), Point(1.0, 0.0))

        p = Ray(Point(-1.0, -1.0), Vector(1.0, 0.0)).intersect(s)
        self.assertIsNone(p)

        p = Ray(Point(0.5, 1.0), Vector(1.0, 1.0)).intersect(s)
        self.assertIsNone(p)

        p = Ray(Point(0.0, 1.0), Vector(0.0, 1.0)).intersect(s)
        self.assertIsNone(p)

        s = Segment(Point(-1.0, -1.0), Point(1.0, -1.0))
        p = Ray(Point(0.0, 0.0), Vector(1.0, 1.0)).intersect(s)
        self.assertIsNone(p)

        s = Segment(Point(-1.0, 1.0), Point(-1.0, -1.0))
        p = Ray(Point(0.0, 0.0), Vector(1.0, 1.0)).intersect(s)
        self.assertIsNone(p)


    def testOneCircleIntersectionPoke(self):
        c = Circle(Point(0.0, 0.0), 1.0)
        f = Segment(Point(-2.0, 0.0), Point(0.0, 0.0)).intersect(c)
        self.assertIsNotNone(f)
        f = Segment(Point(-2.0, 0.1), Point(-0.2, 0.5)).intersect(c)
        self.assertIsNotNone(f)


    def testOneCircleIntersectionExitWound(self):
        # same as previous test, just with differently directed segments
        c = Circle(Point(0.0, 0.0), 1.0)
        f = Segment(Point(0.0, 0.0), Point(-2.0, 0.0)).intersect(c)
        self.assertIsNotNone(f)
        f = Segment(Point(-0.2, 0.5), Point(-2.0, 0.1)).intersect(c)
        self.assertIsNotNone(f)


    def testNoCircleIntersectionParallel(self):
        c = Circle(Point(0.0, 2.0), 1.0)
        f = Segment(Point(-1.0, 0.0), Point(1.0, 0.0)).intersect(c)
        self.assertIsNone(f)


    def testNoCircleIntersectionFallShort(self):
        c = Circle(Point(0.0, 0.0), 1.0)
        f = Segment(Point(-4.0, 0.0), Point(-3.0, 0.0)).intersect(c)
        self.assertIsNone(f)
        f = Segment(Point(-4.0, 1.0), Point(-3.0, -0.2)).intersect(c)
        self.assertIsNone(f)


    def testNoCircleIntersectionPast(self):
        c = Circle(Point(0.0, 0.0), 1.0)
        f = Segment(Point(-3.0, 0.0), Point(-4.0, 0.0)).intersect(c)
        self.assertIsNone(f)
        f = Segment(Point(3.0, 3.5), Point(4.1, 4.3)).intersect(c)
        self.assertIsNone(f)


    def testNoCircleIntersectionCompletelyInside(self):
        c = Circle(Point(0.0, 0.0), 1.0)
        f = Segment(Point(-0.5, 0.0), Point(0.5, 0.0)).intersect(c)
        self.assertIsNone(f)
        f = Segment(Point(-0.5, 0.5), Point(0.5, -0.5)).intersect(c)
        self.assertIsNone(f)


class PolygonTestCase(unittest.TestCase):
    def testOneRayIntersection(self):
        s = Polygon([Point(-1.0, -1.0), Point(1.0, -1.0),
                     Point(1.0, 1.0),   Point(-1.0, 1.0)])

        p = Ray(Point(0.0, 0.0), Vector(1.0, 0.0)).intersect(s)
        self.assertEqual(p, Point(1.0, 0.0))

        p = Ray(Point(0.0, 0.0), Vector(1.0, 1.0)).intersect(s)
        self.assertEqual(p, Point(1.0, 1.0))


    def testMultipleRayIntersections(self):
        s = Polygon([Point(-1.0, -1.0), Point(1.0, -1.0),
                     Point(1.0, 1.0),   Point(-1.0, 1.0)])

        p = Ray(Point(-2.0, 0.0), Vector(1.0, 0.5)).intersect(s)
        self.assertEqual(p, Point(-1.0, 0.5))

        p = Ray(Point(-5.0, -2.0), Vector(4.0, 1.0)).intersect(s)
        self.assertEqual(p, Point(-1.0, -1.0))

        p = Ray(Point(-3.0, 0.0), Vector(4.0, 1.0)).intersect(s)
        self.assertEqual(p, Point(-1.0, 0.5))


    def testNoRayIntersection(self):
        s = Polygon([Point(-1.0, -1.0), Point(1.0, -1.0),
                     Point(1.0, 1.0),   Point(-1.0, 1.0)])

        p = Ray(Point(-2.0, 0.0), Vector(1.0, 2.0)).intersect(s)
        self.assertIsNone(p)

        p = Ray(Point(2.0, 0.0), Vector(1.0, 0.0)).intersect(s)
        self.assertIsNone(p)

        p = Ray(Point(2.0, 2.0), Vector(1.0, 1.0)).intersect(s)
        self.assertIsNone(p)

        p = Ray(Point(2.0, 0.5), Vector(1.0, 0.1)).intersect(s)
        self.assertIsNone(p)


    def testNoCircleIntersection(self):
        s = Polygon([Point(-1.0, -1.0), Point(1.0, -1.0),
                     Point(1.0, 1.0),   Point(-1.0, 1.0)])
        f = Circle(Point(0.0, 0.0), 0.5).intersect(s)
        self.assertIsNone(f)
        f = Circle(Point(3.0, 3.0), 1.0).intersect(s)
        self.assertIsNone(f)


    def testOneCircleIntersection(self):
        s = Polygon([Point(-1.0, -1.0), Point(1.0, -1.0),
                     Point(1.0, 1.0),   Point(-1.0, 1.0)])
        f = Circle(Point(2.0, 0.0), 1.0).intersect(s)
        self.assertIsNotNone(f)
        f = Circle(Point(2.0, 2.0), 2**0.5).intersect(s)
        self.assertIsNotNone(f)


    def testMultipleCircleIntersections(self):
        s = Polygon([Point(-1.0, -1.0), Point(1.0, -1.0),
                     Point(1.0, 1.0),   Point(-1.0, 1.0)])
        f = Circle(Point(0.0, 0.0), 1.0).intersect(s)
        self.assertIsNotNone(f)
        f = Circle(Point(0.0, 0.0), 2**0.5).intersect(s)
        self.assertIsNotNone(f)
        f = Circle(Point(1.0, 0.0), 0.2).intersect(s)
        self.assertIsNotNone(f)
        f = Circle(Point(2.0, 2.0), 3.0).intersect(s)
        self.assertIsNotNone(f)
