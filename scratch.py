#!/usr/bin/python
#
# Steering Behaviours
# Robert Nowotniak, Michal Wysokinski
#

from visual import *
from visual.controls import *
from random import uniform, randint, random
import math, sys
import numpy as num


class Vehicle(box):

    def __init__(self, **args):
        super(Vehicle, self).__init__(**args)
        if 'color' not in args:
            self.color = color.yellow
        self.pos = vector(
            random() * 20 - 10, 0, random() * 20 - 10)
        self.sizeprop = 1.5
        self.size = (self.sizeprop * 0.4, 0.4, 0.4)
        # Some default values
        self.mass = 0.2
        self.force = vector(0, 0, 0)
        self.max_speed = 5
        self.vel = vector(0, 0, 0)
        self.max_force = 3
        self.steering = DoNothing()
        self.arrow = arrow(pos=self.pos, axis=(1, 0, 0), shiftwidth=0.5)

    def hud(self):
        self.arrow.visible = not self.arrow.visible

    def __setattr__(self, name, value):
        if name == 'vel' and mag(value) > self.max_speed:
            value = norm(value) * self.max_speed
        object.__setattr__(self, name, value)
        if name == 'visible':
            self.arrow.visible = False
            return
        if hasattr(self, 'vel') and mag(self.vel) > 0:
            object.__setattr__(self, 'axis', self.width * self.sizeprop * norm(self.vel))
        if hasattr(self, 'arrow') and hasattr(self, 'force'):
            self.arrow.pos = self.pos
            if name == 'force':
                force = self.force
                if mag(force) > 3:
                    force = 3 * norm(force)
                self.arrow.axis = force


#
# Some 2D Primitives
#
def circle(pos, radius, precision=0.1):
    i = 0.0
    circle = curve()
    end = 2 * math.pi + precision

    while i < end:
        x = pos[0] + radius * math.cos(i)
        z = pos[2] + radius * math.sin(i)
        circle.append((x, 0, z))
        i += precision

    return circle


def segment(a, b):
    seg = curve()
    seg.append((a[0], 0, a[1]))
    seg.append((b[0], 0, b[1]))
    return seg


def segment3(a, b):
    seg = curve()
    seg.append(a)
    seg.append(b)
    return seg


def segmentCircleIntersection3(a, o, r):
    bok = vector(a[1]) - vector(a[0])
    a = a[0]

    for n in xrange(0, 11):
        punkt = vector(a + bok * n / 10.0)
        if mag(punkt - o) <= r:
            return True

    return False


def rectCrircleIntersection3(a, b, c, d, o, r):
    if segmentCircleIntersection3(a.pos, o, r):
        return True
    if segmentCircleIntersection3(b.pos, o, r):
        return True
    if segmentCircleIntersection3(c.pos, o, r):
        return True
    if segmentCircleIntersection3(d.pos, o, r):
        return True

    return False


#
# Steering Behaviour strategy implementations
#
class SteeringBehaviour:

    def calculate(self, obj):
        pass

    def draw(self):
        pass


class Flee(SteeringBehaviour):

    def calculate(self, obj):
        return -norm(tar.pos - obj.pos) * obj.max_speed - obj.vel


class Seek(SteeringBehaviour):

    def __init__(self, target):
        self.target = target

    def calculate(self, obj):
        return norm(self.target.pos - obj.pos) * obj.max_speed - obj.vel


class Arrival(SteeringBehaviour):
    slowing_distance = 5.0

    def calculate(self, obj):
        target_offset = tar.pos - obj.pos
        distance = mag(target_offset)
        ramped = obj.max_speed * (distance / Arrival.slowing_distance)
        clipped = ramped
        if obj.max_speed < clipped:
            clipped = obj.max_speed
        desired = (clipped / distance) * target_offset
        return desired - obj.vel


class Wander(SteeringBehaviour):
    constraint_circle_radius = 1.0
    constraint_circle_offset = 1.41
    force_diff_mag = 0.71

    def __init__(self, obj):
        self.object = obj
        self.object.force = norm(self.object.axis) * self.constraint_circle_offset

        self.constraint_circle = circle(
            self.object.pos + norm(self.object.axis) * self.constraint_circle_offset,
            self.constraint_circle_radius)
        self.constraint_circle.color = color.blue

        self.force_diff_circle = circle(self.object.pos + self.object.force,
                                        self.force_diff_mag)
        self.force_diff_circle.color = color.green

        end = self.object.pos + self.object.force
        self.force_line = segment((self.object.pos.x, self.object.pos.z), (end[0], end[2]))
        self.force_line.color = color.red

    def __draw(self, force):
        if not self.object.arrow.visible:
            self.constraint_circle.visible = False
            self.force_diff_circle.visible = False
            self.force_line.visible = False
            return

        # self.object.arrow.visible = False
        self.constraint_circle.visible = False
        self.force_diff_circle.visible = False
        self.force_line.visible = False
        end = self.object.pos + force

        # narysuj constraint circle
        self.constraint_circle = circle(
            self.object.pos + norm(self.object.axis) * self.constraint_circle_offset,
            self.constraint_circle_radius)
        self.constraint_circle.color = color.blue

        # narysuj okrag zmiany sily
        self.force_diff_circle = circle(end, self.force_diff_mag)
        self.force_diff_circle.color = color.green

        # narysuj linie sily
        self.force_line = segment((self.object.pos.x, self.object.pos.z), (end[0], end[2]))
        self.force_line.color = color.red

    def calculate(self, obj):
        versor_x = norm(self.object.axis)
        versor_z = norm(self.object.axis.cross(vector(0, 1, 0)))

        # losuj zmiane wektora
        diff = norm(vector(random(), 0, random())) * self.force_diff_mag
        if randint(0, 1) == 0:
            diff.x = -diff.x
        if randint(0, 1) == 0:
            diff.z = -diff.z

        # nowa wartosc sily
        force = self.object.force + diff
        x = force.dot(versor_x)

        if x <= self.constraint_circle_offset - self.constraint_circle_radius:
            x = self.constraint_circle_offset - self.constraint_circle_radius
            z = 0
        elif x >= self.constraint_circle_offset + self.constraint_circle_radius:
            x = self.constraint_circle_offset + self.constraint_circle_radius
            z = 0
        else:
            x -= self.constraint_circle_offset
            if self.constraint_circle_radius > x:
                z = math.sqrt(self.constraint_circle_radius ** 2 - x ** 2)
            else:
                z = math.sqrt(x ** 2 - self.constraint_circle_radius ** 2)

            if force.dot(versor_z) < 0:
                z = -z
            x += self.constraint_circle_offset

        # wykonaj zmiane ukladu wspolrzednych
        f = num.mat([[x], [0], [z]])
        angle = diff_angle(vector(1, 0, 0), versor_x)
        if cross(vector(1, 0, 0), versor_x).y < 0:
            angle = -angle

        R = [[cos(angle), 0, sin(angle)], [0, 1, 0], [-sin(angle), 0, cos(angle)]]
        R = num.mat(R)
        v = R * f
        force = vector(v[0, 0], v[1, 0], v[2, 0])

        self.__draw(force)
        return force

    def __del__(self):
        self.constraint_circle.visible = False
        self.force_diff_circle.visible = False
        self.force_line.visible = False


class DoNothing(SteeringBehaviour):

    def calculate(self, obj):
        return vector(0, 0, 0)


class PathFollowing(SteeringBehaviour):
    points = []

    def __init__(self):
        if not PathFollowing.points:
            PathFollowing.points = []
            for x in xrange(4):
                p = sphere(radius=0.3, color=color.red)
                p.pos = (random() * 20 - 10, 0, random() * 20 - 10)
                PathFollowing.points.append(p)
        self.active = 0
        self.seek = Seek(PathFollowing.points[0])

    def __del__(self):
        for p in PathFollowing.points:
            p.visible = False
        PathFollowing.points = []

    def calculate(self, obj):
        if mag(obj.pos - PathFollowing.points[self.active].pos) < 0.1:
            self.active = (self.active + 1) % len(PathFollowing.points)
            self.seek.target = PathFollowing.points[self.active]
        return self.seek.calculate(obj)


class ObstacleAvoidance(SteeringBehaviour):
    sensor_length_factor = 1.0
    lateral_force_factor = 2.0
    breaking_force_factor = 0.1

    def __init__(self, obj):
        self.object = obj
        self.sensor_width = obj.width
        self.a = segment3((0, 0, 0), (0, 0, 0))
        self.b = segment3((0, 0, 0), (0, 0, 0))
        self.c = segment3((0, 0, 0), (0, 0, 0))
        self.d = segment3((0, 0, 0), (0, 0, 0))

    def draw(self, versor_x, versor_z):
        zoff = versor_z * self.object.width / 2.0
        xoff = versor_x * self.sensor_length_factor * mag(self.object.vel)

        self.a.visible = False
        self.b.visible = False
        self.c.visible = False
        self.d.visible = False

        self.a = segment3(self.object.pos - zoff, self.object.pos + zoff)
        self.b = segment3(self.object.pos + zoff, self.object.pos + zoff + xoff)
        self.c = segment3(self.object.pos + zoff + xoff, self.object.pos - zoff + xoff)
        self.d = segment3(self.object.pos - zoff + xoff, self.object.pos - zoff)

        if not self.object.arrow.visible:
            self.a.visible = False
            self.b.visible = False
            self.c.visible = False
            self.d.visible = False

    # sila hamowania zmienia sie kwadratowo
    # sila boczna zmienia sie liniowo
    def calculate(self, objjjj):
        global obstacles
        versor_x = norm(self.object.axis)
        versor_z = norm(self.object.axis.cross(vector(0, 1, 0)))
        self.draw(versor_x, versor_z)
        force = vector(1, 0, 0)
        sensor_length = self.sensor_length_factor * mag(self.object.vel)

        for o in obstacles:
            if o.red_counter > 0:
                if o.red_counter == 1:
                    o.color = color.white
                else:
                    o.red_counter -= 1

            if rectCrircleIntersection3(self.a, self.b, self.c, self.d, o.pos, o.radius) == True:
                o.color = color.green
                o.red_counter = 5

                # oblicz sile ucieczki
                dist = sensor_length - math.fabs(dot(vector(o.pos - self.object.pos), versor_x)) + o.radius
                if dist <= 0:
                    dist = 0
                z = self.lateral_force_factor * dist
                x = -dist * dist

                a = cross(vector(o.pos - self.object.pos).norm(), versor_x).y

                if a > 0:
                    z = -z

                    # wykonaj zmiane ukladu
                f = num.mat([[x], [0], [z]])
                angle = diff_angle(vector(1, 0, 0), versor_x)
                if cross(vector(1, 0, 0), versor_x).y < 0:
                    angle = -angle

                R = [[cos(angle), 0, sin(angle)], [0, 1, 0], [-sin(angle), 0, cos(angle)]]
                R = num.mat(R)
                v = R * f
                return vector(v[0, 0], v[1, 0], v[2, 0])

        return vector(1, 0, 0)

    def __del__(self):
        self.a.visible = False
        self.b.visible = False
        self.c.visible = False
        self.d.visible = False


class WanderAndObstacleAvoidance(SteeringBehaviour):
    def __init__(self, obj):
        self.wander = Wander(obj)
        self.oa = ObstacleAvoidance(obj)
        self.object = obj

    def calculate(self, objjjj):
        res = self.oa.calculate(objjjj)
        zero = vector(1, 0, 0)

        if res.astuple() == zero.astuple():
            return self.wander.calculate(objjjj)
        else:
            return res


class Separate(SteeringBehaviour):
    neighbourhood = 5

    def calculate(self, obj):
        force = vector(0, 0, 0)
        for v in vehicles:
            if obj is v:
                continue
            target_offset = obj.pos - v.pos
            distance = mag(target_offset)
            if mag(target_offset) > Separate.neighbourhood:
                continue
            ramped = (distance - Separate.neighbourhood / 2.0) \
                     * (2.0 * obj.max_speed / Separate.neighbourhood)
            if ramped > obj.max_speed:
                ramped = obj.max_speed
            desired = -ramped * (target_offset / distance)

            f = desired - obj.vel
            force = force + f
        return force


class SeparateSeek(SteeringBehaviour):

    def __init__(self, tar):
        self.target = tar
        self.seek = Seek(tar)
        self.separate = Separate()

    def calculate(self, obj):
        return 2 * self.seek.calculate(obj) + self.separate.calculate(obj)


#
# Display area
#
scene = display(title="Steering behaviours. R.Nowotniak, M.Wysokinski",
                width=640, height=600);
scene.forward = (0.5, -1, -1)
scene.ambient = 0.4;

# Grid
for i in range(11):
    curve(pos=[(2 * i - 10, 0, -10), (2 * i - 10, 0, 10)], color=color.cyan)
    curve(pos=[(-10, 0, 2 * i - 10), (10, 0, 2 * i - 10)], color=color.cyan)
# box(pos=(0,-0.6,0),width=20,length=20,height=1,color=(0,0,0.1))

#
# Target
#
tar = sphere(color=color.blue, radius=0.5, pos=(7, 0, -5.5));


def obstacle():
    r = random() * 2;
    if r < 0.4:
        r = 0.4

    pos = (random() * 20 - 10, 0, random() * 20 - 10)
    o = sphere(pos=pos, radius=r, color=color.white);
    o.red_counter = 0
    return o


animate = True
wrap = False
rotate = False
rotate_angle = 0
drag = None

obstacles = []

c = controls(title='Settings',
             x=0, y=400, width=500, height=700, range=50)

m1 = menu(pos=(0, 30, 0), height=6, width=45, text='Choose steering behaviour')

vehicles = []


def select_scene(n):
    global vehicles
    for v in vehicles:
        v.visible = False
        del v.steering
    quantities = (3, 3, 3, 1, 3, 1, 7, 4, 1)
    vehicles = []
    for i in xrange(quantities[n - 1]):
        v = Vehicle()
        vehicles.append(v)
        if n == 1:
            v.steering = Seek(tar)
        elif n == 2:
            v.steering = Flee()
        elif n == 3:
            v.steering = Arrival()
        elif n == 4:
            v.steering = Wander(v)
        elif n == 5:
            v.steering = PathFollowing()
        elif n == 6:
            v.steering = ObstacleAvoidance(v)
        elif n == 7:
            v.steering = Separate()
        elif n == 8:
            v.steering = SeparateSeek(tar)
        elif n == 9:
            v.steering = WanderAndObstacleAvoidance(v)


m1.items.append(('Seek', lambda: select_scene(1)));
m1.items.append(('Flee', lambda: select_scene(2)));
m1.items.append(('Arrival', lambda: select_scene(3)));
m1.items.append(('Wander', lambda: select_scene(4)));
m1.items.append(('Path following', lambda: select_scene(5)));
m1.items.append(('Obstacle Avoidance', lambda: select_scene(6)));
m1.items.append(('---------', None))
m1.items.append(('Separate', lambda: select_scene(7)));
m1.items.append(('Separate+Seek', lambda: select_scene(8)));
m1.items.append(('Wander + Obstacle Avoidance', lambda: select_scene(9)));

select_scene(6)

import time

start_time = time.time()

dt = 0.00333333

while true:
    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time
    rate(25);

    c.interact()

    # Handle keyboard events
    if scene.kb.keys:
        s = scene.kb.getkey()
        if s == 'q':
            sys.exit()
        elif 'h' == s:
            for v in vehicles:
                v.hud()
        if s == ' ':
            animate = not animate
        elif s == 'w':
            wrap = not wrap
        elif s == 'r':
            rotate = not rotate
        elif s == 'o':
            obstacles.append(obstacle())
        elif s == 'c':
            for o in obstacles:
                o.visible = False;
            obstacles = []

    # Handle mouse events
    if scene.mouse.events:
        e = scene.mouse.getevent()
        if drag and (e.drop or e.click):
            dp = scene.mouse.project(normal=scene.up)
            if dp:
                drag.pos = dp
            drag = None
            scene.cursor.visible = True
        elif e.pick:
            drag = e.pick
            scene.cursor.visible = False
    if drag:
        dp = scene.mouse.project(normal=scene.up)
        if dp:
            drag.pos = dp

    if not animate:
        continue

    for v in vehicles:
        # Get steering force acording to the current behaviour
        v.force = v.steering.calculate(v)

        # Update acceleration, velocity and position
        v.acc = v.force / v.mass;
        v.vel = v.vel + v.acc * dt;
        v.pos = v.pos + v.vel * dt

        if wrap:
            if v.pos.x < -10:
                v.pos.x = v.pos.x + 20
            elif v.pos.x > 10:
                v.pos.x = v.pos.x - 20
            if v.pos.z < -10:
                v.pos.z = v.pos.z + 20
            elif v.pos.z > 10:
                v.pos.z = v.pos.z - 20

        if mag(tar.pos - v.pos) < 0.5:
            tar.pos.x = random() * 20 - 10;
            tar.pos.z = random() * 20 - 10;
            # tar.pos.y = random() * 20 - 10;

    if rotate:
        rotate_angle = rotate_angle + 0.1 * math.pi * dt;
        scene.forward = (-1.0 * sin(rotate_angle), -1, -1.0 * cos(rotate_angle))