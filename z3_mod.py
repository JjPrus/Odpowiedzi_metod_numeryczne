from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import time
import numpy as np

# zmienne pomocnicze
pointSize = 5
windowSize = 200
clearColor = [0.0, 0.0, 0.0]
q = np.array([0.0, 0.0, 0.0])
r = np.array([1.0, 2.0, 10.0])
yaw = 15
pitch = 10
camspeed = 0.01
key = 0

pixelMapR = [[clearColor[0] for y in range(windowSize)] for x in range(windowSize)]
pixelMapG = [[clearColor[1] for y in range(windowSize)] for x in range(windowSize)]
pixelMapB = [[clearColor[2] for y in range(windowSize)] for x in range(windowSize)]


class OP:  # parametry projekcji
    l = -10
    r = 10
    b = -10
    t = 10
    n = 10
    f = 100


def clearMap(color):
    global pixelMapR, pixelMapG, pixelMapB
    for i in range(windowSize):
        for j in range(windowSize):
            pixelMapR[i][j] = color[0]
            pixelMapG[i][j] = color[1]
            pixelMapB[i][j] = color[2]


# funkcja rysująca zawartość macierzy pixelMap
def paint():
    glClear(GL_COLOR_BUFFER_BIT)
    glBegin(GL_POINTS)
    for i in range(windowSize):
        for j in range(windowSize):
            glColor3f(pixelMapR[i][j], pixelMapG[i][j], pixelMapB[i][j])
            glVertex2f(0.5 + 1.0 * i, 0.5 + 1.0 * j)
    glEnd()
    glFlush()


# inicjalizacja okna
glutInit()
glutInitWindowSize(windowSize*pointSize, windowSize*pointSize)
glutInitWindowPosition(0, 0)
glutCreateWindow(b"Lab04")
glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB)

# inicjalizacja wyświetlania
glMatrixMode(GL_PROJECTION)
glLoadIdentity()
gluOrtho2D(0.0, windowSize, 0.0, windowSize)
glMatrixMode(GL_MODELVIEW)
glLoadIdentity()
glutDisplayFunc(paint)
glutIdleFunc(paint)
glClearColor(1.0, 1.0, 1.0, 1.0)
glEnable(GL_PROGRAM_POINT_SIZE)
glPointSize(pointSize)


def cupdate(step = 0.1):
    global tick
    ltime = time.clock()
    if ltime < tick + step:
        return False
    tick = ltime
    return True


def odcinek(x1, y1, x2, y2, R, G, B): # odcinek w 2d
    global pixelMapR
    global pixelMapG
    global pixelMapB
    if x2 == x1 and y2 == y1:
        x1 = round(x1)
        y1 = round(y1)
        if 0 <= x1 < windowSize:
            if 0 <= y1 < windowSize:
                pixelMapR[x1][y1] = R
                pixelMapR[x1][y1] = G
                pixelMapR[x1][y1] = B
        return
    ony = False
    d1 = None
    d2 = None
    if x2 == x1:
        d2 = 0
    elif y2 == y1:
        d1 = 0
    else:
        d2 = (x2 - x1) / (y2 - y1)
        if not -1 < d2 < 1:
            d1 = 1 / d2
    if d1 is not None:
        d = d1
        if x1 > x2:
            xtmp = x1; x1 = x2; x2 = xtmp
            ytmp = y1; y1 = y2; y2 = ytmp
        y = y1 - d
        for x in range(int(x1), int(x2)+1):
            y = y + d
            dcx = x
            dcy = int(y)
            if 0 <= dcx < windowSize:
                if 0 <= dcy < windowSize:
                    pixelMapR[dcx][dcy] = R
                    pixelMapG[dcx][dcy] = G
                    pixelMapB[dcx][dcy] = B
    else:
        d = d2
        if y1 > y2:
            xtmp = x1; x1 = x2; x2 = xtmp
            ytmp = y1; y1 = y2; y2 = ytmp
        x = x1 - d
        for y in range(int(y1), int(y2)+1):
            x = x + d
            dcy = y
            dcx = int(x)
            if 0 <= dcx < windowSize:
                if 0 <= dcy < windowSize:
                    pixelMapR[dcx][dcy] = R
                    pixelMapG[dcx][dcy] = G
                    pixelMapB[dcx][dcy] = B


def punkt(x, y, R, G, B):  # punkt w 2d
    global pixelMapR, pixelMapG, pixelMapB
    if 0 <= x <= windowSize:
        if 0 <= y <= windowSize:
            pixelMapR[x][y] = R
            pixelMapG[x][y] = G
            pixelMapB[x][y] = B


def persp(p, l, r, b, t, n, f):  # projekcja ortograficzna
    x = p[0]
    y = p[1]
    z = p[2]

    ret = [2 * ((x * n - r * z) / (r * z - l * z)) + 1,
        2 * ((y * n - t * z) / (t * z - b * z)) + 1,
        1 - 2 * ((z - f) / (n - f))]
    return ret


def screen(p, width, height): # przekształcanie na wymiary ekranu
    ret = [(width - 1) * (p[0] + 1) / 2, (height - 1) * (p[1] + 1) / 2]
    return ret


def odcinek3D(p1, p2, R, G, B): # rysowanie odcinka w 3D
    p1o = persp(p1, OP.l, OP.r, OP.b, OP.t, OP.n, OP.f)
    p2o = persp(p2, OP.l, OP.r, OP.b, OP.t, OP.n, OP.f)
    p1s = screen([p1o[0], p1o[1]], windowSize, windowSize)
    p2s = screen([p2o[0], p2o[1]], windowSize, windowSize)
    odcinek(p1s[0], p1s[1], p2s[0], p2s[1], R, G, B)


def punkt3D(p):
    po = persp(p, OP.l, OP.r, OP.b, OP.t, OP.n, OP.f)
    ps = screen(po, windowSize, windowSize)
    punkt(round(ps[0]), round(ps[1]), 1.0, 1.0, 1.0)


def odcinek3D_w(p1, p2):
    cam_view = camera(q)
    p1 = cam_view @ p1.T
    p2 = cam_view @ p2.T
    p1 = p1.T
    p2 = p2.T
    p1o = persp(p1, OP.l, OP.r, OP.b, OP.t, OP.n, OP.f)
    p2o = persp(p2, OP.l, OP.r, OP.b, OP.t, OP.n, OP.f)
    p1s = screen(p1o, windowSize, windowSize)
    p2s = screen(p2o, windowSize, windowSize)

    odcinek(p1s[0], p1s[1], p2s[0], p2s[1], 1.0, 1.0, 1.0)


def trojkat(p1, p2, p3):
    odcinek3D_w(p1, p2)
    odcinek3D_w(p2, p3)
    odcinek3D_w(p3, p1)


def prostokat(plewygorny, pprawydolny):
    p1 = np.array(plewygorny)
    p3 = np.array(pprawydolny)
    p2 = np.array([plewygorny[0], pprawydolny[1], plewygorny[2]])
    p4 = np.array([pprawydolny[0], plewygorny[1], pprawydolny[2]])

    odcinek3D_w(p1, p2)
    odcinek3D_w(p2, p3)
    odcinek3D_w(p3, p4)
    odcinek3D_w(p1, p4)


def prostopadloscian(dlugoscbokuA, dlugoscbokuB, dlugoscbokuC, psrodek):
    p1 = np.array([psrodek[0] - dlugoscbokuA, psrodek[1] - dlugoscbokuB, psrodek[2] - dlugoscbokuC])
    p2 = np.array([psrodek[0] + dlugoscbokuA, psrodek[1] - dlugoscbokuB, psrodek[2] - dlugoscbokuC])
    p3 = np.array([psrodek[0] + dlugoscbokuA, psrodek[1] + dlugoscbokuB, psrodek[2] - dlugoscbokuC])
    p4 = np.array([psrodek[0] - dlugoscbokuA, psrodek[1] + dlugoscbokuB, psrodek[2] - dlugoscbokuC])
    p5 = np.array([psrodek[0] - dlugoscbokuA, psrodek[1] - dlugoscbokuB, psrodek[2] + dlugoscbokuC])
    p6 = np.array([psrodek[0] + dlugoscbokuA, psrodek[1] - dlugoscbokuB, psrodek[2] + dlugoscbokuC])
    p7 = np.array([psrodek[0] + dlugoscbokuA, psrodek[1] + dlugoscbokuB, psrodek[2] + dlugoscbokuC])
    p8 = np.array([psrodek[0] - dlugoscbokuA, psrodek[1] + dlugoscbokuB, psrodek[2] + dlugoscbokuC])

    odcinek3D_w(p1, p2)
    odcinek3D_w(p2, p3)
    odcinek3D_w(p3, p4)
    odcinek3D_w(p1, p4)
    odcinek3D_w(p5, p6)
    odcinek3D_w(p6, p7)
    odcinek3D_w(p7, p8)
    odcinek3D_w(p5, p8)
    odcinek3D_w(p1, p5)
    odcinek3D_w(p2, p6)
    odcinek3D_w(p3, p7)
    odcinek3D_w(p4, p8)


def szes(dlugoscboku, srodek, rotx, roty, rotz, R, G, B):
    pkt = [[srodek[0] - dlugoscboku, srodek[1] - dlugoscboku, srodek[2] - dlugoscboku],
        [srodek[0] + dlugoscboku, srodek[1] - dlugoscboku, srodek[2] - dlugoscboku],
        [srodek[0] + dlugoscboku, srodek[1] + dlugoscboku, srodek[2] - dlugoscboku],
        [srodek[0] - dlugoscboku, srodek[1] + dlugoscboku, srodek[2] - dlugoscboku],
        [srodek[0] - dlugoscboku, srodek[1] - dlugoscboku, srodek[2] + dlugoscboku],
        [srodek[0] + dlugoscboku, srodek[1] - dlugoscboku, srodek[2] + dlugoscboku],
        [srodek[0] + dlugoscboku, srodek[1] + dlugoscboku, srodek[2] + dlugoscboku],
        [srodek[0] - dlugoscboku, srodek[1] + dlugoscboku, srodek[2] + dlugoscboku]]

    for i in range(8):
        npkt = np.matmul(np.array([[1, 0, 0], [0, np.cos(rotx), -np.sin(rotx)],
                                   [0, np.sin(rotx), np.cos(rotx)]]), np.array(pkt[i]).transpose())
        npkt = npkt.tolist()
        pkt[i] = npkt

    for i in range(8):
        npkt = np.matmul(np.array([[np.cos(roty), 0, np.sin(roty)], [0, 1, 0],
                                   [-np.sin(roty), 0, np.cos(roty)]]), np.array(pkt[i]).transpose())
        npkt = npkt.tolist()
        pkt[i] = npkt

    for i in range(8):
        npkt = np.matmul(np.array([[np.cos(rotz), -np.sin(rotz), 0],
                                   [np.sin(rotz), np.cos(rotz), 0], [0, 0, 1]]), np.array(pkt[i]).transpose())
        npkt = npkt.tolist()
        pkt[i] = npkt

    odcinek3D(pkt[0], pkt[1], R, G, B)
    odcinek3D(pkt[1], pkt[2], R, G, B)
    odcinek3D(pkt[2], pkt[3], R, G, B)
    odcinek3D(pkt[3], pkt[0], R, G, B)
    odcinek3D(pkt[4], pkt[5], R, G, B)
    odcinek3D(pkt[5], pkt[6], R, G, B)
    odcinek3D(pkt[6], pkt[7], R, G, B)
    odcinek3D(pkt[7], pkt[4], R, G, B)
    odcinek3D(pkt[0], pkt[4], R, G, B)
    odcinek3D(pkt[1], pkt[5], R, G, B)
    odcinek3D(pkt[2], pkt[6], R, G, B)
    odcinek3D(pkt[3], pkt[7], R, G, B)


def camera(position):
    global front
    front = np.array([np.cos(np.radians(pitch)) * np.sin(np.radians(yaw)), np.sin(np.radians(pitch)),
             np.cos(np.radians(pitch)) * np.cos(np.radians(yaw))])

    direction = position + front
    len_direction = np.sqrt(direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2)
    if len_direction != 1:
        direction = direction / len_direction

    n = np.array([0, 1, 0])
    right = np.cross(n, direction)
    up = np.cross(direction, right)

    view = np.array([[right[0], right[1], right[2]], [up[0], up[1], up[2]], [direction[0], direction[1], direction[2]]])
    view[0] = view[0] - position[0]
    view[1] = view[1] - position[1]
    view[2] = view[2] - position[2]

    return view


def keyboard(k, x, y):
    global key
    key = k.decode("utf-8")


while True:
    global front
    clearMap([0.0, 0.0, 0.0])
    glutKeyboardFunc(keyboard)
    if key == 'w':
        q += camspeed * front
    if key == 's':
        q -= camspeed * front
    if key == 'q':
        yaw -= 1
    if key == 'e':
        yaw += 1
    if key == 'r':
        pitch += 1
    if key == 'f':
        pitch -= 1
    key = 0
    szescian(5, [2, 2, 30], [2, 3, 5], [1, 2, 3], np.radians(10))
    szescian(3, [-10, -10, 30], [2, 3, 5], [1, 2, 3], np.radians(20))
    szescian(2, [-5, 5, 10], [2, 3, 5], [1, 2, 3], np.radians(0))
    szescian(5, [2, 2, 30], [2, 3, 5], [1, 2, 3], np.radians(i))
    i += 1
    paint()
    glutMainLoopEvent()
