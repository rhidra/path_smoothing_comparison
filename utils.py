import os
import math
import numpy as np
from numbers import Number


class Node:
    def __init__(self, pos, parent):
        self.pos = pos
        self.parent = parent
        if parent:
            self.cost = parent.cost + dist(parent, pos)
        else:
            self.cost = 0

    def __repr__(self):
        return self.pos.__repr__()


class Log:
    logging = False

def set_log(logging):
    Log.logging = logging

def log(*args, **kwargs):
    if Log.logging:
        print(*args, **kwargs)


def intersect_line(l1s, l1e, l2s, l2e):
    a1 = (l1e[1] - l1s[1]) / (l1e[0] - l1s[0]) if l1e[0] != l1s[0] else 1e+100
    b1 = l1s[1] - a1 * l1s[0]
    a2 = (l2e[1] - l2s[1]) / (l2e[0] - l2s[0]) if l2e[0] != l2s[0] else 1e+100
    b2 = l2s[1] - a2 * l2s[0]
    if a1 == a2:
        return None
    x = (b2 - b1) / (a1 - a2)
    y = a1 * x + b1
    # Belongs to l1
    dotproduct = (x - l1s[0]) * (l1e[0] - l1s[0]) + (y - l1s[1])*(l1e[1] - l1s[1])
    squaredlengthba = (l1e[0] - l1s[0])**2 + (l1e[1] - l1s[1])**2
    if dotproduct < 0 or dotproduct > squaredlengthba:
        return None
    # Belongs to l2
    dotproduct = (x - l2s[0]) * (l2e[0] - l2s[0]) + (y - l2s[1])*(l2e[1] - l2s[1])
    squaredlengthba = (l2e[0] - l2s[0])**2 + (l2e[1] - l2s[1])**2
    if dotproduct < 0 or dotproduct > squaredlengthba:
        return None
    return (x, y)


def is_in_obstacle(point, obstacles):
    for obstacle in obstacles:
        if point_in_box(point, obstacle):
            return True
    return False


def point_in_box(p, o):
    return o[0] <= p[0] and p[0] <= o[0] + o[2] and o[1] <= p[1] and p[1] <= o[1] + o[3]


def box_in_box(a, b):
  return (abs(a[0] - b[0]) * 2 < (a[2] + b[2])) and (abs(a[1] - b[1]) * 2 < (a[3] + b[3]))

def line_in_obstacles(origin, dest, obstacles):
    for obstacle in obstacles:
        if intersect_line_rect(origin, dest, obstacle):
            return True
    return False


def intersect_line_rect(ls, le, rect):
    points = []
    points.append(intersect_line(ls, le, [rect[0], rect[1]], [rect[0] + rect[2], rect[1]]))
    points.append(intersect_line(ls, le, [rect[0] + rect[2], rect[1]], [rect[0] + rect[2], rect[1] + rect[3]]))
    points.append(intersect_line(ls, le, [rect[0] + rect[2], rect[1] + rect[3]], [rect[0], rect[1] + rect[3]]))
    points.append(intersect_line(ls, le, [rect[0], rect[1] + rect[3]], [rect[0] + rect[2], rect[1]]))
    hit_point = None
    for p in points:
        if p and dist(ls, p) < dist(ls, hit_point):
            hit_point = p
            dist_min = dist(ls, p)
    return hit_point


def dist(p1, p2, sqrt=False):
    if p1 == None or p2 == None:
        return math.inf
    p1 = p1 if isinstance(p1, tuple) or isinstance(p1, list) else p1.pos
    p2 = p2 if isinstance(p2, tuple) or isinstance(p2, list) else p2.pos
    sqr = (p1[0] - p2[0])**2 + (p1[1]-p2[1])**2
    return math.sqrt(sqr) if sqrt else sqr
