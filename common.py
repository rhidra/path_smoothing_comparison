import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import math, pygame
from pygame.locals import *
import numpy as np
from numbers import Number


class Constants:
    EPSILON = 50


class Simulation:
    screen = None

    # Colors
    node = 255, 240, 200
    background = 20, 20, 40
    obstacle = 255, 0, 0
    path = 0, 255, 0
    goal = 0, 255, 0
    start = 0, 0, 255
    point = 128, 0, 128

    def __init__(self, width, height):
        # Pygame init
        pygame.init()
        Simulation.width = width
        Simulation.height = height
        Simulation.screen = pygame.display.set_mode([width, height])
        pygame.display.set_caption('RRT')

    def draw(start=None, goal=None, obstacles=[], nodes=[], path=[], point=None):
        screen = Simulation.screen

        screen.fill(Simulation.background)
        for o in obstacles:
            pygame.draw.rect(screen, Simulation.obstacle, o)
        for node in nodes:
            if node.parent:
                pygame.draw.line(screen, Simulation.node, node.parent.pos, node.pos)
                pygame.draw.circle(screen, Simulation.node, node.pos, 2)
        for node in path:
            if isinstance(node, tuple):
                pygame.draw.circle(screen, Simulation.path, (math.floor(node[0]),math.floor(node[1])), 2)
            elif node.parent:
                pygame.draw.line(screen, Simulation.path, node.parent.pos, node.pos)
                pygame.draw.circle(screen, Simulation.path, node.pos, 3)
        if goal:
            pygame.draw.rect(screen, Simulation.goal, goal)
        if start:
            pygame.draw.rect(screen, Simulation.start, pygame.Rect(start[0]-4, start[1]-4, 8, 8))
        if point:
            pygame.draw.circle(screen, Simulation.point, point, 4)
        pygame.display.update()

    def draw_astar(start, goal, grid, nodes=[], under_sampling=1, point=None):
        screen = Simulation.screen
        screen.fill(Simulation.background)
        for x, line in enumerate(grid):
            for y, node in enumerate(line):
                if node.value == 1:
                    pygame.draw.rect(screen, Simulation.obstacle, pygame.Rect(x*under_sampling,y*under_sampling,under_sampling,under_sampling))
        for node in nodes:
            if node.parent:
                pygame.draw.line(screen, Simulation.node, node.parent.pos, node.pos)
                pygame.draw.circle(screen, Simulation.node, node.pos, 2)
        pygame.draw.rect(screen, Simulation.goal, pygame.Rect(goal.pos[0]-4, goal.pos[1]-4, 8, 8))
        pygame.draw.rect(screen, Simulation.start, pygame.Rect(start.pos[0]-4, start.pos[1]-4, 8, 8))
        if point:
            pygame.draw.circle(screen, Simulation.point, point, 4)
        pygame.display.update()

    def draw_paths(start, goal, obstacles, *paths):
        screen = Simulation.screen
        screen.fill(Simulation.background)
        for o in obstacles:
            pygame.draw.rect(screen, Simulation.obstacle, o)
        for path, color in zip(paths[0::2], paths[1::2]):
            for i in range(1,len(path)):
                try:
                    pygame.draw.line(screen, color, path[i-1], path[i])
                    pygame.draw.circle(screen, color, (math.floor(path[i][0]), math.floor(path[i][1])), 3)
                except:
                    pass
        if goal:
            pygame.draw.rect(screen, Simulation.goal, goal)
        if start:
            pygame.draw.rect(screen, Simulation.start, pygame.Rect(start[0]-4, start[1]-4, 8, 8))
        pygame.display.update()


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
        if obstacle.collidepoint(point[0], point[1]):
            return True
    return False


def line_in_obstacles(origin, dest, obstacles):
    for obstacle in obstacles:
        if intersect_line_rect(origin, dest, obstacle):
            return True
    return False


def intersect_line_rect(ls, le, rect):
    points = []
    points.append(intersect_line(ls, le, rect.topleft, rect.topright))
    points.append(intersect_line(ls, le, rect.topright, rect.bottomright))
    points.append(intersect_line(ls, le, rect.bottomright, rect.bottomleft))
    points.append(intersect_line(ls, le, rect.bottomleft, rect.topleft))
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
