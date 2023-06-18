#!/usr/bin/env python3

import argparse
import sys
import cairo
import random
import math
import copy
import itertools
import networkx as nx
from scipy.spatial import Delaunay
from shapely import geometry

CMD = None
SEED = None
MAX_DISTANCE = 0.35
MIN_SQUARE_WIDTH = 0.08
MAX_SQUARE_WIDTH = 0.3
MIN_VIEWPOINT_VIEWPOINT_DISTANCE = 0.1
MIN_VIEWPOINT_OBSTACLE_DISTANCE = 0.05
MIN_OBSTACLE_OBSTACLE_DISTANCE = 0.05

MOVE_COST = 1
RECHARGE_COST = 1

class SquareObstacle(object):
    def __init__(self, pos, w):
        self.pos = pos
        self.w = w

        x, y = pos
        self.pts = ((x - w/2., y - w/2.),
                    (x + w/2., y - w/2.),
                    (x + w/2., y + w/2.),
                    (x - w/2., y + w/2.))
        self.shape = geometry.Polygon(self.pts)

    def draw(self, context):
        context.move_to(*self.pts[0])
        context.line_to(*self.pts[1])
        context.line_to(*self.pts[2])
        context.line_to(*self.pts[3])
        context.line_to(*self.pts[0])
        context.stroke()

    def intersects(self, o):
        return self.shape.intersects(o.shape)

    def distance(self, o):
        return self.shape.distance(o.shape)

    def points(self):
        return self.pts


class Line(object):
    def __init__(self, fr, to):
        self.fr = fr
        self.to = to
        self.shape = geometry.LineString([self.fr, self.to])

    def draw(self, context):
        context.move_to(*self.fr)
        context.line_to(*self.to)
        context.stroke()

    def intersects(self, o):
        return self.shape.crosses(o.shape)

    def connections(self):
        return [(self.fr, self.to)]

class ViewPoint(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.shape = geometry.Point(self.x, self.y)

    def points(self):
        return [(self.x, self.y)]

    def distance(self, o):
        return self.shape.distance(o.shape)

    def draw(self, context):
        context.arc(self.x, self.y, 0.01, 0., 2 * math.pi)
        context.stroke()

class Map(object):
    def __init__(self):
        self.viewpoints = []
        self.obstacles = []
        self.edges = []
        self.locations = []
        self.connections = []

    def genRandomObstacles(self, num):
        def random_square():
            w = random.uniform(MIN_SQUARE_WIDTH, MAX_SQUARE_WIDTH)
            x = random.uniform(w/2., 1 - w/2.)
            y = random.uniform(w/2., 1 - w/2.)
            return SquareObstacle((x, y), w)

        def canUseObstacle(o):
            for obs in self.obstacles:
                if o.intersects(obs):
                    return False
                if o.distance(obs) < MIN_OBSTACLE_OBSTACLE_DISTANCE:
                    return False
            return True

        while len(self.obstacles) < num:
            o = random_square()
            if canUseObstacle(o):
                self.obstacles += [o]

    def genRandomViewPoints(self, num):
        def canUseViewPoint(v):
            for w in self.viewpoints:
                if v.distance(w) < MIN_VIEWPOINT_VIEWPOINT_DISTANCE:
                    return False
            for obs in self.obstacles:
                if v.distance(obs) < MIN_VIEWPOINT_OBSTACLE_DISTANCE:
                    return False
            return True

        while len(self.viewpoints) < num:
            x = random.uniform(0.01, 0.99)
            y = random.uniform(0.01, 0.99)
            v = ViewPoint(x, y)
            if canUseViewPoint(v):
                self.viewpoints += [v]

    def triangulate(self):
        def edge(x, y):
            if x < y:
                return (x, y)
            return (y, x)

        def edgeInConflictWithObstacle(s):
            for o in self.obstacles:
                if o.shape.contains(s.shape):
                    return True
            return False

        vpts = []
        #vpts = [(0., 0.), (0., 1.), (1., 0.), (1., 1.)]
        vpts += [(x.x, x.y) for x in self.viewpoints]
        for o in self.obstacles:
            vpts += o.points()
        tri = Delaunay(vpts)
        edges = []
        for s0, s1, s2 in tri.simplices:
            edges += [edge(s0, s1)]
            edges += [edge(s1, s2)]
            edges += [edge(s2, s0)]

        self.locations = vpts
        for e in set(edges):
            f = vpts[e[0]]
            t = vpts[e[1]]
            if geometry.Point(*f).distance(geometry.Point(*t)) > MAX_DISTANCE:
                continue

            line = Line(f, t)
            if edgeInConflictWithObstacle(line):
                continue
            self.edges += [line]
            self.connections += [e]

    def allIsReachable(self):
        queue = [0]
        processed = set([0])
        reached = set()
        while len(queue) > 0:
            p = queue.pop()
            reached.add(p)

            for f, t in self.connections:
                if f == p and t not in processed:
                    queue += [t]
                    processed.add(t)
                elif t == p and f not in processed:
                    queue += [f]
                    processed.add(f)

        return len(reached) == len(self.locations)

    def locationsRadius(self, center, radius):
        g = nx.Graph()
        for e in self.connections:
            g.add_edge(*e)
        distances = dict(nx.all_pairs_shortest_path(g))

        out = []
        for i in range(len(self.locations)):
            if len(distances[center][i]) - 1 <= radius:
                out += [i]
        return sorted(out)

    def locationsByDistance(self, center):
        dist = self.allShortestPaths()
        loc = [(i, len(dist[center][i]) - 1) for i in range(len(self.locations))]
        loc = sorted(loc, key = lambda x: (x[1], x[0]))
        return [x[0] for x in loc]

    def allShortestPaths(self):
        g = nx.Graph()
        for e in self.connections:
            g.add_edge(*e)
        return dict(nx.all_pairs_shortest_path(g))

    def minCover(self, locations, robot_locations = None):
        import cplex
        prob = cplex.Cplex()
        prob.set_log_stream(None)
        #prob.set_error_stream(None)
        prob.set_warning_stream(None)
        prob.set_results_stream(None)
        prob.objective.set_sense(prob.objective.sense.minimize)

        all_locations = [x for x in locations]
        for e in self.connections:
            if e[0] in locations or e[1] in locations:
                all_locations += e
        all_locations = sorted(list(set(all_locations)))

        if robot_locations is not None:
            num_robots = len(robot_locations)
            distances = self.allShortestPaths()
            for i, r in enumerate(robot_locations):
                # Robot variables
                names = ['r{0}_{1}'.format(i, x) for x in all_locations]
                obj = [1. * (len(distances[r][x]) - 1) for x in all_locations]
                prob.variables.add(obj = obj,
                                   types = ['B' for x in all_locations],
                                   names = names)

                # Each robot is headed to at most one location
                x = [names, [1. for n in names]]
                prob.linear_constraints.add(lin_expr = [x], senses = ['L'], rhs = [1.])

            # Location variables
            names = ['x{0}'.format(x) for x in all_locations]
            obj = [0. for x in all_locations]
            prob.variables.add(obj = obj,
                               types = ['B' for x in all_locations],
                               names = names)

            for loc in all_locations:
                # Each location is occupied by at most one robot
                names = ['r{0}_{1}'.format(r, loc) for r in range(num_robots)]
                x = [names, [1. for n in names]]
                prob.linear_constraints.add(lin_expr = [x], senses = ['L'], rhs = [1.])

                # If location is selected, also one of the robots must be
                # selected
                names = ['x{0}'.format(loc)] \
                            + ['r{0}_{1}'.format(r, loc) for r in range(num_robots)]
                x = [names, [-1.] + [1. for n in range(num_robots)]]
                prob.linear_constraints.add(lin_expr = [x], senses = ['G'], rhs = [0.])

        else:
            names = ['x{0}'.format(x) for x in all_locations]
            obj = [1. for x in all_locations]
            prob.variables.add(obj = obj,
                               types = ['B' for x in all_locations],
                               names = names)

#        if num_robots is not None:
#            # Enforce number of robots
#            names = ['x{0}'.format(x) for x in all_locations]
#            x = [names, [1. for n in names]]
#            prob.linear_constraints.add(lin_expr = [x], senses = ['E'],
#                                        rhs = [num_robots])

        for y in locations:
            # Enforce that every location is covered from at least one
            # location
            N = [y]
            for e in self.connections:
                if e[0] == y:
                    N += [e[1]]
                elif e[1] == y:
                    N += [e[0]]
            names = ['x{0}'.format(x) for x in N]
            x = [names, [1. for n in names]]
            prob.linear_constraints.add(lin_expr = [x], senses = ['G'], rhs = [1.])
        prob.solve()


        sol = {
            # List of nodes that cover 'covered-locations'
            'cover' : [],
            # Target location for each robot so that they cover 'covered-locatins'
            'robot-target' : copy.deepcopy(robot_locations),
            # Cost of moving robots from 'robot-locations' to 'robot-targets'
            'move-cost' : -1,
            # List of node that ought to be covered
            'covered-locations' : copy.deepcopy(locations),
            # Initial locations of robots
            'robot-locations' : copy.deepcopy(robot_locations),
        }

        for x in all_locations:
            name = 'x{0}'.format(x)
            xval = prob.solution.get_values([name])
            if xval[0] > 0.5:
                sol['cover'] += [x]
        sol['move-cost'] = int(math.ceil(prob.solution.get_objective_value()))

        if robot_locations is not None:
            for i, rl in enumerate(robot_locations):
                for x in all_locations:
                    name = 'r{0}_{1}'.format(i, x)
                    xval = prob.solution.get_values([name])
                    if xval[0] > 0.5:
                        sol['robot-target'][i] = x
        return sol

    def draw(self, fn, blue = [], red = [], purple = []):
        with cairo.SVGSurface(fn, 600, 600) as surface:
            context = cairo.Context(surface)
            context.scale(600, 600)

#            context.set_source_rgba(0, 0, 0, 0.6)
#            context.set_line_width(0.01)
#            context.move_to(0, 0)
#            context.line_to(1,0)
#            context.line_to(1,1)
#            context.line_to(0,1)
#            context.line_to(0,0)
#            context.stroke()

            context.set_source_rgba(0.7, 0.3, 0.3, 0.6)
            context.set_line_width(0.003)
            for o in self.obstacles:
                o.draw(context)

            context.set_source_rgba(0.3, 0.7, 0.3, 0.6)
            context.set_line_width(0.003)
            for v in self.viewpoints:
                v.draw(context)

            context.set_source_rgba(0.1, 0.1, 0.1, 0.6)
            context.set_line_width(0.001)
            for e in self.edges:
                e.draw(context)

            context.set_source_rgba(0.1, 0.1, 0.7, 0.6)
            context.set_line_width(0.003)
            for i in blue:
                p = self.locations[i]
                context.arc(p[0], p[1], 0.015, 0., 2 * math.pi)
                context.stroke()

            context.set_source_rgba(0.7, 0.1, 0.1, 0.6)
            context.set_line_width(0.003)
            for i in red:
                p = self.locations[i]
                context.arc(p[0], p[1], 0.02, 0., 2 * math.pi)
                context.stroke()

            context.set_source_rgba(0.7, 0.1, 0.7, 0.6)
            context.set_line_width(0.003)
            for i in purple:
                p = self.locations[i]
                context.arc(p[0], p[1], 0.025, 0., 2 * math.pi)
                context.stroke()

class PddlProblem(object):
    def __init__(self, name, num_robots, battery_levels):
        self.name = name
        self.robots = num_robots
        self.battery = battery_levels
        self.locations = 0
        self.connections = []
        self.robot_locations = [-1 for x in range(self.robots)]
        self.robot_charge = [-1 for x in range(self.robots)]
        self.goal_fact = []
        self.guard_config = []

    def setLocations(self, loc):
        self.locations = loc

    def addConnection(self, f, t):
        self.connections += [(f, t)]
        self.locations = max(self.locations, f + 1)
        self.locations = max(self.locations, t + 1)

    def setRobot(self, ridx, loc, charge):
        self.robot_locations[ridx] = loc
        self.robot_charge[ridx] = charge

    def addGoalRobotLocation(self, ridx, loc):
        self.goal_fact += ['(at robot-{0:02d} location-{1:04d})'.format(ridx, loc)]

    def addGuardConfig(self, locs):
        x = [x for x in locs]
        self.guard_config += [x]

    def createObjects(self):
        s = '(:objects\n'
        loc = ['location-{0:04d}'.format(i) for i in range(self.locations)]
        s += '  {0} - location\n'.format(' '.join(loc))
        rob = ['robot-{0:02d}'.format(i) for i in range(self.robots)]
        s += '  {0} - robot\n'.format(' '.join(rob))
        bat = ['battery-{0:04d}'.format(i) for i in range(self.battery + 1)]
        s += '  {0} - battery-level\n'.format(' '.join(bat))
        cfg = ['config-{0:02d}'.format(i) for i in range(len(self.guard_config))]
        if len(cfg) > 0:
            s += '  {0} - config\n'.format(' '.join(cfg))
        s += ')\n'
        return s

    def createInit(self):
        s = '(:init\n'
        s += f'  (= (move-cost) {MOVE_COST})\n'
        s += f'  (= (recharge-cost) {RECHARGE_COST})\n'
        s += '  (= (total-cost) 0)\n'
        for f, t in self.connections:
            s += '  (CONNECTED location-{0:04d} location-{1:04d})\n'.format(f, t)
        s += '\n'
        for i in range(self.battery):
            s += '  (BATTERY-PREDECESSOR battery-{0:04d} battery-{1:04d})\n' \
                    .format(i, i + 1)
        s += '\n'
        for ri in range(self.robots):
            s += '  (at robot-{0:02d} location-{1:04d})\n' \
                    .format(ri, self.robot_locations[ri])
            s += '  (battery robot-{0:02d} battery-{1:04d})\n' \
                    .format(ri, self.robot_charge[ri])
        s += '\n'
        for ci, cfg in enumerate(self.guard_config):
            for loc in cfg:
                s += '  (GUARD-CONFIG config-{0:02d} location-{1:04d})\n' \
                        .format(ci, loc)
            s += '\n'
        s += ')\n'
        return s

    def createGoal(self):
        s = '(:goal\n'
        s += '  (and\n'
        for f in self.goal_fact:
            s += '    {0}\n'.format(f)
        for ci, cfg in enumerate(self.guard_config):
            s += '    (config-fullfilled config-{0:02d})\n'.format(ci)
        s += '  )\n'
        s += ')\n'
        return s

    def create(self):
        assert(len(self.robot_locations) == self.robots)
        assert(-1 not in self.robot_locations)
        assert(-1 not in self.robot_charge)

        s = '(define (problem {0})\n'.format(self.name)
        s += '(:domain recharging-robots)\n'
        s += self.createObjects()
        s += self.createInit()
        s += self.createGoal()
        s += '(:metric minimize (total-cost))\n'
        s += ')\n'
        return s

def createRandomMap(num_obstacles, num_viewpoints):
    m = Map()
    if num_obstacles > 0:
        m.genRandomObstacles(num_obstacles)
    if num_viewpoints > 0:
        m.genRandomViewPoints(num_viewpoints)
    m.triangulate()
    assert(m.allIsReachable())

    return m

def usage():
    print('Usage: {0} SCENARIO [OPTIONS]'.format(sys.argv[0]))
    print('''
SCENARIOS:
  single-source-move-to-locations
  single-source-cover
  covers
''')
    sys.exit(2)

def _common(args):
    global MAX_DISTANCE
    MAX_DISTANCE = args.max_distance
    global MAX_SQUARE_WIDTH
    MAX_SQUARE_WIDTH = args.max_square_width

    global SEED
    SEED = args.random_seed
    random.seed(SEED)
    if SEED is None:
        SEED = random.randint(1, 10000)
        random.seed(SEED)

def planSingleSourceToLocations(m, robot_charge, source, dest):
    dist = m.allShortestPaths()
    length = [len(dist[source][t]) - 1 for t in dest]
    rob_charge = [x for x in robot_charge]
    plan = []
    for r, t in enumerate(dest):
        while rob_charge[r] < length[r]:
            for ro in range(len(dest)):
                if rob_charge[ro] > length[ro]:
                    a = '(recharge robot-{0:02d}'.format(ro)
                    a += ' robot-{0:02d}'.format(r)
                    a += ' location-{0:04d}'.format(source)
                    a += ' battery-{0:04d}'.format(rob_charge[ro])
                    a += ' battery-{0:04d}'.format(rob_charge[ro] - 1)
                    a += ' battery-{0:04d}'.format(rob_charge[r])
                    a += ' battery-{0:04d}'.format(rob_charge[r] + 1)
                    a += ')'
                    plan += [a]
                    rob_charge[ro] -= 1
                    rob_charge[r] += 1

    for r, t in enumerate(dest):
        path = dist[source][t]
        cur_charge = rob_charge[r]
        for i in range(len(path) - 1):
            a = '(move robot-{0:02d}'.format(r)
            a += ' location-{0:04d}'.format(path[i])
            a += ' location-{0:04d}'.format(path[i + 1])
            a += ' battery-{0:04d}'.format(cur_charge)
            a += ' battery-{0:04d}'.format(cur_charge - 1)
            a += ')'
            plan += [a]
            cur_charge -= 1
    return plan

def singleSourceMoveToLocations():
    desc = '''
Generate a problem for re-charging guard robots:
1. All robots are initially located at the same location
2. The goal is to move each robot to a preselected location
3. There is enough battery among all robots for them to move, but they may
   need to re-distribute the battery charge so that each robot can actually
   move to its destination.

4. If --move-from-source is used, TODO
'''
    parser = argparse.ArgumentParser(
                formatter_class = argparse.RawDescriptionHelpFormatter,
                prog = '{0} single-source-move-to-locations'.format(sys.argv[0]),
                description = desc)
    parser.add_argument('--max-distance', type = float, default = 0.35,
                        help = 'maximum distance between points in a map')
    parser.add_argument('--max-square-width', type = float, default = 0.3,
                        help = 'maximum edge width of an obstacle square')
    parser.add_argument('--random-seed', type = int, help = 'Random seed')
    parser.add_argument('--move-from-source', action='store_true',
                        help = 'See description above')
    parser.add_argument('num_robots', type = int, help = 'number of robots')
    parser.add_argument('num_obstacles', type = int,
                        help = 'number of random obstacles')
    parser.add_argument('num_viewpoints', type = int,
                        help = 'number of random additional viewpoints')
    parser.add_argument('charge_multiplier', type = float,
                        help = 'The minimum necessary amount of charge is'
                               ' multiplied by this number.')
    parser.add_argument('output_file', type = str,
                        help = 'Path to output file.')
    parser.add_argument('plan_file', type = str,
                        help = 'Path to the output plan file.')

    args = parser.parse_args()
    _common(args)

    m = createRandomMap(args.num_obstacles, args.num_viewpoints)

    source = random.choice(list(range(len(m.locations))))
    targets = []
    while len(targets) != args.num_robots:
        t = random.choice(list(range(len(m.locations))))
        if t == source or t in targets:
            continue
        targets += [t]

    dist = m.allShortestPaths()
    min_distance = 0
    for t in targets:
        min_distance += len(dist[source][t]) - 1

    charge_amount = int(math.ceil(min_distance * args.charge_multiplier))
    charge = []
    remain_charge = charge_amount
    for t in targets:
        c = random.choice(list(range(remain_charge + 1)))
        charge += [c]
        remain_charge -= c
    charge[-1] += charge_amount - sum(charge)
    assert(sum(charge) == charge_amount)

    sources = [source for i in range(args.num_robots)]
    if args.move_from_source:
        for ri in range(args.num_robots):
            # Select location that does not allowing solving it without
            # re-charging if the robot needed to be re-charged before
            t = targets[ri]
            dt = len(dist[source][t]) - 1
            loc = [i for i in range(len(m.locations)) \
                    if len(dist[i][t]) - 1 >= len(dist[i][source]) - 1 + dt]
            s = random.choice(loc)
            print('Moving {0}: {1} -> {2} | choises: {3}'.format(ri, source, s, loc))
            add_charge = len(dist[s][source]) - 1
            sources[ri] = s
            charge[ri] += add_charge
            charge_amount += add_charge

    m.draw('p.svg', blue = targets, red = sources)

    problem_name = 'recharge-single-source-move-to-locations-{0}' \
                        .format(random.randint(0,10000))
    pddl = PddlProblem(problem_name, len(targets), charge_amount)
    for e in m.connections:
        pddl.addConnection(*e)
    for i in range(len(targets)):
        pddl.setRobot(i, sources[i], charge[i])
    for i, t in enumerate(targets):
        pddl.addGoalRobotLocation(i, t)

    with open(args.output_file, 'w') as fout:
        print(';; Genearated with: {0}'.format(CMD), file = fout)
        print(';; Random seed: {0}'.format(SEED), file = fout)
        print(pddl.create(), file = fout)

    with open(args.plan_file, 'w') as fout:
        plan = []
        for ri in range(args.num_robots):
            cur_charge = charge[ri]
            path = dist[sources[ri]][source]
            for i in range(len(path) - 1):
                a = '(move robot-{0:02d}'.format(ri)
                a += ' location-{0:04d}'.format(path[i])
                a += ' location-{0:04d}'.format(path[i + 1])
                a += ' battery-{0:04d}'.format(cur_charge)
                a += ' battery-{0:04d}'.format(cur_charge - 1)
                a += ')'
                plan += [a]
                cur_charge -= 1
            charge[ri] = cur_charge

        plan += planSingleSourceToLocations(m, charge, source, targets)
        print(';; Cost: {0}'.format(len(plan)), file = fout)
        print(';; Plan:', file = fout)
        for p in plan:
            print(p, file = fout)


def singleSourceCover():
    desc = '''
Generate a problem for re-charging guard robots:
1. All robots are initially located at the same location
2. The goal is to cover certain area
3. There is enough battery among all robots for them to move, but they may
   need to re-distribute the battery charge so that each robot can actually
   move to its destination.
'''
    parser = argparse.ArgumentParser(
                formatter_class = argparse.RawDescriptionHelpFormatter,
                prog = '{0} single-source-cover'.format(sys.argv[0]),
                description = desc)
    parser.add_argument('--max-distance', type = float, default = 0.35,
                        help = 'maximum distance between points in a map')
    parser.add_argument('--max-square-width', type = float, default = 0.3,
                        help = 'maximum edge width of an obstacle square')
    parser.add_argument('--random-seed', type = int, help = 'Random seed')
    parser.add_argument('--move-from-source', action='store_true',
                        help = 'See description above')
    parser.add_argument('num_robots', type = int, help = 'number of robots')
    parser.add_argument('min_cover', type = int,
                        help = 'minimum number of robots required to cover the area')
    parser.add_argument('num_obstacles', type = int,
                        help = 'number of random obstacles')
    parser.add_argument('num_viewpoints', type = int,
                        help = 'number of random additional viewpoints')
    parser.add_argument('charge_multiplier', type = float,
                        help = 'The minimum necessary amount of charge is'
                               ' multiplied by this number.')
    parser.add_argument('output_file', type = str,
                        help = 'Path to output file.')

    args = parser.parse_args()
    _common(args)
    assert(args.num_robots >= args.min_cover)
    assert(args.min_cover > 0)

    m = createRandomMap(args.num_obstacles, args.num_viewpoints)

    # Randomly select center of the guarded area
    dest_center = random.choice(list(range(len(m.locations))))

    # Select the locations closest to dest_center so that they can be
    # covered by args.min_cover robots
    target_min_cover = args.min_cover
    loc = m.locationsByDistance(dest_center)
    loc_to = len(loc)
    sol = m.minCover(loc[:loc_to])
    while len(sol['cover']) != target_min_cover:
        if len(sol['cover']) > target_min_cover:
            loc_to -= 1
        sol = m.minCover(loc[:loc_to])
    dest = loc[:loc_to]

    # Randomly select starting location for all robots outside the guarded
    # area
    source = set(list(range(len(m.locations)))) - set(dest)
    source = random.choice(sorted(list(source)))

    # Get optimal solution considering only moving and extract the minimum
    # required charge (and target locations for each robot to create a plan)
    sol = m.minCover(dest, robot_locations = [source for x in range(target_min_cover)])
    print('Solution:', sol)
    robot_targets = sol['robot-target']
    for i in range(len(robot_targets), args.num_robots):
        robot_targets += [source]
    min_charge = sol['move-cost']

    # Distribute charge among robots
    charge_amount = int(math.ceil(min_charge * args.charge_multiplier))
    charge = []
    remain_charge = charge_amount
    for t in range(args.num_robots):
        c = random.choice(list(range(remain_charge + 1)))
        charge += [c]
        remain_charge -= c
    charge[-1] += charge_amount - sum(charge)
    assert(sum(charge) == charge_amount)
    print(charge)

    # TODO: This is nonsense, remove it
    sources = [source for i in range(args.num_robots)]
    if args.move_from_source:
        def distFromDest(i):
            return min([len(dist[i][j]) - 1 for j in dest])

        dist = m.allShortestPaths()
        dt = distFromDest(source)
        loc = [i for i in range(len(m.locations)) \
                if distFromDest(i) >= len(dist[i][source]) - 1 + dt]
        for ri in range(args.num_robots):
            s = random.choice(loc)
            print('Moving {0}: {1} -> {2} | choises: {3}'.format(ri, source, s, loc))
            add_charge = len(dist[s][source]) - 1
            sources[ri] = s
            charge[ri] += add_charge
            charge_amount += add_charge

    #m.draw('p.svg', blue = dest, red = [dest_center,source], purple = sources)

    # Create PDDL problem
    problem_name = 'recharge-single-source-cover-{0}' \
                        .format(random.randint(0,10000))
    pddl = PddlProblem(problem_name, args.num_robots, charge_amount)
    for e in m.connections:
        pddl.addConnection(*e)
    for i in range(args.num_robots):
        pddl.setRobot(i, sources[i], charge[i])
    pddl.addGuardConfig(dest)

    with open(args.output_file, 'w') as fout:
        print(';; Genearated with: {0}'.format(CMD), file = fout)
        print(';; Random seed: {0}'.format(SEED), file = fout)
        print(pddl.create(), file = fout)

        dist = m.allShortestPaths()
        plan = []
        for ri in range(args.num_robots):
            cur_charge = charge[ri]
            path = dist[sources[ri]][source]
            for i in range(len(path) - 1):
                a = '(move robot-{0:02d}'.format(ri)
                a += ' location-{0:04d}'.format(path[i])
                a += ' location-{0:04d}'.format(path[i + 1])
                a += ' battery-{0:04d}'.format(cur_charge)
                a += ' battery-{0:04d}'.format(cur_charge - 1)
                a += ')'
                plan += [a]
                cur_charge -= 1
            charge[ri] = cur_charge

        plan += planSingleSourceToLocations(m, charge, source, robot_targets)
        cost = len(plan)
        for ri in range(args.min_cover):
            plan += ['(stop-and-guard robot-{0:02d} location-{1:04d})' \
                        .format(ri, robot_targets[ri])]
        plan += ['(verify-guard-config config-00)']

        print('', file = fout)
        print(';; Cost: {0}'.format(cost), file = fout)
        print(';; Plan:', file = fout)
        for p in plan:
            print(';;', p, file = fout)


def genArea(m, min_cover, center_locations):
    # Randomly select center of the guarded area
    dest_center = random.choice(center_locations)

    # Select the locations closest to dest_center so that they can be
    # covered by args.min_cover robots
    loc = m.locationsByDistance(dest_center)
    loc_to = len(loc)
    sol = m.minCover(loc[:loc_to])
    while len(sol['cover']) != min_cover:
        if len(sol['cover']) > min_cover:
            loc_to -= 1
        sol = m.minCover(loc[:loc_to])
    dest = loc[:loc_to]
    return sorted(dest)

def genAreas(m, num_areas, min_cover):
    for _ in range(100):
        center_locations = set([i for i in range(len(m.locations))])
        areas = []
        for i in range(num_areas):
            if len(center_locations) == 0:
                break
            area = genArea(m, min_cover, sorted(list(center_locations)))
            center_locations -= set(area)
            areas += [area]
        if len(areas) == num_areas:
            return areas
    print('Error: Could not generate enough areas')
    sys.exit(-1)

def closestByDistance(m, areas, start):
    closest = -1
    target = None
    d = 10000000000
    for i, area in enumerate(areas):
        sol = m.minCover(area, robot_locations = start)
        print('closestByDistance solution:', sol)
        if sol['move-cost'] < d:
            closest = i
            d = sol['move-cost']
            target = sol['robot-target']
    return closest, target

def sortAreasByDistance(m, areas, start):
    out = []
    target = []
    num_areas = len(areas)
    while len(out) != num_areas:
        closest, start = closestByDistance(m, areas, start)
        out += [areas[closest]]
        target += [start]
        del areas[closest]
    return out, target


class Prob(object):
    def __init__(self, m = None, init = None):
        self.cost = 1000000
        self.m = m

        if init is not None:
            self.cost = 0
            self.areas = []
            self.num_robots = len(init)
            self.cur_state = copy.deepcopy(init)
            self.states = [self.cur_state]

            self.min_charge = [0 for _ in init]

    def randevous(self):
        paths = self.m.allShortestPaths()
        best = None
        best_dist = 10000000
        for lid in range(len(self.m.locations)):
            dist = 0
            for l in self.states[0]:
                dist += len(paths[lid][l]) - 1
            if dist < best_dist:
                best_dist = dist
                best = lid

        for i, l in enumerate(self.states[0]):
            self.min_charge[i] = len(paths[l][best]) - 1

        randevous_state = [best for _ in self.states[0]]
        self.cur_state = randevous_state
        self.states += [self.cur_state]
        self.cost += best_dist
        print('Randevous at', best, 'cost:', best_dist)

    def reachArea(self, area):
        sol = self.m.minCover(area, robot_locations = self.cur_state)
        self.cost += sol['move-cost']
        self.cur_state = copy.deepcopy(sol['robot-target'])
        self.states += [self.cur_state]
        self.areas += [area]

    def computeRequiredCharge(self):
        paths = self.m.allShortestPaths()
        self.req_charge = [0 for _ in self.states[0]]
        cur = self.states[0]
        for s in self.states[1:]:
            for robot in range(len(s)):
                self.req_charge[robot] += len(paths[cur[robot]][s[robot]]) - 1
            cur = s
        assert(sum(self.req_charge) == self.cost)

    def distributeInitCharge(self, mult):
        while True:
            charge = int(sum(self.req_charge) * mult)
            self.init_charge = copy.deepcopy(self.min_charge)
            remain = charge - sum(self.init_charge)
            while remain > 0:
                robot = random.randint(0, len(self.init_charge) - 1)
                c = random.randint(0, remain)
                self.init_charge[robot] += c
                remain -= c
            if any([self.init_charge[i] < self.req_charge[i] \
                        for i in range(len(self.init_charge))]):
                break

    def toPddl(self, name):
        p = PddlProblem(name, len(self.states[0]), sum(self.init_charge))
        for e in self.m.connections:
            p.addConnection(*e)
        for i in range(len(self.states[0])):
            p.setRobot(i, self.states[0][i], self.init_charge[i])

        for area in self.areas:
            p.addGuardConfig(area)

        return p.create()

    def toPlan(self):
        num_robots = len(self.states[0])
        paths = self.m.allShortestPaths()
        cur_loc = copy.deepcopy(self.states[0])
        cur_charge = copy.deepcopy(self.init_charge)
        req_charge = copy.deepcopy(self.req_charge)

        plan_cost = 0
        plan = []
        # first move to the randevous point
        assert(len(set(self.states[1])) == 1)
        for robot in range(num_robots):
            path = paths[self.states[0][robot]][self.states[1][robot]]
            for step in range(len(path) - 1):
                a = '(move robot-{0:02d} location-{1:04d} location-{2:04d}'
                a += ' battery-{3:04d} battery-{4:04d})'
                a = a.format(robot, path[step], path[step + 1],
                             cur_charge[robot], cur_charge[robot] - 1)
                plan_cost += MOVE_COST
                plan += [a]
                cur_charge[robot] -= 1
                req_charge[robot] -= 1
        cur_loc = copy.deepcopy(self.states[1])

        # Now recharge robots to .req_charge
        for robot in range(num_robots):
            while cur_charge[robot] < req_charge[robot]:
                # find other robot with enough charge
                fr = [x for x in range(num_robots) \
                        if x != robot and cur_charge[x] > req_charge[x]]
                fr = fr[0]
                a = '(recharge robot-{0:02d} robot-{1:02d} location-{2:04d}'
                a += ' battery-{3:04d} battery-{4:04d}'
                a += ' battery-{5:04d} battery-{6:04d})'
                assert(cur_loc[robot] == cur_loc[fr])
                a = a.format(fr, robot, cur_loc[robot],
                             cur_charge[fr], cur_charge[fr] - 1,
                             cur_charge[robot], cur_charge[robot] + 1)
                plan_cost += RECHARGE_COST
                plan += [a]
                cur_charge[fr] -= 1
                cur_charge[robot] += 1

        # Move robots to their target locations and guard
        config_id = 0
        for dst in range(2, len(self.states)):
            for robot in range(num_robots):
                path = paths[cur_loc[robot]][self.states[dst][robot]]
                for step in range(len(path) - 1):
                    a = '(move robot-{0:02d} location-{1:04d} location-{2:04d}'
                    a += ' battery-{3:04d} battery-{4:04d})'
                    a = a.format(robot, path[step], path[step + 1],
                                 cur_charge[robot], cur_charge[robot] - 1)
                    plan_cost += MOVE_COST
                    plan += [a]
                    cur_charge[robot] -= 1
                a = '(stop-and-guard robot-{0:02d} location-{1:04d})'
                a = a.format(robot, path[-1])
                plan += [a]
            cur_loc = copy.deepcopy(self.states[dst])
            a = '(verify-guard-config config-{0:02d})'.format(config_id)
            plan += [a]
            config_id += 1

        #for p in plan:
        #    print(p)
        s = f';; Cost: {plan_cost}\n'
        s += f';; Length: {len(plan)}\n'
        s += '\n'.join(plan)
        return s

def covers():
    desc = '''
Generate a problem for re-charging guard robots:

TODO
'''
    parser = argparse.ArgumentParser(
                formatter_class = argparse.RawDescriptionHelpFormatter,
                prog = '{0} covers'.format(sys.argv[0]),
                description = desc)
    parser.add_argument('--max-distance', type = float, default = 0.35,
                        help = 'maximum distance between points in a map')
    parser.add_argument('--max-square-width', type = float, default = 0.3,
                        help = 'maximum edge width of an obstacle square')
    parser.add_argument('--random-seed', type = int, help = 'Random seed')
    parser.add_argument('num_robots', type = int, help = 'number of robots')
    parser.add_argument('min_cover', type = int,
                        help = 'minimum number of robots required to cover each area')
    parser.add_argument('num_areas', type = int,
                        help = 'number of areas that should be covered')
    parser.add_argument('num_obstacles', type = int,
                        help = 'number of random obstacles')
    parser.add_argument('num_viewpoints', type = int,
                        help = 'number of random additional viewpoints')
    parser.add_argument('charge_multiplier', type = float,
                        help = 'The minimum necessary amount of charge is'
                               ' multiplied by this number.')
    parser.add_argument('output_file', type = str,
                        help = 'Path to output pddl file.')
    parser.add_argument('output_plan_file', type = str,
                        help = 'Path to output plan file.')

    args = parser.parse_args()
    _common(args)

    m = createRandomMap(args.num_obstacles, args.num_viewpoints)

    # Generate enough areas
    areas = genAreas(m, args.num_areas, args.min_cover)
    print('areas:', areas)

    # Generate start locations outside the areas for robots
    start_locations = set(range(len(m.locations))) - set(sum(areas, start = []))
    start_locations = sorted(list(start_locations))
    random.shuffle(start_locations)
    start = start_locations[:args.num_robots]
    if len(start) != args.num_robots:
        print('Error: Not enough locations outside the target areas')
        sys.exit(-1)
    #m.draw('map.svg', blue = areas[0], red = areas[1], purple = start)
    print('start:', start)

    # Go over all permutations of areas and select the solution with the
    # minimal cost
    best_p = Prob()
    for ordered_areas in itertools.permutations(areas):
        p = Prob(m, start)
        p.randevous()
        for a in ordered_areas:
            p.reachArea(a)

        if p.cost < best_p.cost:
            best_p = p
    best_p.computeRequiredCharge()
    best_p.distributeInitCharge(args.charge_multiplier)
    print('Cost:', best_p.cost)
    print('States:', best_p.states)
    print('Areas:', best_p.areas)
    print('Minumum initial charge:', best_p.min_charge)
    print('Required charge:', best_p.req_charge)
    print('Init charge:', best_p.init_charge)

    with open(args.output_file, 'w') as fout:
        global CMD
        global SEED
        print(f';; Genearated with: {CMD}', file = fout)
        print(f';; Random seed: {SEED}', file = fout)
        rnd = random.randint(0,10000)
        name = f'recharging-robots-cover-robots{args.num_robots}-areas{args.num_areas}-{SEED}-{rnd}'
        pddl = best_p.toPddl(name)
        print(pddl, file = fout)

    with open(args.output_plan_file, 'w') as fout:
        fout.write(best_p.toPlan())

    return 0

if __name__ == '__main__':
    if len(sys.argv) < 2:
        usage()

    CMD = ' '.join(sys.argv)

    scenario = sys.argv[1]
    del sys.argv[1]
    if scenario == 'single-source-move-to-locations':
        sys.exit(singleSourceMoveToLocations())

    elif scenario == 'single-source-cover':
        sys.exit(singleSourceCover())

    elif scenario == 'covers':
        sys.exit(covers())

    else:
        print('Error: Unkown scenario {0}'.format(scenario))
        usage()
