"""Joint-space RRT-Connect with checked edges and a direct-path fast path.

Collision checks are discrete (configurable maximum joint step), not continuous
collision detection. No dynamics, torque, cable, or timing guarantee is implied.
"""
import numpy as np


def interpolate(a, b, resolution):
    n = max(1, int(np.ceil(np.max(np.abs(b-a))/resolution)))
    return np.linspace(a, b, n+1)


def edge_free(world, a, b):
    step = world.cfg["motion"]["edge_resolution_rad"]
    return all(world.collision(q) is None for q in interpolate(a,b,step))


def path_free(world, path):
    return bool(len(path)) and all(edge_free(world, np.array(a), np.array(b))
                                  for a,b in zip(path, path[1:])) and world.collision(np.array(path[-1])) is None


def plan(world, start, goal, rng):
    start, goal = np.asarray(start), np.asarray(goal)
    if world.collision(start) or world.collision(goal):
        return None
    if edge_free(world,start,goal):
        return [start.tolist(),goal.tolist()]
    cfg = world.cfg["motion"]
    margin = world.cfg["robot"]["joint_limit_margin_rad"]
    lo, hi = world.kin.lower+margin, world.kin.upper-margin
    trees = [([start.copy()],[-1]), ([goal.copy()],[-1])]

    def extend(tree, target):
        nodes, parents = tree
        index = int(np.argmin([np.linalg.norm(v-target) for v in nodes]))
        delta = target-nodes[index]
        length = np.linalg.norm(delta)
        if length < 1e-10:
            return index, True
        q = nodes[index] + delta*min(1.,cfg["rrt_step_rad"]/length)
        if not edge_free(world,nodes[index],q):
            return None, False
        nodes.append(q)
        parents.append(index)
        return len(nodes)-1, length <= cfg["rrt_step_rad"]

    def trace(tree, i):
        seq = []
        while i != -1:
            seq.append(tree[0][i])
            i = tree[1][i]
        return seq[::-1]

    for k in range(cfg["rrt_iterations"]):
        side = k % 2
        ta, tb = trees[side], trees[1-side]
        sample = tb[0][0] if rng.random() < .15 else rng.uniform(lo,hi)
        ia, _ = extend(ta,sample)
        if ia is None:
            continue
        for _ in range(100):
            ib, reached = extend(tb,ta[0][ia])
            if ib is None:
                break
            if reached:
                a,b = trace(ta,ia), trace(tb,ib)
                path = a+b[-2::-1] if side == 0 else b+a[-2::-1]
                # Shortcuts are only kept after checking their complete edge.
                for _ in range(20):
                    if len(path) <= 2:
                        break
                    i,j = sorted(rng.choice(len(path),2,replace=False))
                    if j > i+1 and edge_free(world,path[i],path[j]):
                        path = path[:i+1]+path[j:]
                return [q.tolist() for q in path] if path_free(world,path) else None
    return None
