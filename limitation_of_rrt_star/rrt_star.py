##########################
# Autor: Junyeob Baek
# email: wnsdlqjtm@gmail.com
##########################

from anytree import Node, RenderTree
from random import sample
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point


def is_feasible(obstacles, x):
    p = Point(x[0], x[1])

    for obs in obstacles:
        if obs.contains(p):
            return False

    return True


def cal_dist(x1, x2):
    return np.linalg.norm(np.array(x1) - np.array(x2))

def node_cost(tree, x):
    pass

def edge_cost(x1, x2):
    return cal_dist(x1, x2)

def is_goal(goal_pos, new_node):
    dist = cal_dist(goal_pos, new_node.data)
    return True if dist < 0.05 else False

def steer(x_tree, x_rand):
    dist = cal_dist(x_tree, x_rand)
    sigma = zip(
        np.arange(
            x_tree[0],
            x_rand[0],
            stepscale / dist * (x_rand[0] - x_tree[0]),
        ),
        np.arange(
            x_tree[1],
            x_rand[1],
            stepscale / dist * (x_rand[1] - x_tree[1]),
        ),
    )
    return sigma

def render(problem, tree, x_tree, x_rand):
    init_pos, goal_pos, x_lim, y_lim, obstacles = problem

    # plot init and goal
    plt.scatter(init_pos[0], init_pos[1], c="k", label="init")
    plt.scatter(goal_pos[0], goal_pos[1], c="r", label="goal")

    # plot x_tree and x_rand
    plt.scatter(x_tree[0], x_tree[1], c="k", marker="^", label="x_tree")
    plt.scatter(x_rand[0], x_rand[1], c="r", marker="^", label="x_rand")

    # render all tree nodes
    for row in RenderTree(tree):
        pre, fill, node = row
        leaf_x = [n.data[0] for n in node.children]
        leaf_y = [n.data[1] for n in node.children]
        plt.scatter(leaf_x, leaf_y, s=1, c="b")

    for obs in obstacles:
        xs, ys = obs.exterior.xy
        plt.fill(xs, ys, fc="m", label="obstacle")

    # save figure
    plt.title("RRT Star")
    plt.legend()
    plt.xlim(x_lim[0], x_lim[1])
    plt.ylim(y_lim[0], y_lim[1])
    plt.savefig(f"{len(all_nodes)}.png")
    plt.cla()


if __name__ == "__main__":
    # Problem Setup
    all_nodes = []
    num_max_tries = 5
    stepscale = 0.5
    is_achieved = False
    gamma = 1
    eta = 1.0
    d = 2 # dimension of configuration

    # set init and goal
    init_pos = [-4, -1]
    goal_pos = [4, 7]

    # set the boundary of configuration space
    x_lim = [-10, 10]
    y_lim = [-10, 10]

    # obstacles
    obstacles = [
        Polygon([(-1, 4), (-2, 3), (1, 1), (4, 1)]),
    ]

    # set problem
    problem = (init_pos, goal_pos, x_lim, y_lim, obstacles)

    # initialize a tree with an initial configuration
    root = Node("root", data=init_pos)
    all_nodes.append(root)

    cnt = 0
    while not is_achieved:
        # select random configuration from the collision-free configuration space
        x_rand = None
        scnt = 0
        while x_rand is None:
            sampled_x = [
                np.random.uniform(low=x_lim[0], high=x_lim[1]),
                np.random.uniform(low=y_lim[0], high=y_lim[1]),
            ]

            if is_feasible(obstacles, sampled_x):
                x_rand = sampled_x

            if scnt > num_max_tries:
                raise Exception("Exceeded the maximum number of attempts")

        # select a nearest neighbor node from tree
        sample_node = root
        for row in RenderTree(root):
            pre, fill, node = row
            if cal_dist(sample_node.data, x_rand) > cal_dist(node.data, x_rand):
                sample_node = node
        x_nearest = sample_node
        
        # get x_new
        sigma = steer(x_nearest.data, x_rand)
        x_new = x_nearest.data
        for x in sigma:
            if is_feasible(obstacles, x) and cal_dist(x, x_nearest.data) < eta:
                x_new = x
        
        # get set of near nodes from sampled point
        X_near = []; n = len(all_nodes)
        for row in RenderTree(root):
            pre, fill, node = row
            if cal_dist(node.data, x_new) < max(gamma*(np.log(n)/n)**(1/d), eta):
                X_near.append(node)
        
        # choose parent that have minimum cost        
        minCost = 9999
        for x_near in X_near:
            cost = cal_dist(x_near.data, init_pos) + cal_dist(x_near.data, x_new) # node cost + edge cost
            #TODO: need to calculate node_cost and edge_cost
            if cost < minCost:
                minCost = cost
                x_min = x_near

        # feasibility check for the chosen edge
        sigma = steer(x_min.data, x_new)
        if all([is_feasible(obstacles, x_ent) for x_ent in sigma]):
            # add this node
            new_node = Node(f"{len(all_nodes)}", parent=x_min, data=x_new)
            all_nodes.append(new_node)

            # rewiring among X_near
            for x_near in X_near:
                sigma = steer(x_new, x_near.data)
                new_cost = cal_dist(x_new, init_pos) + cal_dist(x_near.data, x_new)
                prev_cost = cal_dist(x_near, init_pos)
                #TODO: need to calculate node_cost and edge_cost
                if new_cost < prev_cost:
                    if all([is_feasible(obstacles, x_ent) for x_ent in sigma]):
                        x_near.parent = None # detach
                        x_near.parent = new_node# attach
                
                

        # plot scene and tree nodes
        render(problem, root, x_min.data, x_rand)





        # if goal configuration is added to the tree, terminate
        for row in RenderTree(root):
            pre, fill, node = row
            if is_goal(goal_pos, node):
                is_achieved = True
                break

# TODO: draw resulting found path
