##########################
# Autor: Junyeob Baek
# email: wnsdlqjtm@gmail.com
##########################

from anytree import Node, RenderTree
from random import sample
import numpy as np

def is_collide(x): #TODO: implement collision function
    return False

def is_goal(x): #TODO: implement solution checking funciton
    return True

def render(tree): #TODO: implement render tree in plot
    pass

if __name__=="__main__":

    # Problem Setup
    all_nodes = []
    num_max_tries = 5
    stepscale = 0.1
    is_achieved = False

    # set init and goal
    init_pos = [0, 0]
    goal_pos = [4, 6]

    # set the boundary of configuration space
    x_lim = [-10, 10]
    y_lim = [-10, 10]
    
    # obstacles
    obstacles = [[[3,4],
                 [1,2],
                 [4,4]]]

    # initialize a tree with an initial configuration
    root = Node("root", data=init_pos)
    all_nodes.append(root)

    while not is_achieved:
        # select a random node from tree
        x_tree = sample(all_nodes, 1)[0]

        # select random configuration from the collision-free configuration space
        x_rand = None; scnt=0
        while x_rand is None:
            sampled_x = [np.random.uniform(low=x_lim[0], high=x_lim[1]), 
                    np.random.uniform(low=y_lim[0], high=y_lim[1])]

            if not is_collide(sampled_x):
                x_rand = sampled_x
            if scnt > num_max_tries:
                raise Exception("Exceeded the maximum number of attempts")

        # extend from x_tree to x_rand
        extended_nodes = zip(np.arange(x_tree.data[0], x_rand[0], max(stepscale*abs(x_tree.data[0]-x_rand[0]),0.05)),
            np.arange(x_tree.data[1], x_rand[1], max(stepscale*abs(x_tree.data[1]-x_rand[1]), 0.05)))
        x_before = None
        
        # add the new nodes to the tree
        for x_ext in extended_nodes:
            if not is_collide(x_ext):
                new_node = Node(f"{len(all_nodes)}", parent=x_tree if x_before!=None else x_before, data=x_ext)
                all_nodes.append(new_node)
                x_before = new_node

        
        # if goal configuration is added to the tree, terminate
        for node in all_nodes:
            if is_goal(node):
                is_achieved = True
                break
    

    for row in RenderTree(root):
        pre, fill, node = row
        print(f"{pre}{node.name}, data: {node.data}")
    
    