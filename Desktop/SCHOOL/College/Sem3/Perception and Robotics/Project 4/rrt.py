import sys
import time
import math
from map import Map
from gui import *
from utils import *
from robot_sim import *

MAX_NODES = 20000

def RRT(map):
    """ Builds RRT given a map
    """
    
    map.add_node(map.get_start())
    map_width, map_height = map.get_size()
    while (map.get_num_nodes() < MAX_NODES):
        
        random_node = map.node_generator()
        nearest_node = None
        min_distance = float('inf')
        for node in map.get_nodes():  
            distance = get_dist(node, random_node)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        
        max_step = 30 
        direction_vector = (random_node.x - nearest_node.x, random_node.y - nearest_node.y)
        distance = get_dist(nearest_node, random_node)
        if distance > max_step:
            if distance == 0:
                scaled_vector = (0, 0)
            else:
                scaled_vector = (direction_vector[0] * (max_step / distance), direction_vector[1] * (max_step / distance))
            new_node = Node((nearest_node.x + scaled_vector[0], nearest_node.y + scaled_vector[1]))
        else:
            new_node = random_node
        map.add_path(nearest_node, new_node)
        time.sleep(0.01)
        if map.is_solved():
            break

    path = map.get_path()
    smoothed_path = map.get_smooth_path()

    if map.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", map.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")


def robot_planning_with_exploration(robot, map):
    return


class RobotThread(threading.Thread):
    """Thread to run vector code separate from main thread
    """

    def __init__(self, robot, map ):
        threading.Thread.__init__(self, daemon=True)
        self.robot = robot
        self.map = map

    def run(self):
        robot_planning_with_exploration(self.robot, self.map)
        time.sleep(5)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self, map):
        threading.Thread.__init__(self, daemon=True)
        self.map = map

    def run(self):
        RRT(self.map)
        time.sleep(5)
        self.map.reset_paths()
        stopevent.set()


if __name__ == '__main__':
    global stopevent
    stopevent = threading.Event()
    exploration = False
    for i in range(0,len(sys.argv)): 
        #reads input whether we are running the exploration version or not
        if (sys.argv[i] == "-explore"):
            exploration = True

    map = Map("maps/map2.json", exploration)
    
    if exploration:
        r = DDRobot(map.get_start().x, map.get_start().y, map)
        robot_thread = RobotThread(robot=r, map=map)
        visualizer = Visualizer(map, r, stopevent, exploration)
        robot_thread.start()
        visualizer.start()
    else:
        rrt_thread = RRTThread(map=map)
        visualizer = Visualizer(map, None, stopevent, exploration)
        rrt_thread.start()
        visualizer.start()
