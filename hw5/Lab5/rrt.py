import cozmo
import math
import sys
import time
import random
import numpy as np

from cmap import *
from gui import *
from utils import *

MAX_NODES = 20000

def step_from_to(node0, node1, limit=75):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object

    if get_dist(node0, node1) > limit:
        ang_rad = np.arctan2(node1.y - node0.y, node1.x - node0.x)
        node1 = Node((node0.x + limit * math.cos(ang_rad),
            node0.y + limit * math.sin(ang_rad)))
    return node1
    ############################################################################

def node_generator(cmap):
    rand_node = None
    goalNodeRand = random.randint(1, 100)
    if goalNodeRand >= 96:
        rand_node = Node(cmap.get_goals()[random.randint(0, len(cmap.get_goals()) - 1)])
    else:
        rand_node = Node(tuple(random_coordinates(cmap)))
        while cmap.is_inside_obstacles(rand_node) and not cmap.is_inbound(rand_node):
            rand_node = Node(tuple(random_coordinates(cmap)))

    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object

    pass
    ############################################################################
    return rand_node

def random_coordinates(cmap):
    width_mm = cmap.get_size()[0]
    height_mm = cmap.get_size()[1]
    ## Added 100 mm to each size because I am assuming
    ## Incozmito has a 100mm radius max
    width_coord = random.random() * width_mm
    height_coord = random.random() * height_mm
    return width_coord, height_coord


def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        rand_node = cmap.get_random_valid_node()
        all_nodes = cmap.get_nodes()
        nearest_node = all_nodes[0]
        for node in all_nodes:
            if get_dist(rand_node, node) < get_dist(rand_node, nearest_node):
                nearest_node = node
        rand_node = step_from_to(nearest_node, rand_node)

        ########################################################################
        time.sleep(0.01)
        if not cmap.is_collision_with_obstacles((rand_node, nearest_node)):
            cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    path = cmap.get_path()
    smoothed_path = cmap.get_smooth_path()

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", cmap.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")

def diff_heading_deg(heading1, heading2):
    dh = heading1 - heading2
    while dh > 180:
        dh -= 360
    while dh <= -180:
        dh += 360
    return dh

async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    # global cmap, stopevent, marker_dictionary, current_pose
    # cmap.set_start(Node((6*25.4, 10*25.4)))
    # markers = dict()
    # update, center = await detect_cube_and_update_cmap(robot, markers, cmap.get_start())
    # #######################################################################
    # # TODO: please enter your code below.
    # # Description of function provided in instructions
    # current_pose = cmap.get_start()
    # marker_dictionary = {}

    # # All needed states are here:

    # # Seeking Goal
    # swag_number = 0
    # RRT(cmap, current_pose)
    global cmap, stopevent

    markers = dict()
    count = 0
    currentAngle = 0
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    update, center = await detect_cube_and_update_cmap(robot, markers, cmap.get_start())
    if center is None:
        cmap.add_goal(Node((325,225)))
        RRT(cmap, cmap.get_start())
        cmap.set_start(Node((325,225)))
        path = cmap.get_smooth_path()
        while count != (len(path) - 1):
            currNode = path[count]
            nextNode = path[count + 1]
            diff_x = nextNode.x - currNode.x
            diff_y = nextNode.y - currNode.y

            diff_h = math.degrees(math.atan2(diff_y, diff_x))
            final_distance = math.sqrt((diff_y ** 2) + (diff_x ** 2))
            turn_angle = diff_heading_deg(diff_h, currentAngle)

            await robot.turn_in_place(cozmo.util.degrees(turn_angle)).wait_for_completed()
            await robot.drive_straight(cozmo.util.distance_mm(final_distance), cozmo.util.speed_mmps(100)).wait_for_completed()

            currentAngle = turn_angle
            count = count + 1
            update, center = await detect_cube_and_update_cmap(robot, markers, cmap.get_start())
        while center is None:
            await robot.turn_in_place(cozmo.util.degrees(15)).wait_for_completed()
            update, center = await detect_cube_and_update_cmap(robot, markers, cmap.get_start())
    cmap.reset()
    RRT(cmap, cmap.get_start())
    path = cmap.get_smooth_path()
    swag_number = 0
    count = 0
    currentAngle = 0
    while count != (len(path) - 1):
        currNode = path[count]
        nextNode = path[count + 1]
        diff_x = nextNode.x - currNode.x
        diff_y = nextNode.y - currNode.y

        diff_h = math.degrees(math.atan2(diff_y, diff_x))
        final_distance = math.sqrt((diff_y ** 2) + (diff_x ** 2))
        turn_angle = diff_heading_deg(diff_h, currentAngle)

        await robot.turn_in_place(cozmo.util.degrees(turn_angle)).wait_for_completed()
        await robot.drive_straight(cozmo.util.distance_mm(final_distance), cozmo.util.speed_mmps(100)).wait_for_completed()

        currentAngle = turn_angle
        count = count + 1
    # await robot.turn_in_place(cozmo.util.degrees(turn_angle)).wait_for_completed()
    # while swag_number < len(path) - 1:
    #     current_node = path[swag_number]
    #     next_node = path[swag_number + 1]
    #     await robot.go_to_pose(cozmo.util.Pose(next_node.x - current_node.x, next_node.y - current_node.y, 0, angle_z=cozmo.util.Angle(degrees=np.arctan2(next_node.y - current_node.y, next_node.x - current_node.x))), relative_to_robot=True).wait_for_completed()
    #     swag_number = swag_number + 1

    diff_x = center.x - path[-1].x
    diff_y = center.y - path[-1].y

    diff_h = math.degrees(math.atan2(diff_y, diff_x))
    final_distance = math.sqrt((diff_y ** 2) + (diff_x ** 2))
    turn_angle = diff_heading_deg(diff_h, currentAngle)
    await robot.turn_in_place(cozmo.util.degrees(turn_angle)).wait_for_completed()

    # if len(cmap.get_goals()) == 0:
    #     cmap.add_goal(Node((24*25.4, 12*25.4)))
    #     RRT(cmap, current_pose)
        # swag_number = 0
    #     path = cmap.get_smooth_path()
    #     while center == None:
    #         next_node = path[swag_number]
    # await robot.go_to_pose(cozmo.util.Pose(300 - current_pose.x, 300 - current_pose.y, 0, angle_z=cozmo.util.Angle(degrees=np.arctan2(300 - current_pose.y, 300 - current_pose.x))), relative_to_robot=True).wait_for_completed()
    #         current_pose = next_node
    #         swag_number = swag_number + 1
    #         update, center = detect_cube_and_update_cmap(cozmo, marker_dictionary, current_pose)
    #         if update:
    #             RRT(cmap, current_pose)
    #             swag_number = 0

    # # Heading to Goal
    # node_number = 0
    # while current_pose != center:
    #     next_node = cmap.get_smooth_path()[node_number]
    #     await cozmo.robot.go_to_pose(cozmo.util.Pose(next_node.x - current_pose.x, next_node.y - current_pose.y, 0, angle_z=cozmo.util.Angle(degrees=arctan2(next_node.y - current_pose.y, next_node.x - current_pose.x))), relative_to_robot=true)
    #     current_pose = cmap.get_smooth_path[node_number]
    #     node_number = node_number + 1
    #     update, center = detect_cube_and_update_cmap(cozmo, marker_dictionary, current_pose)
    #     if update:
    #         RRT(cmap, current_pose)
    #         node_number = 0



def get_global_node(angle, position, node):
    transformation_matrix = [[math.cos(angle), -math.sin(angle), position.x],
                            [math.sin(angle), math.cos(angle), position.y],
                            [0, 0, 1]]
    local_pos = [node.x, node.y, 1]
    product = np.matmul(transformation_matrix, local_pos)
    return Node((product[0], product[1]))

async def detect_cube_and_update_cmap(robot, marked, cozmo_pos):
    """Helper function used to detect obstacle cubes and the goal cube.
       1. When a valid goal cube is detected, old goals in cmap will be cleared and a new goal corresponding to the approach position of the cube will be added.
       2. Approach position is used because we don't want the robot to drive to the center position of the goal cube.
       3. The center position of the goal cube will be returned as goal_center.

        Arguments:
        robot -- provides the robot's pose in G_Robot
                 robot.pose is the robot's pose in the global coordinate frame that the robot initialized (G_Robot)
                 also provides light cubes
        cozmo_pose -- provides the robot's pose in G_Arena
                 cozmo_pose is the robot's pose in the global coordinate we created (G_Arena)
        marked -- a dictionary of detected and tracked cubes (goal cube not valid will not be added to this list)

        Outputs:
        update_cmap -- when a new obstacle or a new valid goal is detected, update_cmap will set to True
        goal_center -- when a new valid goal is added, the center of the goal cube will be returned
    """
    global cmap

    # Padding of objects and the robot for C-Space
    cube_padding = 60.
    cozmo_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)

    for obj in robot.world.visible_objects:

        if obj.object_id in marked:
            continue

        # Calculate the object pose in G_Arena
        # obj.pose is the object's pose in G_Robot
        # We need the object's pose in G_Arena (object_pos, object_angle)
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y

        object_pos = Node((cozmo_pos.x+dx, cozmo_pos.y+dy))
        object_angle = obj.pose.rotation.angle_z.radians

        # The goal cube is defined as robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id
        if robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id == obj.object_id:

            # Calculate the approach position of the object
            local_goal_pos = Node((0, -cozmo_padding))
            goal_pos = get_global_node(object_angle, object_pos, local_goal_pos)

            # Check whether this goal location is valid
            if cmap.is_inside_obstacles(goal_pos) or (not cmap.is_inbound(goal_pos)):
                print("The goal position is not valid. Please remove the goal cube and place in another position.")
            else:
                cmap.clear_goals()
                cmap.add_goal(goal_pos)
                goal_center = object_pos

        # Define an obstacle by its four corners in clockwise order
        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)):
        if (sys.argv[i] == "-robot"):
            robotFlag = True
    if (robotFlag):
        cmap = CozMap("maps/emptygrid.json", node_generator)
        robot_thread = RobotThread()
        robot_thread.start()
    else:
        cmap = CozMap("maps/map2.json", node_generator)
        sim = RRTThread()
        sim.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()