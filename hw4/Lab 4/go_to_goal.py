## If you run into an "[NSApplication _setup] unrecognized selector" problem on macOS,
## try uncommenting the following snippet

# try:
#     import matplotlib
#     matplotlib.use('TkAgg')
# except ImportError:
#     pass

from skimage import color
import cozmo
import numpy as np
from numpy.linalg import inv
import threading
import time
import sys
import asyncio

from markers import detect, annotator

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *

#particle filter functionality
class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
gui = GUIWindow(grid)
pf = ParticleFilter(grid)
goal_reached = False
camera_settings = None

def compute_odometry(curr_pose, cvt_inch=True):
    '''
    Compute the odometry given the current pose of the robot (use robot.pose)

    Input:
        - curr_pose: a cozmo.robot.Pose representing the robot's current location
        - cvt_inch: converts the odometry into grid units
    Returns:
        - 3-tuple (dx, dy, dh) representing the odometry
    '''

    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees

    dx, dy = rotate_point(curr_x-last_x, curr_y-last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / grid.scale, dy / grid.scale

    return (dx, dy, diff_heading_deg(curr_h, last_h))




async def marker_processing(robot, camera_settings):
    '''
    Obtain the visible markers from the current frame from Cozmo's camera.
    Since this is an async function, it must be called using await, for example:

        markers = await marker_processing(robot, camera_settings)

    Input:
        - robot: cozmo.robot.Robot object
        - camera_settings: 3x3 matrix representing the camera calibration settings
    Returns:
        - a list of detected markers, each being a 3-tuple (rx, ry, rh)
          (as expected by the particle filter's measurement update)
    '''

    global grid

    # Wait for the latest image from Cozmo
    image_event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # Convert the image to grayscale
    image = np.array(image_event.image)
    image = color.rgb2gray(image)

    # Detect the markers
    markers = detect.detect_markers(image, camera_settings)

    # Measured marker list for the particle filter, scaled by the grid scale
    marker_list = [marker['xyh'] for marker in markers]
    marker_list = [(x/grid.scale, y/grid.scale, h) for x,y,h in marker_list]

    return marker_list


async def run(robot: cozmo.robot.Robot):

    global flag_odom_init, last_pose
    global grid, gui, pf, camera_settings

    # start streaming
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    # Obtain the camera intrinsics matrix
    fx, fy = robot.camera.config.focal_length.x_y
    cx, cy = robot.camera.config.center.x_y
    camera_settings = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float)
    sm = StateMachine();
    sm.start(robot);


class StateMachine:

    def start(self, robot):
        self.state = Localizing()
        while True:
            await self.state.run(robot)

class State(object):
    def run(self, robot):
        assert 0
    def next(self):
        assert 0

## Particle filter is localizing robot

class Localizing(State):

    async def run(self, robot):
        if robot.is_picked_up:
            return PickedUp
        elif converge(pf.particles):
            return SeekingGoal
        else:
            robot = robot
            currPose = robot.pose
            odom = compute_odometry(currPose)
            global camera_settings
            markers = marker_processing(robot, camera_settings)
            estimate = pf.update(odom, markers)
            gui.show_particles(pf.particles)
            gui.show_mean(estimate[0], estimate[1], estimate[2], estimate[3])
            gui.updated.set()
            last_pose = curr_pose
            if len(markers) != 0:
                robot.drive_straight(cozmo.util.distance_mm(80), cozmo.util
                    .speed_mmps(40)).wait_for_completed()
            else:
                robot.turn_in_place(cozmo.util.degrees(-30)).wait_for_completed()
            return Localizing

## Robot has been picked up

class PickedUp(State):

    global flag_odom_init, last_pose
    global grid, gui, pf, goal_reached

    pf = ParticleFilter(grid)
    goal_reached = False

    def run(self, robot):
        robot.play_anim_trigger(cozmo.anim.Triggers.KnockOverFailure,
            in_parallel=True).wait_for_completed()
        if not robot.is_picked_up:
            return Localizing
        else:
            return PickedUp


## Robot has localized and is seeking the goal

class SeekingGoal(State):

    global goal_reached
    m_x, m_y, m_h, m_c = compute_mean_pose(pf.particles)

    final_rotate = math.degrees(math.atan2(goal[1]*25/25.6 - m_y,
        goal[0]*25/25.6 - m_x))

    def run(self, robot):
        if robot.is_picked_up:
            return PickedUp
        elif goal_reached:
            robot.turn_in_place(cozmo.util.degrees(final_rotate)).wait_for_completed()
            robot.play_anim_trigger(cozmo.anim.Triggers.AcknowledgeObject).wait_for_completed()
            return Idle
        else:
            m_x, m_y, m_h, m_c = compute_mean_pose(pf.particles)
            face_goal = diff_heading_deg(final_rotate, m_h)
            dist = math.sqrt((goal[0]*25/25.6 - m_x)**2
                + (goal[1]*25/25.6 - m_y)**2) * 25.6
            robot.turn_in_place(cozmo.util.degrees(face_goal)).wait_for_completed()
            if dist < 80:
                robot.drive_straight(cozmo.util.distance_mm(dist),
                    cozmo.util.speed_mmps(40)).wait_for_completed()
                goal_reached = true
            else:
                robot.drive_straight(cozmo.util.distance_mm(80),
                    cozmo.util.speed_mmps(40)).wait_for_completed()
            return SeekingGoal

## Robot has reached the goal

class Idle(State):
    goal_reached = False
    def run(self, robot):
        if robot.is_picked_up:
            return Localizing
        else:
            time.sleep(1)
            return Idle


def converge(particles):
    m_x, m_y, m_h, m_c = compute_mean_pose(particles)
    count = 0
    for p in particles:
        d = grid_distance(p.x, p.y, m_x, m_y)
        ang = diff_heading_deg(m_h, p.h)
        if d < 0.55 and ang < 10:
            count += 1
    return count > 0.96 * PARTICLE_COUNT

class CozmoThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    gui.show_particles(pf.particles)
    gui.show_mean(0, 0, 0)
    gui.start()