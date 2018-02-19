from grid import *
from particle import Particle
from utils import *
from setting import *

import numpy as np
import random
random.seed(RANDOM_SEED)

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    if (odom[0] is 0) and (odom[1] is 0) and (odom[2] is 0):
        return particles
    motion_particles = []
    for particle in particles:
        rotated_x, rotated_y = rotate_point(odom[0], odom[1], particle.h)
        new_local_odom = (particle.x + rotated_x, particle.y + rotated_y, particle.h + odom[2])
        new_pos = add_odometry_noise(new_local_odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
        motion_particles.append(Particle(new_pos[0], new_pos[1], new_pos[2]))
        # noisy_odom = add_odometry_noise(odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
        # new_particle = Particle(x.x + noisy_odom[0],x.y + noisy_odom[1],x.h + noisy_odom[2])
        # motion_particles.append(new_particle)
    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    noisy_true_markers = [add_marker_measurement_noise(x, MARKER_TRANS_SIGMA, MARKER_ROT_SIGMA) for x in measured_marker_list]

    particle_dict = dict()
    for particle in particles:
        prob = 1
        particle_markers = particle.read_markers(grid)
        noisy_particle_markers = [add_marker_measurement_noise(x, MARKER_TRANS_SIGMA, MARKER_ROT_SIGMA) for x in particle_markers]


    # Assign weight of 1 to all particles
    # particle_dict = dict()
    # for particle in particles:
    #     if grid.is_in(particle.x, particle.y):
    #         particle_dict[particle] = 1
    #     else:
    #         particle_dict[particle] = 0

    # measured_particles = []
    # alpha = 0
    # for particle in particle_dict.keys():
    #     alpha += particle_dict[particle]
    # for particle in particle_dict.keys():
    #     particle_dict[particle] = particle_dict[particle] / alpha

    # for particle in particles:
    #     if grid.is_in(particle.x, particle.y):
    #         new_particle = np.random.choice(list(particle_dict.keys()), replace = True, p = list(particle_dict.values()))
    #         measured_particles.append(new_particle)
    #     else:
    #         measured_particles += Particle.create_random(1, grid)
    #
    #
    measured_particles = []
    for particle in particles:
        if grid.is_in(particle.x, particle.y):
            measured_particles.append(particle)
        else:
            measured_particles += Particle.create_random(1, grid)
    return measured_particles


