# Cozmo Group 1 - InCozMito
# Jackson Cook
# Harsh Patel

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
    # No new movement update, so particles remain stationary
    if (odom[0] is 0) and (odom[1] is 0) and (odom[2] is 0):
        return particles
    motion_particles = []
    for particle in particles:
        # Transform reference frame of robot to that of the particle
        rotated_x, rotated_y = rotate_point(odom[0], odom[1], particle.h)
        # Update the coordinates and heading of each particle
        new_local_odom = (particle.x + rotated_x, particle.y + rotated_y, particle.h + odom[2])
        # Add Motion Gaussian noise
        new_pos = add_odometry_noise(new_local_odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
        motion_particles.append(Particle(new_pos[0], new_pos[1], new_pos[2]))
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
    particleWeightArray = []

    # Equalize weights if no new sensor data is retrieved
    if len(measured_marker_list) == 0:
        for particle in particles:
            particleWeightArray.append((particle, 1 / len(particles)))
    else:
        for particle in particles:
            # Quality assurance checks:
            # Particle out of bounds should have zero weight
            if not grid.is_in(particle.x, particle.y):
                particleWeightArray.append((particle, 0))
                continue
            # Particle inside walls or obstacles should have zero weight
            if not grid.is_free(particle.x, particle.y):
                particleWeightArray.append((particle, 0))
                continue
            # Extract the markers the particle can see
            particleMarkers = particle.read_markers(grid)
            # Extract the particle-robot marker pairings
            matchedMarkers, markerNumDifference = matchMarkers(measured_marker_list, particleMarkers)

            # Define constants and starting values for weight update
            probability = 1
            constant_distanceSigma = 2 * (MARKER_TRANS_SIGMA ** 2)
            constant_angleSigma = 2 * (MARKER_ROT_SIGMA ** 2)

            # Define starting values for max particle deviations
            max_DistanceDeviation = 0
            max_AngleDeviation = (ROBOT_CAMERA_FOV_DEG ** 2) / constant_angleSigma
            # Update probability based on matched markers
            for particle_marker, robot_marker in matchedMarkers:
                # Find the distance between the two markers
                distanceBetweenMatchedMarkers = grid_distance(particle_marker[0], particle_marker[1], robot_marker[0], robot_marker[1])
                # Find the angle between the two markers
                angleBetweenMatchedMarkers = diff_heading_deg(particle_marker[2], robot_marker[2])
                # Apply Gaussian formula from the slides
                particleDistTerm = (distanceBetweenMatchedMarkers ** 2) / constant_distanceSigma
                particleAngleTerm = (angleBetweenMatchedMarkers ** 2) / constant_angleSigma
                # Keep track of the max distance between markers seen thus far
                max_DistanceDeviation = max(max_DistanceDeviation, particleDistTerm)
                probability *= np.exp(-(particleDistTerm + particleAngleTerm))

            # Need some way to punish particles who saw too few or too many markers
            # compared to the robot. Reducing the probability to zero would be too harsh,
            # so reducing the probablity by the furthest possible "imaginary" marker able
            # to be seen by that particle
            for num in range(markerNumDifference):
                probability *= np.exp(-(max_DistanceDeviation + max_AngleDeviation))

            particleWeightArray.append((particle, probability))

    particleWeightArray, totalParticlesRemoved = pruneAndCleanBottom(particleWeightArray, 0.01)
    totalWeightSum = getTotalWeight(particleWeightArray)
    # Normalize Particles Based on Total Weight
    normalizedParticleArray, normalizedWeightArray = normalizeParticles(particleWeightArray, totalWeightSum)
    # Resamples Particles Based on Weight
    resampledParticles = resampleParticles(normalizedParticleArray, normalizedWeightArray)
    # Create new random particles equal to the total number pruned
    randomParticles = Particle.create_random(totalParticlesRemoved, grid)[:]
    # Create output particle list
    measured_particles = []
    # Add noise to resampled particles
    noisyParticles = []
    for particle in resampledParticles:
        noisyParticleTraits = add_odometry_noise((particle.x, particle.y, particle.h), ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
        noisyParticle = Particle(noisyParticleTraits[0], noisyParticleTraits[1], noisyParticleTraits[2])
        noisyParticles.append(noisyParticle)
    # Combine noisy, resampled particles and random particles
    measured_particles = noisyParticles + randomParticles
    return measured_particles

def matchMarkers(robotMarkers, particleMarkers):
    matchedMarkers = []
    # Number of seen markers different between robot and particle
    markerNumDifference = int(math.fabs(len(robotMarkers) - len(particleMarkers)))

    for robot_marker in robotMarkers:
        # Quality Assurance Check
        # No markers should match if particle sees nothing
        if len(particleMarkers) == 0:
            break
        # Add noise to robot's measurements
        noisyMarkerX, noisyMarkerY, noisyMarkerH = add_marker_measurement_noise(robot_marker, MARKER_TRANS_SIGMA, MARKER_ROT_SIGMA)

        # Simple greedy algorithm to find closest particle_marker to robot_marker
        min_particle_marker = particleMarkers[0]
        min_marker_distance = grid_distance(noisyMarkerX, noisyMarkerY, min_particle_marker[0], min_particle_marker[1])
        # Loop through and find closest particle_marker
        for particle_marker in particleMarkers:
            noisyParticleX, noisyParticleY, noisyParticleH = add_marker_measurement_noise(particle_marker, MARKER_TRANS_SIGMA, MARKER_ROT_SIGMA)
            marker_distance = grid_distance(noisyMarkerX, noisyMarkerY, noisyParticleX, noisyParticleY)
            if marker_distance < min_marker_distance:
                min_particle_marker = particle_marker
                min_marker_distance = marker_distance

        # Append the closest marker found to the matchedMarkers list
        matchedMarkers.append((min_particle_marker, robot_marker))
        # Remove the min_particle_marker so that it can't be matched again
        particleMarkers.remove(min_particle_marker)
    # Return the matched markers and the number of markers different between particle and robot
    return matchedMarkers, markerNumDifference

def getTotalWeight(particlesWithWeights):
    totalWeightSum = 0
    for particle, weight in particlesWithWeights:
        totalWeightSum += weight
    return totalWeightSum

def pruneAndCleanBottom(particlesWithWeights, pruning_percentage = 0.01):
    amountToPrune = int(PARTICLE_COUNT * pruning_percentage)
    # Sort the particles by weight
    particlesWithWeights.sort(key = lambda x: x[1])
    # Prune off bottom "pruning_percentage"
    particlesWithWeights = particlesWithWeights[amountToPrune:]
    # Prune remaining particles with weights of zero
    zeroWeightNum = 0
    for particle, weight in particlesWithWeights:
        if weight == 0:
            zeroWeightNum += 1
    particlesWithWeights = particlesWithWeights[zeroWeightNum:]
    # Report back pruned particles and total number of particles removed
    return (particlesWithWeights, amountToPrune + zeroWeightNum)

def normalizeParticles(particlesWithWeights, totalWeightSum):
    # Create new particles and "assign" them normalized weights
    newParticleList = []
    normalizedWeightList = []
    for particle, weight in particlesWithWeights:
        newParticle = Particle(particle.x, particle.y, particle.h)
        # Weight normalization step
        normalizedWeightList.append(weight / totalWeightSum)
        newParticleList.append(newParticle)
    return (newParticleList, normalizedWeightList)

def resampleParticles(particles, weights):
    # Resample old particles based on their weight
    resampledParticles = []
    if len(particles) != 0:
        # Resampling step
        resampledParticles = np.random.choice(particles, size = len(particles), replace = True, p = weights)
    return resampledParticles