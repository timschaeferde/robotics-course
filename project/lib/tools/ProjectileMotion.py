from cmath import sqrt
import numpy as np


class ProjectileMotion:
    def __init__(self, position0=None, t0=0, mass=None, gravity=9.81):
        self.t0 = t0  # inital time in s

        if position0 is None:
            self.t = []  # inital time in ms
            self.positions = []  # positions in m
        else:
            self.t = [self.t0]  # inital time in ms
            self.positions = [position0]  # positions in m

        self.velocities = []  # velocities in m/s
        self.mean_velocity = []  # mean_velocity in m/s
        self.accelerations = []  # accelerations in m/(s*s)
        self.mean_acceleration = []  # mean_acceleration in m/(s*s)

        # self.flying = True  # Not in use yet

        self.m = mass  # mass
        self.g = gravity  # gravity

    def updatePosition(self, position, t):
        # perdict position for error check
        # pred = self.getPosition(t)

        self.t.append(t)
        self.positions.append(np.array(position))

        # calulate velocities
        self._updateVel()

        # currently not needed
        # self._updateAccel()

        # compute error form predicted value
        # if pred is not None:
        #    error = pred - position
        #    with np.printoptions(precision=3, suppress=True):
        #        print("Error in m: {}".format(error))

    def _updateVel(self):
        if len(self.positions) < 2:
            return
        if 0 == len(self.velocities):
            timeDelta = (self.t[1] - self.t[0])
            self.velocities.append(np.array((self.positions[1] - self.positions[0]) / timeDelta -
                                            (timeDelta / 2 * self.getAccelerlation(timeDelta / 2))))

        for i in range(len(self.positions) - 1):
            # skip already calulated ones
            if i < len(self.velocities) - 1:
                continue

            timeDelta = (self.t[i + 1] - self.t[i])
            # average velocity plus acceleration results in perfect velocity
            vel = ((self.positions[i + 1] - self.positions[i]) / timeDelta +
                   (timeDelta / 2 * self.getAccelerlation(timeDelta / 2)))

            # print("Vel: \t" + str(vel))
            self.velocities.append(np.array(vel))
            self.mean_velocity = np.array(self.velocities).mean(axis=0)
        return

    def _updateAccel(self):
        if len(self.velocities) < 2:
            return
        for i in range(len(self.velocities) - 1):
            # skip already calulated ones
            if i < len(self.accelerations):
                continue

            timeDelta = (self.t[i + 1] - self.t[i])
            accel = (self.velocities[i + 1] - self.velocities[i]) / timeDelta

            # print("Accel: \t" + str(accel))
            self.accelerations.append(np.array(accel))
            self.mean_acceleration = np.array(self.accelerations).mean(axis=0)
        return

    # formulas form: https://en.wikipedia.org/wiki/Projectile_motion
    def getPosition(self, time):
        if len(self.velocities) < 1:
            return
        position = np.array(self.positions[-1]) + \
            self.getVelosity(time) * (time - self.t[-1]) \
            - .5 * self.getAccelerlation(time) * \
            (time - self.t[-1])**2

        return position

    def getVelosity(self, time):
        if len(self.velocities) < 1:
            return
        return np.array(self.velocities[-1] + self.getAccelerlation(time) * (time - self.t[-1]))

    def getAccelerlation(self, time):
        return np.array([0., 0., -self.g])

    def getTimeOfArrival(self, value, axis=0):
        if len(self.velocities) < 1:
            return 0.

        # x or y axis
        if axis < 2:
            timeOfArrival = ((value - self.positions[-1][axis]) /
                             (np.array(self.velocities[-1][axis])) +
                             self.t[-1])

        # z axis
        else:
            v_0 = self.velocities[-1][axis]
            x_0 = self.positions[-1][axis]
            a_0 = self.g

            sqrt_term = sqrt(v_0**2 + 2 * a_0 * (x_0 - value))
            # time of arrival plus current time to get absolute time
            timeOfArrival = (v_0 + sqrt_term) / a_0 + self.t[-1]

        return float(timeOfArrival)
