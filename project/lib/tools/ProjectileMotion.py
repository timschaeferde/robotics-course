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

        pred = self.getPosition(t)
        self.t.append(t)
        self.positions.append(position)
        self._updateVel()
        self._updateAccel()
        if pred is not None:
            error = np.linalg.norm(position - pred)
            print("Error in m: {:.5f}".format(error))

    def _updateVel(self):
        if len(self.positions) < 2:
            return
        for i in range(len(self.t) - 1):
            vel = []

            # skip already calulated ones
            if i < len(self.velocities):
                continue
            # in all 3 dimensions
            for direction in range(3):
                pos0 = self.positions[i][direction]
                pos1 = self.positions[i + 1][direction]
                timeDelta = (self.t[i + 1] - self.t[i])

                vel.append((pos1 - pos0) / timeDelta)
            #print("Vel: \t" + str(vel))
            self.velocities.append(vel)
            self.mean_velocity = np.array(self.velocities).mean(axis=0)
        return

    def _updateAccel(self):
        if len(self.velocities) < 2:
            return
        for i in range(len(self.t) - 2):
            accel = []

            # skip already calulated ones
            if i < len(self.accelerations):
                continue
            # in all 3 dimensions
            for direction in range(3):
                pos0 = self.velocities[i][direction]
                pos1 = self.velocities[i + 1][direction]
                timeDelta = (self.t[i + 2] - self.t[i + 1])

                accel.append((pos1 - pos0) / timeDelta)
            #print("Accel: \t" + str(accel))
            self.accelerations.append(accel)
            self.mean_acceleration = np.array(self.accelerations).mean(axis=0)
        return

    # formulas form: https://en.wikipedia.org/wiki/Projectile_motion
    def getPosition(self, time):
        if len(self.velocities) < 1:
            return
        delta_t = (time - self.t[-1])
        return np.array(self.positions[-1]) + self.getVelosity(time) * delta_t

    def getVelosity(self, time):
        if len(self.velocities) < 1:
            return
        return np.array(self.velocities[-1] + self.getAccelerlation(time) * (time - self.t[-1]))

    def getAccelerlation(self, time):
        return np.array([0., 0., -self.g])

    def getTimeOfArival(self, value, axis=0):
        return
