import numpy as np
import matplotlib.pyplot as plt

class Trajectory:
    def __init__(self, Amax, Jmax, Vmax, Dist):
        self.Amax = Amax
        self.Jmax = Jmax
        self.Vmax = Vmax
        self.Distance = Dist
        self.T = [0.0] * 7
        self.A = [0.0] * 7
        self.B = [0.0] * 7
        self.C = [0.0] * 7
        self.D = [0.0] * 7

    def calculate_trajectory(self):
        self.T[6] = (self.Amax / self.Jmax) + (self.Vmax / self.Amax) + (self.Distance / self.Vmax)
        self.T[0] = self.Amax / self.Jmax
        self.T[1] = self.Vmax / self.Amax
        self.T[2] = (self.Amax / self.Jmax) + (self.Vmax / self.Amax)
        self.T[3] = self.T[6] - self.T[2]
        self.T[4] = self.T[6] - self.T[1]
        self.T[5] = self.T[6] - self.T[0]

        self.A[0] = self.Jmax
        self.A[1] = 0
        self.A[2] = -self.Jmax
        self.A[3] = 0
        self.A[4] = -self.Jmax
        self.A[5] = 0
        self.A[6] = self.Jmax

        self.B[0] = 0
        self.B[1] = self.Amax
        self.B[2] = self.Amax + (self.Jmax * self.T[1])
        self.B[3] = 0
        self.B[4] = self.Jmax * self.T[3]
        self.B[5] = -self.Amax
        self.B[6] = -self.Amax - (self.Jmax * self.T[5])

        self.C[0] = 0
        self.C[1] = ((self.A[0] * (self.T[0] ** 2)) / 2 + self.B[0] * self.T[0] + self.C[0]) - \
                    ((self.A[1] * (self.T[0] ** 2)) / 2 + self.B[1] * self.T[0])
        self.C[2] = ((self.A[1] * (self.T[1] ** 2)) / 2 + self.B[1] * self.T[1] + self.C[1]) - \
                    ((self.A[2] * (self.T[1] ** 2)) / 2 + self.B[2] * self.T[1])
        self.C[3] = ((self.A[2] * (self.T[2] ** 2)) / 2 + self.B[2] * self.T[2] + self.C[2]) - \
                    ((self.A[3] * (self.T[2] ** 2)) / 2 + self.B[3] * self.T[2])
        self.C[4] = ((self.A[3] * (self.T[3] ** 2)) / 2 + self.B[3] * self.T[3] + self.C[3]) - \
                    ((self.A[4] * (self.T[3] ** 2)) / 2 + self.B[4] * self.T[3])
        self.C[5] = ((self.A[4] * (self.T[4] ** 2)) / 2 + self.B[4] * self.T[4] + self.C[4]) - \
                    ((self.A[5] * (self.T[4] ** 2)) / 2 + self.B[5] * self.T[4])
        self.C[6] = ((self.A[5] * (self.T[5] ** 2)) / 2 + self.B[5] * self.T[5] + self.C[5]) - \
                    ((self.A[6] * (self.T[5] ** 2)) / 2 + self.B[6] * self.T[5])

        self.D[0] = 0
        self.D[1] = ((self.A[0] * (self.T[0] ** 3)) / 6 + (self.B[0] * (self.T[0] ** 2)) / 2 + self.C[0] * self.T[0] + self.D[0]) - \
                    ((self.A[1] * (self.T[0] ** 3)) / 6 + (self.B[1] * (self.T[0] ** 2)) / 2 + self.C[1] * self.T[0])
        self.D[2] = ((self.A[1] * (self.T[1] ** 3)) / 6 + (self.B[1] * (self.T[1] ** 2)) / 2 + self.C[1] * self.T[1] + self.D[1]) - \
                    ((self.A[2] * (self.T[1] ** 3)) / 6 + (self.B[2] * (self.T[1] ** 2)) / 2 + self.C[2] * self.T[1])
        self.D[3] = ((self.A[2] * (self.T[2] ** 3)) / 6 + (self.B[2] * (self.T[2] ** 2)) / 2 + self.C[2] * self.T[2] + self.D[2]) - \
                    ((self.A[3] * (self.T[2] ** 3)) / 6 + (self.B[3] * (self.T[2] ** 2)) / 2 + self.C[3] * self.T[2])
        self.D[4] = ((self.A[3] * (self.T[3] ** 3)) / 6 + (self.B[3] * (self.T[3] ** 2)) / 2 + self.C[3] * self.T[3] + self.D[3]) - \
                    ((self.A[4] * (self.T[3] ** 3)) / 6 + (self.B[4] * (self.T[3] ** 2)) / 2 + self.C[4] * self.T[3])
        self.D[5] = ((self.A[4] * (self.T[4] ** 3)) / 6 + (self.B[4] * (self.T[4] ** 2)) / 2 + self.C[4] * self.T[4] + self.D[4]) - \
                    ((self.A[5] * (self.T[4] ** 3)) / 6 + (self.B[5] * (self.T[4] ** 2)) / 2 + self.C[5] * self.T[4])
        self.D[6] = ((self.A[5] * (self.T[5] ** 3)) / 6 + (self.B[5] * (self.T[5] ** 2)) / 2 + self.C[5] * self.T[5] + self.D[5]) - \
                    ((self.A[6] * (self.T[5] ** 3)) / 6 + (self.B[6] * (self.T[5] ** 2)) / 2 + self.C[6] * self.T[5])

        self.VMCal = self.Vmax

    def trajectory_evaluation(self,t):
        if 0 <= t < self.T[0]:
            QJ = self.A[0]
            QA = self.A[0] * t + self.B[0]
            QV = (self.A[0] * (t ** 2)) / 2 + self.B[0] * t + self.C[0]
            QVP = (self.A[0] * (t ** 2)) / 2 + self.B[0] * t + self.C[0]
            QX = self.A[0] * (t ** 3) / 6 + self.B[0] * (t ** 2) / 2 + self.C[0] * t + self.D[0]
        elif self.T[0] <= t < self.T[1]:
            QJ = self.A[1]
            QA = self.A[1] * t + self.B[1]
            QV = (self.A[1] * (t ** 2)) / 2 + self.B[1] * t + self.C[1]
            QVP = (self.A[1] * (t ** 2)) / 2 + self.B[1] * t + self.C[1]
            QX = self.A[1] * (t ** 3) / 6 + self.B[1] * (t ** 2) / 2 + self.C[1] * t + self.D[1]
        elif self.T[1] <= t < self.T[2]:
            QJ = self.A[2]
            QA = self.A[2] * t + self.B[2]
            QV = (self.A[2] * (t ** 2)) / 2 + self.B[2] * t + self.C[2]
            QVP = (self.A[2] * (t ** 2)) / 2 + self.B[2] * t + self.C[2]
            QX = self.A[2] * (t ** 3) / 6 + self.B[2] * (t ** 2) / 2 + self.C[2] * t + self.D[2]
        elif self.T[2] <= t < self.T[3]:
            QJ = self.A[3]
            QA = self.A[3] * t + self.B[3]
            QV = (self.A[3] * (t ** 2)) / 2 + self.B[3] * t + self.C[3]
            QVP = (self.A[3] * (t ** 2)) / 2 + self.B[3] * t + self.C[3]
            QX = self.A[3] * (t ** 3) / 6 + self.B[3] * (t ** 2) / 2 + self.C[3] * t + self.D[3]
        elif self.T[3] <= t < self.T[4]:
            QJ = self.A[4]
            QA = self.A[4] * t + self.B[4]
            QV = (self.A[4] * (t ** 2)) / 2 + self.B[4] * t + self.C[4]
            QVP = (self.A[4] * (t ** 2)) / 2 + self.B[4] * t + self.C[4]
            QX = self.A[4] * (t ** 3) / 6 + self.B[4] * (t ** 2) / 2 + self.C[4] * t + self.D[4]
        elif self.T[4] <= t < self.T[5]:
            QJ = self.A[5]
            QA = self.A[5] * t + self.B[5]
            QV = (self.A[5] * (t ** 2)) / 2 + self.B[5] * t + self.C[5]
            QVP = (self.A[5] * (t ** 2)) / 2 + self.B[5] * t + self.C[5]
            QX = self.A[5] * (t ** 3) / 6 + self.B[5] * (t ** 2) / 2 + self.C[5] * t + self.D[5]
        elif self.T[5] <= t < self.T[6]:
            QJ = self.A[6]
            QA = self.A[6] * t + self.B[6]
            QV = (self.A[6] * (t ** 2)) / 2 + self.B[6] * t + self.C[6]
            QVP = (self.A[6] * (t ** 2)) / 2 + self.B[6] * t + self.C[6]
            QX = self.A[6] * (t ** 3) / 6 + self.B[6] * (t ** 2) / 2 + self.C[6] * t + self.D[6]
        else:
            QJ = 0
            QA = 0
            QV = 0
            QVP = 0
            QX = self.Distance
        return QJ, QA, QV, QVP, QX

