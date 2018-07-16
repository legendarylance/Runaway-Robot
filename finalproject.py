import sys
import os
import numpy as np
import math

class matrix:
    # implements basic operations of a matrix class
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0

    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1

    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res

    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res

    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions

    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = math.sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res

    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res

    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res

    def __repr__(self):
        return repr(self.value)

# Kalman Filter
class KalmanFilter(object):

  def __init__(self, pos, top_left, bot_right):
    self.pos        = pos # x, y coordinates
    self.top_left = top_left
    self.bot_right = bot_right
    self.v          = None # heading direction
    self.vx_pred = None
    self.vy_pred = None
    self.x_pred  = None # x prediction
    self.y_pred  = None # y prediction
    self.x = matrix([[self.pos[0]], [self.pos[1]], [0.], [0.]]) # initial state (location and velocity)
    dt = 1.0
    self.u = 0.005
    self.Bu = matrix([[dt**2 / 2 * self.u], [dt**2 / 2 * self.u], [dt * self.u], [dt * self.u]]) # external motion
    self.P = matrix([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 1000., 0.], [0., 0., 0., 1000.]]) # initial uncertainty: 0 for positions x and y, 1000 for the two velocities
    self.F = matrix([[1., 0., dt, 0.], [0., 1., 0., dt], [0., 0., 1., 0.], [0., 0., 0., 1.]]) # next state function
    self.Ex = matrix([[dt**4/4, 0., dt**3/2, 0.], [0., dt**4/4, 0., dt**3 / 2], [dt**3/2, 0., dt**2, 0.], [0., dt**3/2, 0., dt**2]])
    self.H = matrix([[1., 0., 0., 0.], [0., 1., 0., 0.]]) # measurement function: reflect the fact that we observe x and y but not the two velocities
    uncertainty = 1.
    self.R = matrix([[uncertainty, 0.], [0., uncertainty]]) # measurement uncertainty
    self.I = matrix([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]) # 4d identity matrix

  def process(self, measurement):
    self.x = (self.F * self.x) + self.Bu
    self.P = self.F * self.P * self.F.transpose() + self.Ex
    # measurement update
    #error = np.sqrt((self.x[0] - measurement[0])**2 + (self.x[1] - measurement[1])**2)
    if measurement:
      self.Z = matrix([measurement])
    else:
      self.Z = matrix([[self.x.value[0][0], self.x.value[1][0]]])

    self.y = self.Z.transpose() - (self.H * self.x)
    self.S = self.H * self.P * self.H.transpose() + self.R

    self.K = self.P * self.H.transpose() * self.S.inverse()
    self.x = self.x + (self.K * self.y)

    #[x y x_dot y_dot]
    if not measurement:
      if self.x.value[0][0] < self.top_left[0]:
        self.x.value[0][0] = self.top_left[0] + (self.top_left[0] - self.x.value[0][0])
        self.x.value[2][0] = -self.x.value[2][0]
      if self.x.value[1][0] < self.top_left[1]:
        self.x.value[1][0] = self.top_left[1] + (self.top_left[1] - self.x.value[1][0])
        self.x.value[3][0] = -self.x.value[3][0]

      if self.x.value[0][0] > self.bot_right[0]:
        self.x.value[0][0] = self.bot_right[0] + (self.bot_right[0] - self.x.value[0][0])
        self.x.value[2][0] = -self.x.value[2][0]
      if self.x.value[1][0] > self.bot_right[1]:
        self.x.value[1][0] = self.bot_right[1] + (self.bot_right[1] - self.x.value[1][0])
        self.x.value[3][0] = -self.x.value[3][0]

    self.P = (self.I - (self.K * self.H)) * self.P

    return (self.x.value[0][0], self.x.value[1][0])


# KNN (K-Nearest Neighbor)
def distance(p1, p2):
  return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

def l2_d(list1, list2):
  return distance(list1[0], list2[0]) + distance(list1[1], list2[1]) + distance(list1[2], list2[2])+ distance(list1[3], list2[3]) + distance(list1[4], list2[4])

class KNN(object):

  def __init__(self, pos, top_left, bot_right):
    self.pos        = pos # x, y coordinates
    self.top_left = top_left
    self.bot_right = bot_right
    self.x_pred  = None # x prediction
    self.y_pred  = None # y prediction
    self.pos = [(pos[0], pos[1])]
    self.training_data = []

  def train(self, training_file):
    text  = open(training_file, "r")

    # create data
    with text as f:
      for i, line in enumerate(f):
        x, y = line.split(",")
        self.training_data.append((int(x), int(y)))

  def process(self, measurement):
    toretX, toretY = 0, 0
    if measurement:
      self.pos.append(measurement)
      toretX, toretY = measurement
    else:
      if len(self.pos) > 5:
        list1 = (self.pos[-1], self.pos[-2], self.pos[-3], self.pos[-4], self.pos[-5])
        best = 10000000000
        best_idx = 0
        for i in range(len(self.training_data) - 2, 3, -1):
          list2 = (self.training_data[i],self.training_data[i-1],self.training_data[i-2],self.training_data[i-3],self.training_data[i-4])
          t = l2_d(list1, list2)

          if t < best:
            best, best_idx = t, i

          if best < 1:
            break
        dx = ((self.training_data[best_idx+1][0] - self.training_data[best_idx][0]))
        dy = ((self.training_data[best_idx+1][1] - self.training_data[best_idx][1]))
        toretX = self.pos[-1][0] + dx
        toretY = self.pos[-1][1] + dy
        if toretX < self.top_left[0]:
          toretX += 2 * (self.top_left[0] - toretX)
        if toretX > self.bot_right[0]:
          toretX += 2 * (self.bot_right[0] - toretX)
        if toretY < self.top_left[1]:
          toretY += 2 * (self.top_left[1] - toretY)
        if toretY > self.bot_right[1]:
          toretY += 2 * (self.bot_right[1] - toretY)
        if not measurement:
          self.pos.append((toretX, toretY))
    return (toretX, toretY)


def distance(p1, p2):
  return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 


filename = sys.argv[1]
x, y = open(filename, 'r').readlines()[-1].split(',')
results = []

def main():
  # read input
  text  = open(sys.argv[1], "r")
  # create data
  data = []
  with text as f:
    for i, line in enumerate(f):
      x, y = line.split(",")
      data.append((int(x), int(y)))

  t_0 = 1700
  p_kf = KalmanFilter(pos = data[t_0], top_left = (240, 105), bot_right = (1696, 974))
  p_knn = KNN(pos = data[t_0], top_left = (240, 105), bot_right = (1696, 974))
  p_knn.train(os.path.dirname(os.path.realpath(__file__)) + '/training_data.txt')



  MAX_FRAME = len(data)
  i       = t_0 # start at this iteration
  iMax    = MAX_FRAME + 60# 1200 # stop at this iteration
  FLIP_THRESHOLD = 50
  dataX = []
  dataY = []
  for j in range(i - 200, len(data)):
    dataX.append(data[j][0])
    dataY.append(data[j][1]) 
  print np.std(dataX), np.std(dataY), np.mean(dataX), np.mean(dataY)
  if (np.std(dataX) <= FLIP_THRESHOLD) and (np.std(dataY) <= FLIP_THRESHOLD):
    x_avg = int(np.mean(dataX))
    y_avg = int(np.mean(dataY))
    #print data[i:, 0], data[i:][1]
    for j in range(60):
      results.append([x_avg, y_avg])
  else:
    while True:
      data_point = None
      if i < MAX_FRAME:
        data_point = data[i]
      result_kf = p_kf.process(data_point)
      result_knn = p_knn.process(data_point)
    
      if i >= len(data):
        x_pred = int((result_kf[0] + result_knn[0]) / 2)
        y_pred = int((result_kf[1] + result_knn[1]) / 2)
    
        results.append([x_pred, y_pred])
      i += 1
      if i == iMax:
        break

if __name__ == "__main__":
  main()
print len(results)
with open('prediction.txt', 'w') as f:
    for i in range(60):
        print >> f, '%s,%s' % (results[i][0], results[i][1])
