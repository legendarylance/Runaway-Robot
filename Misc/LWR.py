import sys
import numpy as np
import cv2
import pandas as pd
from math import *

def gaussian(a, sigma):
  # calculates the probability of x for 1-dim Gaussian with mean mu and var sigma
  return np.exp(-(a ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))

def distance_between(point1, point2):
  # computes distance between point1 and point2
  x1, y1 = point1
  x2, y2 = point2
  return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def angle_trunc(a):
  # truncate between -pi and pi
  while a < 0.0:
    a += np.pi * 2
  return ((a + np.pi) % (np.pi * 2)) - np.pi


class LocallyWeightedRegression(object):

    def __init__(self, training_input, training_output, **kwargs):
        # t (frame number)
        self.training_inputs  = [[training_input]]
        # either x or y
        self.training_outputs  = [[training_output]]
        self.output_prediction = training_output

    def gaussian_kernel(self, x, x0, a, c):
        """
        Gaussian kernel.

        :Parameters:
          - `x`: nearby datapoint we are looking at.
          - `x0`: data point we are trying to estimate.
          - `c`, `a`: kernel parameters.
        """
        # Euclidian distance
        diff = x - x0
        dot_product = diff * diff.T
        return a * np.exp(dot_product / (-2.0 * c**2))


    def get_weights(self, training_inputs, data_point, a, c):
        """
        Function that calculates weight matrix for a given data point and training
        data.

        :Parameters:
        - `training_inputs`: training data set the weights should be assigned to.
        - `datapoint`: data point we are trying to predict.
        - `c`: kernel function parameter

        :Returns:
        NxN weight matrix, there N is the size of the `training_inputs`.
        """
        x = np.mat(training_inputs)
        n_rows = x.shape[0]
        # Create diagonal weight matrix from identity matrix
        weights = np.mat(np.eye(n_rows))
        for i in xrange(n_rows):
            weights[i, i] = self.gaussian_kernel(data_point, x[i], a, c)

        return weights


    def lwr_predict(self, data_point, a, c):

        weights = self.get_weights(self.training_inputs, data_point, a, c)

        x = np.mat(self.training_inputs)
        y = np.mat(self.training_outputs)

        xt = x.T * (weights * x)

        # Singularity Check
        if np.linalg.det(xt) != 0:
            betas = xt.I * (x.T * (weights * y))
            self.output_prediction =  data_point * betas
        else:
            self.output_prediction = [0,0]

    def add_data(self, training_input, training_output):
        self.training_inputs.append([training_input])
        self.training_outputs.append([training_output])

def main():

  # world
  rect   = [(240, 105), (1696, 974)] # top left and bottom right vertices
  circle = [(1004, 539), 10] # center and radius

  # read input
  text  = open(sys.argv[1], "r")
  video = cv2.VideoCapture(sys.argv[2])

  # create data
  data = []
  with text as f:
    for i, line in enumerate(f):
      x, y = line.split(",")
      data.append((int(x), int(y)))

  # grab first frame
  grab, frame = video.read()

  # initialize template
  template = np.zeros(frame.shape)

  # draw world
  midX, midY = circle[0]
  radius     = circle[1]
  xl, yu     = rect[0]
  xr, yd     = rect[1]
  cv2.circle(template, (midX, midY), 95, (0, 255, 0), radius)
  cv2.rectangle(template, (int(xl), int(yu)), (int(xr), int(yd)), (0, 255, 0), 10)

  # kernel parameters
  c = 1.0 # sigma
  a = 1./(c * sqrt(2*pi))

  i       = 0 # start at this iteration
  iMax    = 300 # stop at this iteration
  results = []
  while True:
    if i == 0:
        vx_lwr = LocallyWeightedRegression(training_input = 0, training_output = data[i][0])
        vy_lwr = LocallyWeightedRegression(training_input = 0, training_output = data[i][1])

        x_pred, y_pred = data[i]

    else:
        vx_lwr.add_data(i, data[i][0] - data[i-1][0])
        vy_lwr.add_data(i, data[i][1] - data[i-1][1])

        # LWR of current point
        vx_lwr.lwr_predict(i, a, c)
        vy_lwr.lwr_predict(i, a, c)
        x_pred = data[i][0] + vx_lwr.output_prediction
        y_pred = data[i][1] + vy_lwr.output_prediction

    x, y           = data[i + 1] # actual for i + 1
    dist           = distance_between((x_pred, y_pred), (x, y)) # error distance

    # time step i + 1, x prediction, y prediction, x actual, y actual, error distance
    results.append([i + 1, x_pred, y_pred, x, y, dist])

    i += 1
    if i == iMax:
      break

    # show results real time
    cv2.circle(template, (x, y), 2, (0, 255, 0), 2) # actual for i + 1
    cv2.circle(template, (x_pred, y_pred), 2, (0, 0, 255), 2) # prediction for i + 1
    out = cv2.resize(template, (0, 0), fx = 0.5, fy = 0.5)
    cv2.imshow("out", out)
    if cv2.waitKey(2) & 0xFF == ord('q'):
      break

  # freeze last frame
  cv2.imshow("out", out)
  cv2.waitKey(0)

  # write last frame
  # cv2.imwrite("frame_out.png", out)

  # create data frame of results
  columns = ["time", "x_pred", "y_pred", "x", "y", "error"]
  df      = pd.DataFrame(np.array(results), columns = columns)

  # export results
  df.to_csv("results.csv")

if __name__ == "__main__":
  main()
