import sys
import numpy as np
from math import pi, atan2

def gaussian(a, sigma):
  # calculates the probability of x for 1-dim Gaussian with mean mu and var sigma
  return np.exp(-(a ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))

def distance_between(p1, p2):
  # computes distance between point 1 and point 2
  x1, y1 = p1
  x2, y2 = p2
  return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def get_heading(dst_pos, src_pos):
  # compute heading direction given source and destination points
  src_x, src_y = src_pos
  dst_x, dst_y = dst_pos
  heading      = atan2(dst_y - src_y, dst_x - src_x)
  heading      = angle_trunc(heading)
  return heading

def angle_trunc(a):
  # truncate between -pi and pi
  while a < 0.0:
    a += np.pi * 2
  return ((a + np.pi) % (np.pi * 2)) - np.pi

def degree_to_radians(degree):
  return (degree / 360.) * (2. * pi)

# PF
class ParticleFilter(object):
  def __init__(self, x, **kwargs):
    self.x             = x # x, y coordinates
    self.head          = 0. # heading direction 
    self.trans         = None # translation
    self.turn          = None # turn direction
    self.num_particles = kwargs.get("num_particles", 1000) # number of particles
    self.range_head    = kwargs.get("range_head", np.pi) # initial range of heading direction
    self.range_trans   = kwargs.get("range_trans", 100) # initial range of translation
    self.range_turn    = kwargs.get("range_turn", np.pi) # initial range of turn direction
    self.sigma_head    = kwargs.get("sigma_head", 0.01) # std of heading direction
    self.sigma_trans   = kwargs.get("sigma_trans", 1.) # std of translation
    self.sigma_turn    = kwargs.get("sigma_turn", 0.01) # std of turn
    self.sigma         = kwargs.get("sigma", 1.) # std of gaussian for weights
    self.P             = [] # particles
    self.W             = [1. / float(self.num_particles)] * self.num_particles # weights
    self.x_pred        = None # x prediction
    self.y_pred        = None # y prediction

    # world
    self.wall_margin   = kwargs.get("wall_margin", 50) # define when robot touches a wall
    self.wall_dist     = None # distance to nearest wall
    self.wall          = None # 0: rectangle, 1: circle
    self.rect_side     = None # 0: left, 1: up, 2: right, 3: down
    self.rect_vertex   = None # 0: top left, 1: top right, 2: bottom right, 3: bottom left
    self.rect_out      = None # 0: left, 1: up, 2: right, 3: down
   
    # rectangle
    self.rect          = kwargs.get("rect", None) # top left and bottom right vertices of rectangle
    self.xl, self.yu   = self.rect[0][0], self.rect[0][1] # top left vertex
    self.xr, self.yd   = self.rect[1][0], self.rect[1][1] # bottom right vertex

    # circle
    self.circle        = kwargs.get("circle", None) # circle center and radius
    self.xc, self.yc   = self.circle[0]
    self.radius        = self.circle[1]

    # bounce
    self.prev_bounce   = False # bounce indicator
    self.bounce_side   = None

    self.pred_bounce   = [
    # Left
    [-60, -71.3465413392, -77, -88.1931613502, -11.0756380888, -18, -25.6299494331, -61.66081082, -72.1399059293, -60.4785847602, -70.2683568144, -78.2407877302, -63.6275199404, -61.7255726273, -76.9438703849, -67.5367875106, -76.0326327978, -69.0140515263, -77.3356246497, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 84.2600550204, 72.0165517003, 78.8801656324, 75.7787580447, 57.0075510821, 37.8062904576, 60.0424356655, 40.6544147764, 77.6609127217, 23.0509466346, 70.6134576791, 2.59721445387, 32, 22, 75.8219355853, 11, 1],
    # Top
    [176.061147049, 85.9038022149, 170.202558986, 85.8497664838, 166.11932362, 158.426401932, 162.86940926, 150.725649848, 176.456392041, 174.868252225, 141.676380511, 113.110205821, 84.0696671796, 23.4313474934, 121.849752473, 87.3620283666, 54.3299783481, 73, 116.234370718, 48.0874121706, 61, 135.579863252, 19.6796562982, 34.4059758933, 61.9909120958, 8.91917561786, 27.2118608017, 17.0210067855, 3.78448202657, 69.7142446774, 13.3900594127, 9.87831386775, 24.5307966871, 18, 14, 110.136303428, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    # Right
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -99.1864143932, -95.0303448977, -108.033920291, -110.391609707, -104.345530323, -109.731921033, -104.009478204, -127.581858216, -127, -127.56290432, -131.090912582, -127, -123, -120.379126011, -111, -102, -95, -92.3336464238, -91, 91, 92, 93, 93.9283072254, 97, 102.129676089, 103, 105, 107, 111.171176792, 103.254241086, 100.284603793, 122.471192291, 109.312257167, 107.480075614, 104.119210099, 94.8642474684, 99.6786707144, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    # Bottom
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -20, -14.4391607326, -23, -63.2956557941, -16.8198566198, -17.4868815923, -21.1561859436, -23.1874302925, -31.2889664764, -42.1002181724, -1.83711702438, -76.7851953252, -115.661895144, -7.85224761441, -49.3155617478, -173.620751832, -46.1021129968, -29.4851919926, -128.501469021, -69.0564988424, -54.4583589655, -143.223023629, -115.587050115, -129.557219951, -95, -117.668663695, -156.192295534, -162.753442813, -103.741529627, -162.933631747, -86.880097274, -153.864816794, -122.275644315, -174.255590487, -162.102831129, -170]
    ]

    # generate initial particles
    for i in range(self.num_particles):
      hp = np.random.uniform(-self.range_head, self.range_head) # heading direction
      tp = np.random.uniform(0., self.range_trans) # translation      
      rp = np.random.uniform(-self.range_turn, self.range_turn) # turning direction
      self.P.append((hp, tp, rp))

  def localize(self):
    # distance to rectangle
    rect_dist0       = abs(self.x[0] - self.xl) # left
    rect_dist1       = abs(self.x[1] - self.yu) # up
    rect_dist2       = abs(self.x[0] - self.xr) # right
    rect_dist3       = abs(self.x[1] - self.yd) # down
    rect_dist        = np.min([rect_dist0, rect_dist1, rect_dist2, rect_dist3])

    # evaluate side
    rect_side_ind    =  rect_dist < self.wall_margin
    if rect_side_ind:
      self.rect_side = np.argmin([rect_dist0, rect_dist1, rect_dist2, rect_dist3])
    else:
      self.rect_side = None

    # evaluate corners
    rect_vert0       = rect_dist1 < self.wall_margin and rect_dist0 < self.wall_margin
    rect_vert1       = rect_dist1 < self.wall_margin and rect_dist2 < self.wall_margin
    rect_vert2       = rect_dist3 < self.wall_margin and rect_dist2 < self.wall_margin
    rect_vert3       = rect_dist3 < self.wall_margin and rect_dist0 < self.wall_margin
    rect_vert_ind    = np.max([rect_vert0, rect_vert1, rect_vert2, rect_vert3])
    if rect_vert_ind:
      self.rect_vertex = np.amax([rect_vert0, rect_vert1, rect_vert2, rect_vert3])
    else:
      self.rect_vertex = None

    # distance to circle
    circle_dist      = abs(distance_between(self.x, (self.xc, self.yc)) - self.radius)

    # evaluate wall
    self.wall_dist   = np.min([rect_dist, circle_dist])  
    self.wall        = np.argmin([rect_dist, circle_dist]) # rectangle or circle
    pass

  def domain(self):
    # restrict domain
    self.x_pred = np.clip(self.x_pred, self.xl, self.xr)
    self.y_pred = np.clip(self.y_pred, self.yu, self.yd)
    pass
    
  def scatter(self):
    # reset if exceed wall margin
    if self.wall_dist < self.wall_margin:
      self.P = []     
      for i in range(self.num_particles):
        hp = np.random.uniform(-self.range_head, self.range_head) # heading direction
        tp = np.random.uniform(0., self.range_trans) # translation
        rp = np.random.uniform(-self.range_turn, self.range_turn) # turning direction
        self.P.append((hp, tp, rp))
    pass

  def bounce(self):
    # input
    in_angle = angle_trunc(self.head)

    # rectangle
    if self.wall == 0 and self.rect_side != None:
      rect_normal = [0, pi / 2., pi, -pi / 2.] # normal vectors to query

      # physical model
      out_angle1  = angle_trunc(2 * rect_normal[self.rect_side] - pi - in_angle)

      # predicted bounce, bin size 5
      i = int(in_angle / (2 * pi) * 360) / 5 + 36      
      out_angle2 = degree_to_radians(self.pred_bounce[self.rect_side][i])

      # average of physical model and inference
      out_angle = (out_angle1 + out_angle2) / 2.

    # circle
    if self.wall == 1:
      if self.x[0] > self.xc:
        dst_pos = self.x
        src_pos = (self.xc, self.yc)
      else:
        dst_pos = (self.xc, self.yc)
        src_pos = self.x
      circle_normal = angle_trunc(get_heading(dst_pos, src_pos))
      out_angle     = angle_trunc(2 * circle_normal - pi - in_angle) # physical model

    # bounce if exceed wall margin or outside of domain
    if self.wall_dist < self.wall_margin:
      # if no previous bounce in margin or different side
      if not self.prev_bounce or self.bounce_side != self.rect_side \
      or self.x_pred == self.x[0] or self.y_pred == self.x[1]: # test
        self.prev_bounce = True
        self.bounce_side = self.rect_side
        for i, p in enumerate(self.P):  
          hp = np.random.normal(out_angle, self.sigma_head) # normal distribution with mean of out angle
          self.P[i] = (angle_trunc(hp), p[1], p[2])
    elif self.prev_bounce:
      self.prev_bounce = False
    pass

  def diffusion(self):
    # add gaussian noise
    for i in range(len(self.P)):
      hp, tp, rp = self.P[i]
      hp         = angle_trunc(hp + np.random.normal(0, self.sigma_head))
      tp         = max(int(tp + np.random.normal(0, self.sigma_trans)), 0)
      rp         = angle_trunc(rp + np.random.normal(0, self.sigma_turn))
      self.P[i]  = (hp, tp, rp)        
    pass

  def dynamics(self, x, h, t, r):
    # motion velocity model
    xm = x[0] + t * np.cos(angle_trunc(h + r))
    ym = x[1] + t * np.sin(angle_trunc(h + r))
    return xm, ym

  def measurement(self, x):
    # if no measurement available then use previous prediction
    if x == None: 
      x = (self.x_pred, self.y_pred)

    # calculate weights
    for i in range(len(self.P)):
      hp, tp, rp = self.P[i]
      xp, yp     = self.dynamics(self.x, hp, tp, rp)
      dist       = distance_between((xp, yp), (x[0], x[1]))
      w          = gaussian(dist, self.sigma) # probability
      self.W[i]  = w
        
    # normalize weights
    W_sum = sum(self.W)
    if W_sum == 0: # reset
      self.W = [1. / float(self.num_particles)] * self.num_particles
    else:
      self.W = [w / W_sum for w in self.W]

    # update measurement
    self.x = x

    # update new heading based on turn
    for i, p in enumerate(self.P):  
      self.P[i] = (angle_trunc(p[0] + p[2]), p[1], p[2]) 
    pass

  def sample(self):
    # sample based on distribution of weights
    I = np.random.choice(
                        range(self.num_particles)
                        ,size = self.num_particles
                        ,replace = True
                        ,p = self.W
                        )
    self.P = [self.P[i] for i in I]
    pass

  def predict(self, x):
    # update prediction
    H, T, R     = np.array(self.P)[:, 0], np.array(self.P)[:, 1], np.array(self.P)[:, 2]
    self.head   = np.average(H, weights = self.W)
    self.turn   = np.average(R, weights = self.W)
    if x != None: # if measurement available
      self.trans = np.average(T, weights = self.W)
    self.x_pred = int(self.x[0] + self.trans * np.cos(angle_trunc(self.head + self.turn)))
    self.y_pred = int(self.x[1] + self.trans * np.sin(angle_trunc(self.head + self.turn)))
    pass

  def process(self, x):
    # run iteration
    self.diffusion()
    self.measurement(x)
    self.sample()
    self.predict(x)
    self.domain()
    self.localize()
    if x != None: 
      self.scatter() # if measurement available
    else:
      self.bounce() # if measurement not available
    return (self.x_pred, self.y_pred)

def main():

  # read input
  in_text = open(sys.argv[1], "r")

  # create data
  data = []
  with in_text as f:
    for i, line in enumerate(f):
      x, y = line.split(",")
      data.append((int(x), int(y)))
  
  # initialize particle filter at t = 0
  pf = ParticleFilter(
                     x = data[0] # initial coordinates
                     ,num_particles = 500
                     ,range_trans   = 100
                     ,range_head    = np.pi
                     ,range_turn    = 0.5
                     ,sigma_head    = 0.05
                     ,sigma_trans   = 1.
                     ,sigma_turn    = 0.01
                     ,sigma         = 5.
                     # world
                     ,rect          = [(240, 105), (1696, 974)] # top left and bottom right vertices
                     ,circle        = [(1004, 539), 95] # center and radius
                     ,wall_margin   = 60
                     )
    
  # initialize prediction    
  x_pred = 0
  y_pred = 0
  pred   = []
  
  # run particle filter on input data starting at t = 1
  for i in range(1, len(data)): # input data
    try:
      x_pred, y_pred = pf.process(data[i]) # use measurement at t = i to predict t = i + 1
    except:
      print "i: ", i
      pass
      
  # predict next 60 frames
  n = 60
  for i in range(n):
    try:
      x_pred, y_pred = pf.process(None) # no measurements
    except:
      pass
    pred.append((x_pred, y_pred)) # store prediction

  # write output
  out_text = open("prediction.txt", "w")
  with out_text as f:
    for x, y in pred:
      f.write("%i,%i\n" % (int(x), int(y)))
  
if __name__ == "__main__":
  main()
