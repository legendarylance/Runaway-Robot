# Runaway-Robot

**Final Project<br />
CS 8803 AI for Robotics – Fall 2015<br />
Georgia Institute of Technology**

For this project, 60-second video clips of a robot moving in a wooden box was provided ten. Each video was recorded at 30 frames per second. The goal was to predict the position of the robot as accurately as possible for each of the following 60 frames (2 seconds) after the end of each video. The predictions were compared to the actual video data after the end of the video provided. In other words, the full videos were 62 seconds long, but only the first 60 seconds of each were provided, and predictions were compared to the last 2 seconds. <br />
About 20 minutes of “training” videos were provided, along with corresponding centroid data which were used to write the program.

## Report introduction
This report provides a thorough overview of the algorithm used to predict the position of the robot for 60 frames (2 seconds) after the end of each test case. The project was split into three main phases.<br />
First, the training data was used to infer a map of the world.<br />
Second, an ensemble learning (EL) algorithm was implemented to combine the predictions of Kalman Filter (KF) and k Nearest Neighbor (kNN).<br />
Third, the bouncing dynamics of the robot were modeled when it interacted with the world. This allowed to predict the outgoing angle given an incoming angle.<br />
The purpose of the report is to demonstrate that the implementation of the algorithm is reasonably sophisticated for a graduate level CS course. Moreover, compelling justification for the algorithm selection is provided.<br />
For complete report, see <a href="report.pdf">report.pdf</a>

## Screenshots:
<img src="images/1.PNG" width="700"><br />
Mapof the World (left) with Background Elimination (right)

<img src="images/2.PNG" width="700"><br />
Visualization of Training Data with 2-dimensional Histogram

<img src="images/3.PNG" width="700"><br />
Example EL Output # 1

<img src="images/4.PNG" width="700"><br />
Heading and Turning Angle Convention

<img src="images/5.PNG" width="700"><br />
High (red) and Low (blue) Turn Angles

<img src="images/6.PNG" width="700"><br />
Collision Calculation Diagram

<img src="images/7.PNG" width="700"><br />
Inward Angles vs. Bounce-off Angles (With Fitting Lines) 


## Execution Details
- Main File - finalproject.py<br />
- Training Data text file is reuired<br />
- training_data.txt is required for k-nearest neighbors algorithm. It must be in the same directory as finalproject.py file.


## Misc Folder
Misc folder contains different approaches that were designed during the development process, but were not incorporated in the Fnal Project<br />
- InwardBounceOffAngles.py file was used to create a database of Inward and Bounce Off angles, which was later used in Particle Filter approach <br />
- PF.py is our approach using particle filters. It performed well, but was computationally expensive and took more than 60 seconds to execute on Virtual Machine <br />
- LWR.py is our approach using Locally Weighted Regressions to account for non-linearity of Hexbug's trajectory

## Teammates
Johnson Kuan, Hai Dang, Ivan Fernandez
