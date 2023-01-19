# 2D-LIDAR-simulation-and-SLAM
Refrence: https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245 \
**A line segment extraction algorithm
using laser data based on seeded
region growing** 

Development of a 2D LIDAR sensor and SLAM (Simultaneous Localization and Mapping) algorithm to detect features from the sensor data. The sensor is simulated using the LaserSensors class, which has methods to calculate the distance between the sensor and an obstacle, and sense obstacles in the environment. The sensor adds uncertainty to the sensor data using the uncertainty_add function. 

The SLAM algorithm is implemented using the featuresDetection class which has several methods that process the sensor data and detect features in the environment. The methods include distance calculation between points and lines, line-fitting, and point projection. The class also has methods for converting between different line representation and for detecting and growing seed segments. 

The simulation is run using Pygame library, which is used to display the sensor data and detected features on the map of the environment. The simulation starts by initializing the environment, sensor, and feature detection classes. The sensor data is collected by moving a cursor on the screen, which simulates the movement of the robot. The sensor data is then processed by the SLAM algorithm, and features are detected and displayed on the map of the environment. 

The simulation was implemented using Pygame library which provides an interactive and visual way to display the sensor data and detected features. 

![feature](https://user-images.githubusercontent.com/85798077/213577668-76031d64-2ef1-4a42-8c25-2a74839d03f5.png)
![f2](https://user-images.githubusercontent.com/85798077/213577712-76c75900-9f7e-4a42-af24-973d3041faa2.png)
![s](https://user-images.githubusercontent.com/85798077/213577733-e3cbb8f7-96d0-4334-bf00-e21f2caf2d6c.png)




 MATH IMPLEMENTED IN FEATUREDETECTION.PY FROM **A line segment extraction algorithm
using laser data based on seeded
region growing** RESEARCH PAPER

dist_point2point(self, point1, point2) : This method calculates the Euclidean distance between two points represented as tuples (x1, y1) and (x2, y2). The distance is calculated using the Pythagorean theorem which states that the distance between two points (x1, y1) and (x2, y2) is the square root of (x1-x2)^2 + (y1-y2)^2. 

dist_point2line(self, params, point) : This method calculates the distance between a point represented as a tuple (x, y) and a line represented by the parameters A, B, C. The distance is calculated using the equation |Ax + By + C| / sqrt(A^2 + B^2) 

line_2points(self, m, b): This method returns the two points of a line represented by the slope-intercept form equation y = mx + b. The two points are calculated by assuming x=5 and x=2000 and solving for y. 

lineForm_G2SI(self, A, B, C): This method converts a line represented in the general form (Ax + By + C = 0) to slope-intercept form (y = mx + b). The slope is calculated by -A/B and the y-intercept is calculated by -C/B 

lineForm_Si2G(self, m, B): This method converts a line represented in the slope-intercept form (y = mx + b) to the general form (Ax + By + C = 0). A, B, C are calculated by -m, 1, -b respectively. 

line_intersect_general(self, params1, params2): This method calculates the point of intersection of two lines represented by the general form parameters (A1, B1, C1) and (A2, B2, C2). The point of intersection is calculated using the equation (c1b2 - b1c2)/(b1a2-a1b2) and (a1c2-a2c1)/(b1a2-a1b2) for x and y respectively. 

points_2line(self, point1, point2): This method converts two points represented as tuples (x1, y1) and (x2, y2) to the slope-intercept form of a line (y = mx + b). The slope is calculated by (y2-y1)/(x2-x1) and the y-intercept is calculated by y2 - m*x2 

projection_point2line(self, point, m, b): This method calculates the projection of a point represented by a tuple (x, y) on a line represented by the slope-intercept form (y = mx + b). The intersection point is calculated by first calculating the perpendicular line passing through the point with slope = -1/m and y-intercept = y - m2*x, then calculating the intersection point of the perpendicular line


