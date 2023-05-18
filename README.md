# 2D LIDAR simulation and SLAM Implementation using Python<br>
This repository contains an implementation of a line segment extraction algorithm that utilizes laser data and seeded region growing. The algorithm is designed to robustly detect line segments in noisy environments, enabling accurate perception and decision-making for robotic applications such as mapping, navigation, and object detection.<br>


**Line Segment Extraction Algorithm using Laser Data and Seeded Region Growing**<br>



Development of a 2D LIDAR sensor and SLAM (Simultaneous Localization and Mapping) algorithm to detect features from the sensor data. The sensor is simulated using the LaserSensors class, which has methods to calculate the distance between the sensor and an obstacle, and sense obstacles in the environment. The sensor adds uncertainty to the sensor data using the uncertainty_add function. <br>

The SLAM algorithm is implemented using the featuresDetection class which has several methods that process the sensor data and detect features in the environment. The methods include distance calculation between points and lines, line-fitting, and point projection. The class also has methods for converting between different line representation and for detecting and growing seed segments. <br>

The simulation is run using Pygame library, which is used to display the sensor data and detected features on the map of the environment. The simulation starts by initializing the environment, sensor, and feature detection classes. The sensor data is collected by moving a cursor on the screen, which simulates the movement of the robot. The sensor data is then processed by the SLAM algorithm, and features are detected and displayed on the map of the environment. <br>

The simulation was implemented using Pygame library which provides an interactive and visual way to display the sensor data and detected features. <br>

![feature](https://user-images.githubusercontent.com/85798077/213577668-76031d64-2ef1-4a42-8c25-2a74839d03f5.png)<br>


![ezgif com-optimize](https://user-images.githubusercontent.com/85798077/229351322-a78ef329-0bfb-45ca-a803-5bc1e7874be4.gif)<br><br>

**Features:**

- Accurately extracts line segments from laser data using the seeded region growing technique.<br>
- Handles noise and clutter in the environment to ensure reliable segment detection.<br>
- Utilizes the Orthogonal Distance Regression (ODR) method for line fitting.<br>
- Provides an easy-to-use Python interface for seamless integration into existing projects.<br>
- Includes an example script (example.py) that demonstrates the usage and showcases the algorithm's capabilities.<br>

**Installation:**<br>

To use this line segment extraction algorithm, follow these steps:<br>

1. Clone this repository to your local machine using the following command:<br>
   git clone https://github.com/shivam-gupta0/2D-LIDAR-simulation-and-SLAM-Implementation-using-python.git<br>

2. Install the required dependencies by running the following command:<br>
   pip install -r requirements.txt<br>

3. Ensure that you have a suitable environment for running Python code (e.g., Python 3.7+).<br>

**Usage:**<br>

1. Import the necessary classes and functions from the feature_main.py file into your project.<br>

2. Create an instance of the Environment class to set up the environment and map surface.<br>

3. Initialize the laser sensors using the LaserSensors class, providing the range parameter and map surface.<br>

4. In a loop or based on your application's requirements, use the featuresDetection class to detect line segments in the laser data.<br>

5. Retrieve the detected line segments from the LINE_SEGMENTS variable in the featuresDetection class.<br>

6. Update the map with the detected line segments using the provided show_sensorData function.<br>

7. Customize the algorithm's parameters and criteria as needed for your specific application.<br>

For a complete example, refer to the example.py file in this repository.<br>



**Future Enhancements:**<br>

- Implementing additional line fitting methods to provide flexibility for different applications.<br>
- Enhancing the algorithm's performance by optimizing the code or utilizing parallel processing.<br>
- Integrating with other sensor data to provide a more comprehensive perception system for robots.<br>

**Contributing:**<br>

Contributions to this line segment extraction algorithm are welcome! If you have any ideas, improvements, or bug fixes, please open an issue or submit a pull request.<br>

When contributing, please follow the existing code style, include appropriate tests and documentation, and adhere to the repository's license.<br>



**Acknowledgements:**<br>

- Refrence: https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245 <br>

**Contact:**
shivamgupta0@hotmail.com



