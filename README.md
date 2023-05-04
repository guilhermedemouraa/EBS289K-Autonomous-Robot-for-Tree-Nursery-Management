# EBS289K-Autonomous-Robot-for-Tree-Nursery-Management
Projects for the course EBS 289K: Sensors and Actuators in Agricultural Automation (UC Davis)

![simulation](https://user-images.githubusercontent.com/39603677/114103867-b8e13400-987e-11eb-8772-7a22a133c107.gif)

## Disclaimer:
While every attempt has been made to accurately create a mobile robot simulation, the work has not been verified independently and some typographical and/or technical errors may be present due to the type of assumptions made. In any case, the authors (Guilherme De Moura Araujo, Bennett Evans, and Nicholas Buxbaum) hope that the present files will be a helpful guide and assist in making further improvements as necessary.

The project you will find in this repository is directly taken from course assignments with the instructor’s consent.

## Project Description
Computer simulation of an agricultural robot inspecting a tree nursery. This project is part of the course EBS289K: Agricultural Robotics and Automation, taught at UC Davis by Prof. Stavros Vougioukas.
The challenge consisted in programming a mobile robot equipped with a GPS, a digital compass, and a 2D laser scanner to traverse the field block and count the number of trees in each row. In addition, the diameter of the trunk of each tree should be estimated, so that management could decide which trees are ready for the market.

![commercial_nursery_alternating_offering-greater-selection_640x426](https://user-images.githubusercontent.com/39603677/114094037-ad870c00-9870-11eb-8c6d-378ed0a3fb38.png)

Fig. 1 - Robot-compatible tree nursery. Source: [Davey Nursery](https://www.davey.com/commercial-landscape-services/the-davey-nursery/).

The project's success relied on developing specific components, including Guidance, Navigation, and Control (GNC) for the robot, Simultaneous Localization and Mapping (SLAM) for environment perception, and computer vision algorithms for tree classification based on diameter.

• To ensure the robot moved along the optimal route through the orchard, we implemented a genetic algorithm for path planning that created the shortest Dubins path. This helped us save time and resources.

• In order to have a better understanding of the environment, we used a 2D laser scanner to generate an occupancy grid, which is basically a map of the orchard. This allowed us to classify each tree based on their diameter, which is important for orchard management.

• To make sure that the robot moved along the planned path, we used a technique called Pure Pursuit algorithm. It's like playing a game of follow-the-leader, but with the robot as the leader.

• For the robot to know where it was in the orchard, we used a combination of GPS and odometer sensors, which we fused using an Extended Kalman Filter. This made it possible for us to get an accurate location of the robot and helped us simulate the real world better.

• Since real-world conditions are not always perfect, we added simulated noise to the GPS, odometer, and laser scanner data. However, to make sure our results were reliable, we filtered out the noise using a low-pass filter.

### Simulation steps:

1. Generate random nursery

![Random nursery generate](https://user-images.githubusercontent.com/39603677/114100448-24280780-9879-11eb-8a3c-0f64d813e69a.png)

2. Plan the path using a genetic algorithm and the dubbins path

![Path plan](https://user-images.githubusercontent.com/39603677/114100858-c6e08600-9879-11eb-935b-53494ac2deab.png)

3. Traverse the field and scan the environment with the 2D LiDAR (SLAM)

![traverse](https://user-images.githubusercontent.com/39603677/114101067-0e671200-987a-11eb-969a-bd941313440c.JPG)

4. Use low-pass filter to enhance LiDAR data (filter out noise)

![lowpass](https://user-images.githubusercontent.com/39603677/114101183-38b8cf80-987a-11eb-8a3d-00fef9fec8eb.JPG)

5. Use image processing techniques such as morphological operations to enhance the orchard map (enhanced image), locate trees and estimate their diameter using the Hough Transform

![part 2](https://user-images.githubusercontent.com/39603677/114101450-99e0a300-987a-11eb-95ca-9c816a5e01d4.JPG) 

6. Run the simulation

![final](https://user-images.githubusercontent.com/39603677/114101459-9c42fd00-987a-11eb-82a3-99535892bb05.JPG)

7. Data Analysis

a) Navigation error

![path error](https://user-images.githubusercontent.com/39603677/114101728-fe9bfd80-987a-11eb-9bcd-ab3d8e76530c.JPG)

b) Tree localization error

![tree location error](https://user-images.githubusercontent.com/39603677/114101764-0e1b4680-987b-11eb-95d2-1ce8dcd8d304.JPG)

c) Tree diameter estimate error

![tree diamater error](https://user-images.githubusercontent.com/39603677/114101806-1b383580-987b-11eb-9b16-dc6b1555475a.JPG)

### Files:
• FinalProjectMain.m: Author: Guilherme De Moura Araujo, Bennett Evans, and Nico Buxbaum. Description: Masterfile for project. Initializes parameters, calls all functions, performs C.V. algorithms and produces final output.

• generateNursery.m: Author: Stavros G. Vougioukas. Description: Creates a randomized simulated nursery for testing.

• tspof_ga.m: Author: Joseph Kirk. Description: Solves traveling salesman problem to compute optimal path through orchar using genetic algorithm.

• robotOdo.m: Author: Stavros G. Vougioukas. Description: Produces simulated noisy odometery data.

• GPS_CompassNoisy: Author: Stavros G. Vougioukas. Description: Produces simulated noisy GPS data.

• ekfODO: Author: Guilherme De Moura Araujo, Bennett Evans, and Nico Buxbaum. Description: Extended Kalman Filter implementation

• purePursuit.m: Author: Guilherme De Moura Araujo, Bennett Evans, and Nico Buxbaum. Description: Path tracking script, uses pure pursuit algorithm.

• bresenhamfast.m: Author:Peter I. Corke. Description: Performs Bresenham line algorithm (processes 2D LIDAR data).

• laserScannerNoisy.p: Author: Stavros Vougioukas. Description: Produces simulated noisy LIDAR (2D) data.

• laserRange.m: Author: Peter I. Corke and Stavros Vougioukas. Description: Takes noisy LIDAR data and performs range finding algorithm.

• XYtoIJ.m: Authot: Stavros G. Vougioukas. Description: Transforms coordinates from world to robot frame.

• updateLaserBeamGrid.m: Author: Guilherme De Moura Araujo, Bennett Evans, and Nico Buxbaum. Description: Updates nursery occupancy grid.

• updateLaserBeamBitmap.m: Author: Guilherme De Moura Araujo, Bennett Evans, and Nico Buxbaum. Description: Updates nursery bitmap.

• findTrees.m: Author: Guilherme De Moura Araujo, Bennett Evans, and Nico Buxbaum. Description: Image processing algorithm to scan an occupancy grid, find circular shapes, and 
 record circular shape center location (x,y) and diameter.

• draw_disc.m: Author: Stavros G. Vougioukas. Description: Draws simulated nursery.

• Image_Processing_Error.m: Author: Guilherme De Moura Araujo, Bennett Evans, and Nico Buxbaum. Description: Performs statistical analysis on output data.

• bycicle_model.m: Author: Guilherme De Moura Araujo, Bennett Evans, and Nico Buxbaum. Description: Performs euler integration to find final state of the robot.

• draw_tractor.m: Author: Guilherme De Moura Araujo, Bennett Evans, and Nico Buxbaum. Description: Plots tractor model (robot).

• move_robot.m: Author: Guilherme De Moura Araujo, Bennett Evans, and Nico Buxbaum. Description: Creates animated version of the robot moving.
