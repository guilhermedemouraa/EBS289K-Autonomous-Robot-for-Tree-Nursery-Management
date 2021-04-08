# EBS289K-Autonomous-Robot-for-Tree-Nursery-Management
Projects for the course EBS 289k: Sensors and Actuators in Agricultural Automation

![simulation](https://user-images.githubusercontent.com/39603677/114103867-b8e13400-987e-11eb-8772-7a22a133c107.gif)

## Project Description
Computer simulation of an agricultural robot inspecting a tree nursery. This project is part of the course EBS289K: Agricultural Robotics and Automation, thought at UC Davis by Prof. Stavros Vougioukas.
The challenge consisted in programming a mobile robot equipped with a GPS, a digital compass, and a 2D laser scanner to traverse the field block and count the number of trees in each row. In addition, the diameter of the trunk of each tree should be estimated, so that management could decide which trees are ready for the market.

![commercial_nursery_alternating_offering-greater-selection_640x426](https://user-images.githubusercontent.com/39603677/114094037-ad870c00-9870-11eb-8c6d-378ed0a3fb38.png)

Fig. 1 - Robot-compatible tree nursery. Source: Davey Nursery (https://www.davey.com/commercial-landscape-services/the-davey-nursery/).

The key steps for the success of this project included developing code for, path planning, robot localization, path following, and computer vision (to scan the environment, process digital images, create an occupancy grid, and classify each tree according to their diameter).

• Path planning was performed through a genetic algorithm to optomize route through orchard and create shortest dubins path. 

• Robot localization was performed using sensor fusion (GPS and Odometer) through an Extended Kalman Filter. In order to make the simulation more realistic, simulated noise data was added to both GPS and odometer sensors.

• Path following was performed performed through the Pure Pursuit algorithm (Coulter, R.C., 1992 - Implementation of the purse pursuit algorithm).

• Environment was characterized by an occupancy grid, which was generated from the data of a 2D laser scanner. In order to make the simulation more realistic, simulated noise data was added to the laser scanner. The noisy data was filtered with a low-pass filter.

### Simulation steps:

1. Generate random nursery

![Random nursery generate](https://user-images.githubusercontent.com/39603677/114100448-24280780-9879-11eb-8a3c-0f64d813e69a.png)

2. Plan path

![Path plan](https://user-images.githubusercontent.com/39603677/114100858-c6e08600-9879-11eb-935b-53494ac2deab.png)

3. Traverse the field and scan the environment with the 2D LiDAR

![traverse](https://user-images.githubusercontent.com/39603677/114101067-0e671200-987a-11eb-969a-bd941313440c.JPG)

4. Use low-pass filter to enhance LiDAR data

![lowpass](https://user-images.githubusercontent.com/39603677/114101183-38b8cf80-987a-11eb-8a3d-00fef9fec8eb.JPG)

5. Use image processing to locate trees and estimate their diameter

![part 2](https://user-images.githubusercontent.com/39603677/114101450-99e0a300-987a-11eb-95ca-9c816a5e01d4.JPG) ![final](https://user-images.githubusercontent.com/39603677/114101459-9c42fd00-987a-11eb-82a3-99535892bb05.JPG)

6. Data Analysis

a) Navigation error

![path error](https://user-images.githubusercontent.com/39603677/114101728-fe9bfd80-987a-11eb-9bcd-ab3d8e76530c.JPG)

b) Tree localization error

![tree location error](https://user-images.githubusercontent.com/39603677/114101764-0e1b4680-987b-11eb-95d2-1ce8dcd8d304.JPG)

c) Tree diameter estimate error

![tree diamater error](https://user-images.githubusercontent.com/39603677/114101806-1b383580-987b-11eb-9b16-dc6b1555475a.JPG)

### Files:
• FinalProjectMain.m: Author: Guilherme De Moura Araujo, Bennet Evans, and Nico Buxbaum. Description: Masterfile for project. Initializes parameters, calls all functions, performs C.V. algorithms and produces final output.

• generateNursery.m: Author: Stavros G. Vougioukas. Description: Creates a randomized simulated nursery for testing.

• tspof_ga.m: Author: Joseph Kirk. Description: Solves traveling salesman problem to compute optimal path through orchar using genetic algorithm.

• robotOdo.m: Author: Stavros G. Vougioukas. Description: Produces simulated noisy odometery data.

• GPS_CompassNoisy: Author: Stavros G. Vougioukas. Description: Produces simulated noisy GPS data.

• ekfODO: Author: Guilherme De Moura Araujo, Bennet Evans, and Nico Buxbaum. Description: Extended Kalman Filter implementation

• purePursuit.m: Author: Guilherme De Moura Araujo, Bennet Evans, and Nico Buxbaum. Description: Path tracking script, uses pure pursuit algorithm.

• bresenhamfast.m: Author:Peter I. Corke. Description: Performs Bresenham line algorithm (processes 2D LIDAR data).

• laserScannerNoisy.p: Author: Stavros Vougioukas. Description: Produces simulated noisy LIDAR (2D) data.

• laserRange.m: Author: Peter I. Corke and Stavros Vougioukas. Description: Takes noisy LIDAR data and performs range finding algorithm.

• XYtoIJ.m: Authot: Stavros G. Vougioukas. Description: Transforms coordinates from world to robot frame.

• updateLaserBeamGrid.m: Author: Guilherme De Moura Araujo, Bennet Evans, and Nico Buxbaum. Description: Updates nursery occupancy grid.

• updateLaserBeamBitmap.m: Author: Guilherme De Moura Araujo, Bennet Evans, and Nico Buxbaum. Description: Updates nursery bitmap.

• findTrees.m: Author: Guilherme De Moura Araujo, Bennet Evans, and Nico Buxbaum. Description: Image processing algorithm to scan an occupancy grid, find circular shapes, and 
 record circular shape center location (x,y) and diameter.

• draw_disc.m: Author: Stavros G. Vougioukas. Description: Draws simulated nursery.

• Image_Processing_Error.m: Author: Guilherme De Moura Araujo, Bennet Evans, and Nico Buxbaum. Description: Performs statistical analysis on output data.

• bycicle_model.m: Author: Guilherme De Moura Araujo, Bennet Evans, and Nico Buxbaum. Description: Performs euler integration to find final state of the robot.

• draw_tractor.m: Author: Guilherme De Moura Araujo, Bennet Evans, and Nico Buxbaum. Description: Plots tractor model (robot).

• move_robot.m: Author: Guilherme De Moura Araujo, Bennet Evans, and Nico Buxbaum. Description: Creates animated version of the robot moving.
