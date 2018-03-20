# FCND - 3D Motion Planning

### 1. Project Overview
Goal of this project is to integrate the techniques that we have learned to create a planning solution, Which plan and generate safe and smooth trajectories for a simulated drone through an urban environment in a simulator. 

##### 1.1 The 2.5D map of the urban environment is in colliders.csv
The colliders.csv file that we used is a 2.5D grid representation showing downtown San Francisco at roughly one meter resolution. We read the global home location from the first line of the colliders.csv file and set that position as global home (self.set_home_position()).  Starting from the third row is the obstacles data in the map, Position of each obstacle is represented by discrete X,Y,Z coordinates, Size of each obstacle is represented by halfSizeX,halfSizeY,halfSizeZ.

[2.5D map](https://... FIXME)

<!-- # Final Result
Video (https://youtu.be/xWD0j_8Z6gg  FIXME) With this path planner, our drone successfully plan a path through an urban environment and fly around the 2D flight path.

![pathplanner5](https://user-images.githubusercontent.com/24623272/29002135-5933af78-7ace-11e7-8e9a-8fee53692b5f.png FIXME)  -->


### 2. Project Rubric

#### 2.1. Explain the Starter Code

#### 2.2 Implementing of the Path Planning Algorithm 

##### Read and set the global home location

##### From global position to local position 

##### Adding flexibility to the start location

##### Adding flexibility to the goal location

##### A* algorithm
Write your search algorithm. Minimum requirement here is to add diagonal motions to the A* implementation provided, and assign them a cost of sqrt(2).

##### Prune path with collinearity


### 3. Executing the flight


