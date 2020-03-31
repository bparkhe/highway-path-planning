
## Highway Driving Project

The goals/steps of this project are the following:

* Implement a pipeline to guide a vehicle driving on a highway
* Generate trajectories that are within legal and safety thresholds 
* Plan behaviors appropriate to the circumstances in the lane

#### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  
[Rubric](https://review.udacity.com/#!/rubrics/1971/view)

---

### Writeup / README

#### 1. Valid Trajectories
##### The car is able to drive at least 4.32 miles without incident

The car is able to handle the different obstructions arising in this pipeline and generate trajectories that are collision-free for 4.32 miles.

##### The car drives according to the speed limit
The max speed is set to 47 to avoid the excess speeds on the lane switching trajectories.

##### Max Acceleration and Jerk are not Exceeded.
The speed up and slow down are set to a constant of 1.5mph per 0.02seconds which is under the safe travel thresholds for the vehicle. This constant also makes sure that the excess acceleration on the turns is also within thresholds.

##### Car does not have collisions
The car slows down to the speed of the vehicle ahead of it if it detects that it is within 30 meters. The slow down is a constant of 1.5mph per 0.02 seconds to avoid jerk.
A PD controller was implemented initially to arrive at the speed of the vehicle ahead of it. However, it is fairly taxing to select its parameters to avoid jerk.
The car does not have collisions with adjacent lane vehicles and is marked safe to switch lanes only if +30/-20 meters is open in that lane.

##### The car stays in the lane except for the time of changing lanes
The spline generation feature takes care of generating trajectories that are smoothly connected from the end path points to the one 30,60 and 90 meters ahead of the ego vehicle. In some cases it comes close to the lane lines and to avoid that, we can introduce a few more points between them.


##### The car is able to change lanes
The car continuously monitors adjacent lanes for vehicles and marks them if they are safe to switch. If there is a vehicle ahead in its current lane, it will go through the vector 'safetoswitch' to check for open lanes and set it as the new lane. Additionally, it will set switching lanes to 'true' so that the pipeline for vehicle ahead algorithm is not triggered prematurely. Once the vehicle reaches the threshold of +/-1 in the destination lane, the flag is set to false.

#### 2. Reflection
Although this pipeline has accounted for incidences on this road, it is a very rigid model and a much more generic one with cost functions can be implemented. The spline library is very effective in generating trajectories and to make them jerk minimizing, the division of the generated spline can be done based on equal acceleration rather than equal distance.

#### 3. Future implementation

Although for the purpose of the project I implemented a simple model-based algorithm, there can be a much more general model that I aim to program using the cost functions and jerk minimizing-polynomial trajectories.
Additionally, a behavior planner can be implemented to plan much more in advance to avoid getting stuck behind a vehicle and beside a vehicle in the adjacent lane.
