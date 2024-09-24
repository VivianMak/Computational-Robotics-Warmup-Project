# Computational-Robotics-Warmup-Project
This warmup project serves as an introduction to ROS2 and working with the Neatos with various tasks. The 6 behaviors we implemented for this project were: robot teleoperation, driving in a square, wall following, person following, obstacle avoidance, and finite-state control (which combines multiple behaviors). Our approach to implementing these behaviors was to complete the MVP in a way that allowed for more complicated integrations.

## Project Writeup
(see documenation -- insert pdf link int he repo)

## Behaviors Implemented
1. __Robot Teleoperation.__ - This behavior assigns different keys to movements. 
2. __Driving in a Square.__ - This behavior utilizes odometry to drive a square relative to its starting location.
3. __Wall Following.__ - This behavior drives the Neato to the parallel with a detected wall.
4. __Person Following.__ - This behavior allows you to walk in front of the Neato and it will follow your movements while maintaining a specified following distance.
5. __Obstacle Avoidance.__ - This behavior allows the Neato to traverse any environment without hitting any obstacles
6. __Finite-State Control.__ - This behavior combines drive square and person following. The Neato will always be trying to drive in a square until a person is detected, then the Neato will start the person_following behavior until the person is not detected. It will then start drawing a square again.