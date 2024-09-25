# Computational-Robotics-Warmup-Project
This warmup project serves as an introduction to ROS2 and working with the Neatos with various tasks. The 6 behaviors we implemented for this project were: robot teleoperation, driving in a square, wall following, person following, obstacle avoidance, and finite-state control (which combines multiple behaviors). Our approach to implementing these behaviors was to complete the MVP in a way that allowed for more complicated integrations.

## Project Writeup
[Warmup Project - Writeup](./Warmup%20Project%20-%20Writeup.pdf)


## Behaviors Implemented
1. __Robot Teleoperation.__ - This behavior assigns different keys to movements. 
2. __Driving in a Square.__ - This behavior utilizes odometry to drive a square relative to its starting location.
3. __Wall Following.__ - This behavior drives the Neato to the parallel with a detected wall.
4. __Person Following.__ - This behavior allows you to walk in front of the Neato and it will follow your movements while maintaining a specified following distance.
5. __Obstacle Avoidance.__ - This behavior allows the Neato to traverse any environment without hitting any obstacles
6. __Finite-State Control.__ - This behavior combines drive square and person following. The Neato will always be trying to drive in a square until a person is detected, then the Neato will start the person_following behavior until the person is not detected. It will then start drawing a square again.

## Conclusion
This project provided a good scaffold for future projects and introduction to the class with the practice of using ROS2 commands, writing good code, logic thinking, integration, and documentation. 

### Takeaways
Our key takeaway was that breaking down these complicated behaviors into smaller steps makes implementing the code much easier. A big challenge when starting this project was figuring out where to start, having no previous experience in ROS2. For each behavior, thinking through what the publisher and subscriber is narrowed down what exactly we needed to implement. From there, we used variables assigned to the topics, calculated how to find what we were looking for, wrote pseudocode, and calibrated as needed. This learned step-by-step process made approaching more complex tasks like person following more manageable.

Another takeaway was how to actually manage a project without knowing too many details about it. This is a skill that comes up a lot when approaching group projects at Olin because our motto is learn by doing. Before starting the project, our first meeting sorted out all the details of timeline, task list, scheduling work blocks, and setting up for parallelizing documentation.  We used a Notion workspace to document all assigned tasks, timeline, creating a sample write up, and saving often used command line calls. This process became very helpful also for breaking down the projects into more manageable sections. With this, we were able to see clear steps and milestones, and delegate tasks evenly among us.

### Code Structure
A class was created for each behavior we implemented inherited from Node. Each class had a publisher and subscriber topic depending on what the behavior needed. For example, for wall_follower, the publisher is Laserscan and the subscriber is Twist (commands velocity). The publisher will send data to the subscriber, and with our written logic, the robot is able to move with constraints. Within the class, there is a minimum of one function to process data from the publisher and a run_loop function which actually commands the Neato. This function will run indefinitely and is where the logic happens.

### Challenges
There were quite a few minor challenges we ran into along the project. In terms of the setup, we each were only able to connect to either the Neato or Gazebo. This made it difficult to test asynchronously. For technical challenges, figuring out the math for wall_following took a while, but we eventually figured it out. There were more individual behavior related challenges, but overall, we learned a lot through each obstacle and the limitations of what each topic could provide.

### Next Steps
This project was very open ended in the sense that there are many ways to improve the software. PID control is something we wanted to apply to all behaviors. Currently the robot jitters as it makes a decision, so having that implemented with a calibrated tolerance will solve that issue. Fixing up the software to work around the limitations of each topic is something we want to do. For example, for drive_square, if the heading or position changed from initiation, the square’s length won’t exactly be 1m. Know that we know the small details like that for each behavior, developing software for more complex behaviors will be easier. 
