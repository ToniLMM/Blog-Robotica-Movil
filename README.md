# Blog-Robotica-Movil
This will be Toni's Mobile Robotics blog, where the status of the practices, their news, progress, errors and solutions will be reported.

# Index

* [Index][Ind]
* [Practice 1][p1]
* [Practice 2][p2]
* [Practice 3][p3]
* [Practice 4][p4]

[Ind]: https://github.com/ToniLMM/Blog-Robotica-Movil/blob/main/README.md#index
[p1]: https://github.com/ToniLMM/Blog-Robotica-Movil/blob/main/README.md#practice-1-basic-vacuum-cleaner
[p2]: https://github.com/ToniLMM/Blog-Robotica-Movil/blob/main/README.md#practice-2-follow-line
[p3]: https://github.com/ToniLMM/Blog-Robotica-Movil/blob/main/README.md#practice-3-obstacle-avoidance
[p4]: https://github.com/ToniLMM/Blog-Robotica-Movil/blob/main/README.md#practice-4-global-navigation
 
## Practice 1: Basic Vacuum Cleaner
During this practice I've been testing with sensors and actuators with simple values. From my point of view that's the best way to know how the vacuum react to different situations and scenarios. Other than that the first days I've been also testing with the spiral motion.

### First Algorithm

As I said before the first days were trial days where I became familiar with the platform, the parameters, the way to implement the movement and the strategy. As a consequence the first algorithm I made was simple and primitive, without states or laser. However was completely random and many times due to luck worked incredibly well. Here I show some examples of its good results
![WhatsApp Image 2023-09-28 at 20 10 53](https://github.com/ToniLMM/Blog-Robotica-Movil/assets/92941378/3e857e19-5b34-4cf2-8dcc-cc5f9a80c5cd)
![WhatsApp Image 2023-09-28 at 19 13 36](https://github.com/ToniLMM/Blog-Robotica-Movil/assets/92941378/8deacb79-b3a0-41e0-85a1-9fc94692636c)

### Final Algorithm

The second algorithm consists of a 4 state machine: 'FORWARD', 'BACK', 'TURN', 'SPIRAL'. This algorithm starts with a increasing spiral until the laser detects an obstacle. When the vacuum detects an obstacle goes back (state 'BACK') and immediately rotates with a random angle (state 'TURN'). After that the robot continues on its way in a straight line (state 'FORWARD') until it detects an obstacle again where the loop repeats indefinitely. In summary, it is a reactive code as requested in practice.


#### (SPIRAL) ---> BACK ---> TURN ---> FORWARD ---> BACK ---> TURN ---> FORWARD.............  

This is the state 'SPIRAL':
```python
if current_state == RobotState.SPIRAL:
       
        HAL.setV(V)
        HAL.setW(W)

        if (time.time() - time_start) > rotation_period:
            time_start = time.time()
            V -= 0.5
            rotation_period = 2 * PI / abs(W)  # Recalculate rotation period
            collision_start_time = time.time()
            current_state = RobotState.BACK
```

This is the state 'BACK':
```python
if current_state == RobotState.BACK:
      HAL.setV(-MAX_SPEED)
      HAL.setW(0)

      if (time.time() - collision_start_time) > BACK_DURATION:
          collision_start_time = time.time()
          V = 1
          W = V * PI
          current_state = RobotState.TURN
```

This is the state 'TURN':
```python
elif current_state == RobotState.TURN:
        angle = random.uniform(MIN_ANGLE, MAX_ANGLE)
        HAL.setV(0)
        HAL.setW(angle)

        if (time.time() - collision_start_time) > TURN_DURATION:
                  collision_start_time = time.time()
                  print("FORWARD")
                  current_state = RobotState.FORWARD
```

This is the state 'FORWARD':
```python
elif current_state == RobotState.FORWARD:
        print("FORWARD")
        HAL.setV(MAX_SPEED)
        HAL.setW(0)  # Stop rotating

        if laser_dist_mesurement == False:
          current_state = RobotState.BACK
```
This is the implementation of the laser:
```python
def parse_laser_data(laser_data):
    laser = []
    for i in range(180):
        dist = laser_data.values[i]
        angle = math.radians(i)
        laser += [(dist, angle)]
    return laser

def laser_mesurement(laser): 
    for i in range(60):
      if laser[60+i][0] < MIN_DIST:
        return False
    return True
```

Here is a video of the algorithm working

[Screencast from 28-09-23 23:34:44.webm](https://github.com/ToniLMM/Blog-Robotica-Movil/assets/92941378/83e5691b-c9a3-4262-b55e-d074c8ca5757)







### Observations

In many attempts I have had problems with 2 specific places where the robot got stuck because the space between the table leg and the sofa was exactly the same as the width of the vacuum cleaner. That's why the robot always got stuck when it entered these holes

![WhatsApp Image 2023-09-28 at 20 33 26](https://github.com/ToniLMM/Blog-Robotica-Movil/assets/92941378/439c2e96-85d5-4838-b62c-0bbaa4311e38)


## Practice 2: Follow line
This second practice consists of a Formula 1 car that must follow the center of the red line and complete the circuit as quickly as possible. The goal of this exercise is to perform a PID reactive control capable of following the line painted on the racing circuit

### First days

During the first days I was testing the movements of the car and how it reacted to the curves of the red line. I also tried different color masks to familiarize myself with the environment. At first I started with a black and white mask that was later replaced by the current red mask. Finally, I implemented some basic PIDs to test at small speeds how the car behaved on different circuits.

### Final version

This final version is a mix of speed and staying on top of the line as much as possible. However, speed is prioritized a little more than going above the line, because minimum times are required in this practice. Even if speed is prioritized, line tracking continues at all times. If it is necessary to be on top of the line all time, the maximum speed can be reduced and the PIDs adjusted, in this way the car can correct its trajectory more efficiently and effectively and make smoother movements.

#### Performance and structure

This final version is structures with 3 functions: 'image_filtering', 'speed' and 'is_curve'.

The 'image_filtering' function applies a red mask to distinguish the line, in addition provides a reduced image for greater efficiency of the PIDs and a centroid that marks the center of the red line

This is the red mask applied in the function:
```python
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    mask = cv2.add(mask0, mask1)
```


This is the line of code that allows the image to be reduced because the sky does not provide any useful information in this practice:
```python
reduced_image = image[220:460, 0:image.shape[1]]
```

The 'speed' function is responsible for controlling the speed and direction of the vehicle, adjusting the linear and angular speed depending on whether it is in a curve or on a straight section of the circuit, using a PID controller for the angular speed.

Here you can see what the car does depending on whether it is a curve or a straight line and the PID controller used for the angular velocity:
```python
    if is_curv:
        KP, KI, KD = KP_curve, KI_curve, KD_curve
        V = Vmax * 0.6 
    else:
        KP, KI, KD = KP_straight, KI_straight, KD_straight
        V += time_increment * 0.7 
        V = min(V, Vmax)

    W = KP * error + KD * (error - prev_error) + KI * sum_error / i
```

The 'is_curve' function evaluates whether an object in an image lies on a curve based on the difference in the object's position on the x-axis compared to a reference value and compares this error to a predefined threshold.

This is the 'is_curve' function:
```python
def is_curve(image, error_threshold=0.15):
    center = image_filtering(image)
    object_x = center[0]
    error_x = abs(329 - object_x) / 480
    is_curved = error_x > error_threshold
    return is_curved
```

Here is the video of the final version working:

https://github.com/ToniLMM/Blog-Robotica-Movil/assets/92941378/664ed3fa-b93b-407c-9565-3c090de6a309


### Observations

In the previous video the time is 114 seconds, however a factor to take into account is the workload of the computer because time can vary. My best time was 107 seconds but depending on the performance of the computer it may vary. Nevertheless the average time of this version is around 110 seconds.


For me the biggest difficulty of this practice has been finding suitable PIDs for the speeds because changing a simple value makes the car oscillate a lot and deviate very easily. Although it now continues to have oscillations, they are much smaller, in addition, it runs the circuit in less than 2 minutes, which is an acceptable time.

### Other circuits

As I have put in the code, the other circuits work with this code but the parameters of the Vmax and the linear speed in the curves must be changed because these circuits have very tight curves that at high speed cannot be detected in time by the car.

## Practice 3: Obstacle avoidance
The objective of this third practice is to implement the logic of the VFF navigation algorithm. This algorithm consists of each object generating a repulsive force towards the robot, while the target generates an attractive force. The main objective is for the car to complete a lap of the circuit, however more levels of difficulty can be added such as: doing it as fast as possible, choosing the safest path...

### First days

During the first days I've been testing with different functions and ways to see how the robot behaved in different situations and obstacles. At first, when the robot moved, the laser detected the car but the force vectors were not well configured, therefore it ran over the first car and continued in a straight line. Another problem that arose for me was taking targets since there were some under obstacles and therefore the car collided with them regardless of the repulsive force vector. An additional problem was that when the weights of the forces were equal, in the first car, a local minimum was created where the car remained static and did not move because the forces canceled each other. Finally, another of the big problems was calculating the weight of the values ​​of the vectors that had to be put on them so that the car did not collide, because if these values ​​were not adjusted correctly, the car would have strange behaviors and would collide on many occasions with obstacles.

### Final version

The final version of the VFF navigation algorithm consists of a series of functions that allow us to avoid obstacles with forces thanks to laser data, know the current position of the car and know the position of the current target.

This is the 'absolute2relative' function that allow us to convert absolute coordinates to relative ones:
```python3
    dx = x_abs - robotx
    dy = y_abs - roboty
    x_rel = dx * math.cos(-robott) - dy * math.sin(-robott)
    y_rel = dx * math.sin(-robott) + dy * math.cos(-robott)
    return x_rel, y_rel
```

'parse_laser_data' takes data from the laser and converts it into a list of pairs (distance, angle) in radians. This is used to calculate the repulsive forces.
On the other hand 'laser_vector' takes a list of pairs (distance, angle) and converts it to a list of pairs (x, y) in the robot's reference frame.
```python3
def parse_laser_data(laser_data):
    laser = [(dist / 1000.0, math.radians(i)) for i, dist in enumerate(laser_data.values)]
    return laser

def laser_vector(laser):
    return [(d * math.cos(a - math.pi/2) * -1, d * math.sin(a - math.pi/2) * -1) for d, a in laser]
```

Here's how the attractive force is calculated:
```python3
    att_module = math.hypot(x_rel, y_rel)
    att_phase = math.atan2(y_rel, x_rel)
    if att_module > 2:
        att_module = 2
    return att_module * math.cos(att_phase), att_module * math.sin(att_phase)
```

In contrast this is how the repulsive force is calculated:
```python3
    sumX, sumY = 0, 0
    for d, a in laser:
        x = (1 / d) * math.cos(a - math.pi/2) * -1
        y = (1 / d) * math.sin(a - math.pi/2) * -1
        sumX += x
        sumY += y
    return sumX, sumY
```

This function gets the position of the current target, marks the target as reached, and returns the (x,y) coordinates of the target:
```python3
def get_current_target_position():
    current_target = GUI.map.getNextTarget()
    current_target.setReached(True)
    
    return current_target.getPose().x, current_target.getPose().y
```

Finally this function updates the forces acting on the robot (attractive and repulsive force) and then sets the speed of the robot
```python3
def update_forces_and_velocity(att_force, rep_force, target_position):
    GUI.map.targetx, GUI.map.targety = target_position
    GUI.map.carx, GUI.map.cary = att_force
    GUI.map.obsx, GUI.map.obsy = rep_force
    res_vector = (ALPHA * att_force[0] + BETA * rep_force[0], ALPHA * att_force[1] + BETA * rep_force[1])
    GUI.map.avgx, GUI.map.avgy = res_vector
    HAL.setV(res_vector[0])
    HAL.setW(res_vector[1])
```



https://github.com/ToniLMM/Blog-Robotica-Movil/assets/92941378/5d4989a2-7d2a-42fb-95d2-8dc9c31ed6e7


## Practice 4: Global Navigation

The objective of this practice is to implement the logic of a Gradient Path Planning (GPP) algorithm. Selected a destination, the GPP algorithm is responsible for finding the shortest path to it, avoiding, in the case of this practice, everything that is not road. Once the path has been selected, the logic necessary to follow this path and reach the objective must be implemented in the robot. With this, it is possible for the robot to go to the marked destination autonomously and following the shortest path.


### First days

### Final version

[Screencast from 04-12-23 23:02:02.webm](https://github.com/ToniLMM/Blog-Robotica-Movil/assets/92941378/fd7a32eb-1d68-4ec7-be96-d5d39e7b1b38)





