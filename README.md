# Blog-Robotica-Movil
This will be Toni's Mobile Robotics blog, where the status of the practices, their news, progress, errors and solutions will be reported.

# Index

* [Index][Ind]
* [Practice 1][p1]
* [Practice 2][p2]

[Ind]: https://github.com/ToniLMM/Blog-Robotica-Movil/blob/main/README.md#index
[p1]: https://github.com/ToniLMM/Blog-Robotica-Movil/blob/main/README.md#practice-1-basic-vacuum-cleaner
[p2]: https://github.com/ToniLMM/Blog-Robotica-Movil/edit/main/README.md#practice-2-follow-line
 
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

### Other circuits

As I have put in the code, the other circuits work with this code but the parameters of the Vmax and the linear speed in the curves must be changed because these circuits have very tight curves that at high speed cannot be detected in time by the car.








