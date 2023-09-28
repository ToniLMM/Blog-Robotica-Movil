# Blog-Robotica-Movil
This will be Toni's Mobile Robotics blog, where the status of the practices, their news, progress, errors and solutions will be reported.

## Practice 1: Basic Vacuum Cleaner
During this practice I've been testing with sensors and actuators with simple values. From my point of view that's the best way to know how the vacuum react to different situations and scenarios. Other than that the first days I've been also testing with the spiral motion.

### First Algorithm

As I said before the first days were trial days where I became familiar with the platform, the parameters, the way to implement the movement and the strategy. As a consequence the first algorithm I made was simple and primitive, without states or laser. However was completely random and many times due to luck worked incredibly well. Here I show some examples of its good results
![WhatsApp Image 2023-09-28 at 20 10 53](https://github.com/ToniLMM/Blog-Robotica-Movil/assets/92941378/3e857e19-5b34-4cf2-8dcc-cc5f9a80c5cd)
![WhatsApp Image 2023-09-28 at 19 13 36](https://github.com/ToniLMM/Blog-Robotica-Movil/assets/92941378/8deacb79-b3a0-41e0-85a1-9fc94692636c)

### Final Algorithm

The second algorithm consists of a 4 state machine: 'FORWARD', 'BACK', 'TURN', 'SPIRAL'. This algorithm starts with a increasing spiral until the laser detects an obstacle. When the vacuum detects an obstacle goes back (state 'BACK') and immediately rotates with a random angle (state 'TURN'). After that the robot continues on its way in a straight line (state 'FORWARD') until it detects an obstacle again where the loop repeats indefinitely. However the spiral has a 5% of probability of being executed again.

### Observations

In many attempts I have had problems with 2 specific places where the robot got stuck because the space between the table leg and the sofa was exactly the same as the width of the vacuum cleaner. That's why the robot always got stuck when it entered these holes

![WhatsApp Image 2023-09-28 at 20 33 26](https://github.com/ToniLMM/Blog-Robotica-Movil/assets/92941378/439c2e96-85d5-4838-b62c-0bbaa4311e38)
