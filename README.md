**Overview**

![werqwer](https://github.com/user-attachments/assets/4d530957-efd4-42d7-a84a-a17bb3a2cb36)

This project features a line follower robot built with a custom-made 16-channel line sensor array and a custom controller board based on an Arduino Nano with a TB6612FNG motor driver.

The robot is capable of PID-controlled line following for smooth and precise movement and can navigate maze intersections using its sensor array and maze-cross logic.

**Features**

- PID Line Following: Smooth motor control to maintain accurate line tracking.

- Maze Navigation:

-  Detects intersections with left, center, or right lines.

-  Executes predefined maneuvers at intersections (turn left, turn right, or go straight).

- Motor Control: Uses TB6612FNG driver to control left and right motors.

**Hardware**

- Controller Board: Custom PCB with Arduino Nano and TB6612FNG motor driver

- Sensor Array: Custom 16-channel line sensor array with multiplexers

- Motors & Drive: DC motors controlled via TB6612FNG

- Power Supply: 12V LiPo Battery
