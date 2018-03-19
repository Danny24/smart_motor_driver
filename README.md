# Smart Motor Driver
This motor driver is designed to be interfaced from a host microcontroller by I2C and be able to control the motor with a PID autonomously.

![alt text](https://github.com/Danny24/smart_motor_driver/blob/master/Hardware/Renders/2.png)

This is a smart motor driver designed to run a Pololu micro gearmotor, the objective of this board is to be able to implement a PID control over this motors with the corresponding RPM feedback using a hall effect sensor. So it must have his own microcontroller and H bridge to be capable of do the work by itself.

The host microcontroller (or any system that can use I2C) will communicate by I2C in order to give the commands to the smart driver module, this includes speed and direction of the motor. The module will automatically implement the PID to maintain the speed and apply more/less power in the motor to archive that.

![alt text](https://github.com/Danny24/smart_motor_driver/blob/master/Hardware/Photos/1.jpg)

Also the control module will be capable of driving the motor for a desired distance and then stop when is reached. For doing that you must specify the diameter of your wheel and the gearbox relation.

![alt text](https://github.com/Danny24/smart_motor_driver/blob/master/Hardware/Renders/6.png)

This is a project under development, so please follow me get updates :) This will be an open source project, so if you have any ideas to improve it you are welcome! I am currently working on the firmware.

<iframe width="854" height="480" src="https://www.youtube.com/embed/lJUFUr4bWiI" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

Thank you for reading :)
