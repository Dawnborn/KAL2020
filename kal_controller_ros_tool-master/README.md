# KAL CONTROLLER ROS TOOL

Controller package for KAL pipeline.

This package provides lateral and longitudinal controller for vehicle in KAL in order to follow the input trajectory. 
The lateral controller is based on "Werling-Controller" which is one of the subjects taught in the lecture "Verhaltensgenerierung für  Fahrzeuge".

The longitudinal controller computes desired path based on the distance and time horizon of the input trajectory.

In the output, the desired speed and computed steering angle are used to generate [Ackeramann Drive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html) messages which will be sent for the carla controller.
