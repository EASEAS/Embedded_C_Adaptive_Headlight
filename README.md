# Embedded_C_Adaptive_Headlight

Code for a mini project on an adaptive headlight sub-system. Light
intensity and beam direction were controlled by the driver to help
illuminate areas relevant to the driver.

The input consister of a steering angle from a steering angle sensor,
and a vehicle speed snesor. The output is a pwm which controls a servo
and pwm for LED.

There are 3 modes for this subsystem: Class C, Class V and Class E. 

Class V has an inward and outward angle maximum of 8 degrees and the 
lowest brightness

Class C has an inward maximum of 10 degree and an outward maximum of 
20 degrees. This has the second brightest setting for the headlight.

Class E has an inward maximum of 10 degree and an outward maximum of 
20 degrees. This is the brightest setting for the headlight.
