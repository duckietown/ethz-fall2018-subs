# Description

The first folder contains videos of control that where made all the time with :

omega_sat = 2.0
time_delay = 0.0
sampling_rate = 1.0


The robot was never used on internal curves as they were occupied, and I could actually have a pretty good controller for the external curves. But as soon as I tried on the internal curves, I understood that the robot needs to turn faster on internal turns. 

So I looked on how to change the omega_sat value (because the changing of values from the command line in the first version of the exercise didn't work), and once I found it, I put it to 5.0, then to 4.0 which was enough for internal turn (no need to put more then). 

This is why the second folder contains the last tuning of the PID with omega_sat at 4.0.

The last folder contains the experiment with sampling rate at 0.5. 

I was not able to try the delay, as the code would not launch when putting a value other than 0.0...


### In conclusion, my controller is the following:

omega_sat = 4

error function = delta (6 r + 1.5 phi)

kp = 4
ki = 0.05
kd = 0.02

