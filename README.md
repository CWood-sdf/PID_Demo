# PID_Demo

This is a very simple demo of a turning pid function. This is meant for rookie vex users to understand the basics of a pid and also learn how to tune it. 

## Note

The kP, kI, and kD constants are at 0, and thus the robot will not move when the program is run. My recommendation would be to start by increasing the kP until there is a little bit of a wobble at the end, then increase kD until there is no wobble, then increase kP, then kD...

I would not recommend using kI in this because there is nothing to prevent integral windup (AKA the integral value getting so big that the robot will just turn very fast). 

There are 3 primary ways of preventing integral windup: only allowing the integral to grow in a certain error range, zeroing the integral when the error is in a certain range, or preventing it from growing over a certain value. 
