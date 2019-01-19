# TLC5940-control
A script written for the control of the TLC5940 (a PWM LED driver) with the arduino.

I wrote this because the existing library was broken by updates to the arduino environment. It was also an interesting academic exercise.

Documentation isn't great.  The important thing is to observe the pin mapping (lines 37-46) when wiring up the chips.  You'll also have to set some of the constants defined at the top of the file for your specific application.  From there, if you make sure you read and fully understand the TLC5940 datasheet inside out then you should be able to make some sense of my code.  But then at that point you might want to write your own code...
