# Documentation for Maestro 
This was a massive pain for us so we've tried to document it. Code can be seen in `/cusub2_teleop/motorController.py`

## What is the Maestro?
The pololu mini maestro is a multi-channel servo controller. This device is able to send PWM signals sperately to each of these channels. There is a GUI for windows which allows you to do this visually, see [this link.](https://www.pololu.com/docs/0J40/3.a)

## How does it work?
There are two things to note about the Maestro:
- The Maestro works in quarter-microseconds, so this is why you see the 4x multiplier on PWM values
- You must manually convert the desired PWM signal to byte-level code. This is why you see shifts and hex values in the code. The documentation for this process can be seen [here.](https://www.pololu.com/docs/0J40/5.c)

## How to update/maintain
The function that sends commands to the Maestro is `motorController.run(self, channels, target, duration=-1)`. This function takes in an array of channels (or just one) and sends a given command to these channel(s). The mapping from the channel on the maestro to the motor number can be seen in the picture below. The code accesses these motors through arrays of seemingly arbitrary indices, so this is quite helpful. It is best to ensure that nobody changes the order in which the motors are plugged in on the maestro.
![IMG_0607](https://github.com/user-attachments/assets/9c323e78-4d39-477e-9599-3f3b5e5681d4)



