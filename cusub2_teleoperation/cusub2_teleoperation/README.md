# cusub2_teleop
Code to interface with motors on sub

## joyListener.py
- ROS2 listener, suscribes to joy topic and sends commands to motorController.py

## motorController.py
- Class to take commands and send these to motors through the mini maestro

## testMC.py
- file used to test motorController.py methods
