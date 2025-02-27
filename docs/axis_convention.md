# Axis Conventions #

## NED Axis Convention ##

Maritime vehicles (and aircraft) use the North-East-Down (NED) axis convention. In this convention +X is forward, +Y is left, and +Z is down. When looking forward, +Roll is clockwise, +Pitch is upward, and +Yaw is right.

![Diagram describing NED axis convention for maritime vehicles](https://upload.wikimedia.org/wikipedia/commons/8/83/RPY_angles_of_ships.png)

![Diagram describing NED convention for rotations](https://upload.wikimedia.org/wikipedia/commons/c/c1/Yaw_Axis_Corrected.svg)

## Implementation Philosophy ##

Not all sensors will report readings in the appropriate axis convention. To maintain the convention throughout the code base, data should be converted as soon as possible. Raw readings should only be accessed by the code responsible for converting and exposing the corrected data. For maintainability, it is important that the raw readings are only converted in one place.
