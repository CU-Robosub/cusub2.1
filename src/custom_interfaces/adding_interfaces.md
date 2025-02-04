# Creating Custom Interfaces #

Create the .msg (message) or .srv (service) file in the appropriate directory using this format:
[ROS Interfaces](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html)

After defining the interface, update the `CMakeList.txt` by adding a new entry to the generate_interfaces call:

``` cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MyService.srv"
  "msg/MyMessage.msg"
)
```

Include the interfaces by importing them in your code:

``` python
from custom_interfaces.srv import MyService
```

And be sure these dependencies are added to the package's `package.xml`

``` xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
