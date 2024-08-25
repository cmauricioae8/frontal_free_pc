
# frontal_free_pc

ToDo:
+ Finish the README.md (explained in detail)


Package with the 'frontal_free_pc' functionality using node and node component.


## Information source
- ROS 2 composition tutorial: https://www.youtube.com/watch?v=PD0VJYBkkfQ


## Build the package

After cloning or creating any ROS 2 package, the colcon ws must be built:

```bash
cd ~/colcon_ws
colcon build --symlink-istall
source install/setup.bash
```


This builds the ros2 workspace with all packages.

To build only this package, use:

```bash
cd ~/colcon_ws
colcon build --packages-select ros2_basic_cpp --symlink-istall
source install/setup.bash
```

**IMPORTANT:**

+ This builds the package and sets a symbolic link to the python files (nodes and launch files). With this, re-building every time that a python file is modified, is not required.
+ In ROS 2, launch files may be written in yaml, xml or python languages, but it is extremely recommended to use python. Also, the name of all launch files must finish with 'launch.py'. Otherwise, the file will not be recognized.
+ The last command, `source ~/colcon_ws/install/setup.bash`, it is required every time that a new terminal is opened. To avoid this, add it to the .bashrc file with:

`echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc`

+ **Furthermore**, every time that a new file or folder is created in the workspace or for the first built, the source command must be executed. This is required to update the changes in the colcon_ws. Otherwise, the file will not be found.




