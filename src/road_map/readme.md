# Terminal 1:
```
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/yonatan/Desktop/Team10_ws/free_area_map.yaml
```
change the path to the correct one.

# Terminal 2:
```
ros2 lifecycle set map_server configure
```

after getting this output:
```
yonatan@yonatan:~$ ros2 lifecycle set map_server configure
Transitioning successful
```
run thi command:
```
ros2 lifecycle set map_server activate
```
you should get this output:
```
yonatan@yonatan:~$ ros2 lifecycle set map_server activate
Transitioning successful
```
# Terminal 3:
open rviz:
```
rviz2
```
In rviz adding map.