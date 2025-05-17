## pure pursuit in rviz

### Terminal 1:
Go to the correct location:
```
cd /path/to/Team10_ws/

```
RUN:
```
ros2 run purepursuit_new pure_pursuit_node
```
### Terminal 2:
open RVIZ

RUN:
```
rviz2
```
add -> By topic:
```
1. /desired_path
    path

2. /trajectory
    path

3. /vehicle_marker
    Marker
```
