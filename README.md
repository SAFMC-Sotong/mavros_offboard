### To build
```
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
chmod +x install_geographiclib_datasets.sh
sudo install_geographiclib_datasets.sh
colcon build --symlink-install
```

### To launch

```
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" # if usinig make px4_sitl gz_x500
ros2 launch mavros_offb mavros_offb.launch.py
```


