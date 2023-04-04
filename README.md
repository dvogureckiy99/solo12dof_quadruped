### This repository created for modeling Solo12 robot

Was tested with ubuntu 20.04, Gazebo Fortress, ROS2 Humble.
Library: Rviz

if you need recompile package
```
rm -rfv install/solo12dof_quadruped/ build/solo12dof_quadruped log
colcon build --symlink-install --packages-select solo12dof_quadruped
```
OR (cause there is one package)
```
rm -rfv install/ build/ log/
colcon build --symlink-install
```

How run world with Solo12 model ?
```
cd ~/git/Solo12/src/solo12dof_quadruped/worlds
ign gazebo solo12dof_quadruped world.sdf
```
