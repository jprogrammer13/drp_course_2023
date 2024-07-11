# REPO GUIDE

# Configure the Docker image
## Build the Docker image

```bash
cd docker 
./build.sh
```
# Run the simulation

```bash
./run_docker.sh
```

Inside the Docker container

```bash
cd  ros_ws/
rosdep update
rosdep install --from-paths src --ignore-src -r -y 
catkin_make
source devel/setup.bash
```

Commit after the installation of dep
