### Build the ORB-SLAM2-ROS by Dockerfile
Firstly, git clone the algorithm Dockerfile repository to your computer, then:
```
cd orb-slam2-ros-mono/
./install.sh
```
You can check whether the image is successfully built as follows:
```
docker images
```
you can see:
```
slam-hive-algorithm orb-slam2-ros-mono [IMAGE ID] [CREATED] [SIZE]
```
And ORB_SLAM2/Vocabulary/ORBvoc.txt are too large to push to github, so we zip it at same path, and you can just unzip it before use.

For source code, we just change the places where used to save trajectory and mapping result files.

