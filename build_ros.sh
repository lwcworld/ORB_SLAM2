echo "Building ROS nodes"

./creatSwapfile.sh

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
