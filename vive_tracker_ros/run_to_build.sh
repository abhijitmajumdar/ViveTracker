echo "Building vive_tracker_ros_package"
BASEDIR=$(dirname "$BASH_SOURCE")
WORKDIR=`pwd`
if [ ! -d "${BASEDIR}/vive_tracker_ros_package/build" ]; then
  echo "Creating directory "${BASEDIR}"/vive_tracker_ros_package/build"
  mkdir ${BASEDIR}/vive_tracker_ros_package/build
fi
cd ${BASEDIR}/vive_tracker_ros_package/build
cmake .. -DROS_BUILD_TYPE=Release
make
echo "Build complete"
