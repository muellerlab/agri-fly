catkin build hiperlab_rostools hiperlab_hardware hiperlab_components hiperlab_common --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -D_ECLIPSE_VERSION=4.6 -DCMAKE_CXX_COMPILER_ARG1=-std=c++11 -DCMAKE_BUILD_TYPE=Release

ROOT=$PWD
cd build
for PROJECT in `find $PWD -name .project`; do
    DIR=`dirname $PROJECT`
    echo $DIR
    cd $DIR
    awk -f $(rospack find mk)/eclipse.awk .project > .project_with_env && mv .project_with_env .project
done
cd $ROOT
