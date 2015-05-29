#!/bin/bash

### script for testing cmake build chain ###
#
# In order to check correctness of cmake macros defined in
# cmake/xacro-extras.cmake.em, this script checks for
# successfully running the cmake toolchain, i.e. cmake, make, make install.
# Providing different test directories as first argument, this script can check
# different CMakeLists.txt.
#
# Even better would be a unittest framework within cmake, e.g.
# http://stackoverflow.com/questions/29818725/cmake-how-to-unit-test-your-own-cmake-script-macros-functions

# Is argument $1 a valid cmake source directory?
test -d $1 || exit 2
test -r $1/CMakeLists.txt || exit 2

dir=`basename $1`

# redirect stdout and stderr to $dir.log
exec &> $dir.log 2>&1

# cleanup our build dir
rm -rf $dir
mkdir $dir
cd $dir

# load catkin environent
build_env=../../build_env.sh

echo "*** running cmake ***"
${build_env} cmake -DCATKIN_DEVEL_PREFIX=devel -DCMAKE_INSTALL_PREFIX=install $1 || exit $?

echo
echo "*** running make ***"
${build_env} make || exit $?

echo
echo "*** running make install ***"
${build_env} make || exit $?
