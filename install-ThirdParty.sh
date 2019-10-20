#!/bin/sh
set -ex
cd ../
mkdir ThirdParty
mkdir ThirdParty/eigen3
cd -

wget  http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2
tar -xjf 3.3.7.tar.bz2
mv eigen-eigen-323c052e1731/Eigen ../ThirdParty/eigen3/.

git clone https://github.com/lava/matplotlib-cpp.git
mv matplotlib-cpp ../ThirdParty
