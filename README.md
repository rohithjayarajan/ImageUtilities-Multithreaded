# Image Manipulation Library

## Overview

Image Manipulation Library which currently includes multithreaded Rotation by Nearest Neighbor Interpolation:

- cmake
- googletest

## Standard install via command-line
```
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app <PATH_TO_INPUT_IMG_DESTINATION> <PATH_TO_OUTPUT_IMG_DESTINATION>
```

Note: When building and running for tests, comment out the displayImage function used in multithreadRotation() to display image and also comment out the imwrite function in multithreadRotation() as these two functions (cv::imshow and cv::imwrite) cause unexpected failing of 2 tests. 

## Building for code coverage
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser. 
