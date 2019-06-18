# Image Manipulation Library

## Overview

Image Manipulation Library which currently includes multithreaded Rotation by Nearest Neighbor Interpolation:

- cmake
- googletest

## Standard install via command-line
```
git clone --recursive https://github.com/rohith/ImageManipulationLibrary
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```

## Building for code coverage
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.