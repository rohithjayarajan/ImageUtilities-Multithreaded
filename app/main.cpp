/******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (C) 2019, Rohith Jayarajan
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the
 * names of its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 *  Copyright 2019 rohithjayarajan
 *  @file    main.cpp
 *  @author  rohithjayarajan
 *  @date 06/18/2019
 *  @version 1.0
 *
 *  @brief Image Manipulation
 *
 *  @section DESCRIPTION
 *
 *  Image Manipulation Library. Currently includes image rotation by Nearest
 * Neighbor Interpolation
 *
 */

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include "ImageManipulate.hpp"

int main(int argc, char *argv[]) {
    std::clock_t c_start = std::clock();
    auto t_start = std::chrono::high_resolution_clock::now();
    // check if correct number of arguments are provided
    if (argc == 4) {
        // create an object of ImageManipulate class
        ImageManipulate rotater(argv[1], argv[2], atof(argv[3]));
        // rotate the image using nearest neighbor interpolation technique
        rotater.multithreadRotation();
        std::cout << "Rotated image " << argv[1] << " successfully."
                  << std::endl;
    } else {
        std::cerr << "Possible Missing Argument(s)" << std::endl;
        std::cout << "[Program input expected: ./app/shell-app "
                     "$PATH_TO_INPUT_IMAGE$ $PATH_TO_OUTPUT_IMAGE$ "
                     "$ROTATION_ANGLE_IN_RADIAN$]"
                  << std::endl;
    }
    std::clock_t c_end = std::clock();
    auto t_end = std::chrono::high_resolution_clock::now();

    std::cout
        << std::fixed << std::setprecision(2)
        << "CPU time used: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC
        << " ms\n"
        << "Wall clock time passed: "
        << std::chrono::duration<double, std::milli>(t_end - t_start).count()
        << " ms\n";
    return 0;
}
