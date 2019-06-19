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
 *  @file    ImageManipulate.cpp
 *  @author  rohithjayarajan
 *  @date 06/18/2019
 *  @version 1.0
 *
 *  @brief Image Rotation
 *
 *  @section DESCRIPTION
 *
 *  Class definition for Image Rotation. Includes rotation by Nearest Neighbor
 * Interpolation
 *
 */
// user defined header
#include "ImageManipulate.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
// BOOST header
#include <boost/range/irange.hpp>
// OpenCV header
#include "opencv2/opencv.hpp"

ImageManipulate::ImageManipulate() {}

ImageManipulate::ImageManipulate(std::string inputImageName_,
                                 std::string outputImageName_,
                                 double rotationInRad_) {
    // set value of member variable indicating output path of image
    outputImagePath_m = outputImageName_;
    // set value of member variable indicating input image
    inputImage_m = cv::imread(inputImageName_, 1);
    // Check for invalid input
    if (!inputImage_m.data) {
        std::cerr << "Could not open or find the image" << std::endl;
    }
    // set value of member variable indicating rotation angle
    theta_m = rotationInRad_;
    // set value of member variable indicating image dimensions
    imgDims.x = inputImage_m.rows;
    imgDims.y = inputImage_m.cols;
    // create empty output image initialized with all ones(white image)
    outputImage_m =
        cv::Mat(imgDims.x, imgDims.y, CV_8UC3, cv::Scalar(255, 255, 255));
}

ImageManipulate::~ImageManipulate() {}

bool ImageManipulate::displayImage(const std::string& imageName_,
                                   const cv::Mat& image_) const {
    // show image
    cv::imshow(imageName_, image_);
    // wait for key press
    cv::waitKey(0);
    // destroy window
    cv::destroyWindow(imageName_);
    return true;
}

point ImageManipulate::nearestNeighbor(const double& x_, const double& y_) {
    point position_nn;  // point structure to hold output of nearest neighbor
                        // interpolation
    // perform nearest neighbor interpolation by converting value to int
    position_nn.x = static_cast<int>(x_);
    position_nn.y = static_cast<int>(y_);
    // return pixel coordinates calculated by nearest neighbor interpolation
    return position_nn;
}

bool ImageManipulate::rotateImage(point (*f)(const double&, const double&),
                                  cv::Mat& outputImage_, int rb, int re) {
    std::string displayString = "Input Image";  // std::string value to hold
                                                // window name for image display
    // display input image
    // displayImage(displayString, inputImage_m);
    // point structure holding image center
    point imgCenter = {static_cast<int>((imgDims.x / 2)),
                       static_cast<int>((imgDims.y / 2))};
    // for all pixel location, perform rotation about image center
    // given by the below matrix operation
    // [x'] = |cos(theta)  sin(theta)| * |x - x_c|  + [x_c]
    // [y']   |-sin(theta) cos(theta)|   |y - y_c|  + [y_c]
    for (int r : boost::irange(static_cast<int>(rb), static_cast<int>(re))) {
        for (int c : boost::irange(0, static_cast<int>(imgDims.y))) {
            double x_new = static_cast<int>(
                (std::cos(theta_m) * (r - imgCenter.x)) +
                (std::sin(theta_m) * (c - imgCenter.y)) + imgCenter.x);
            double y_new = static_cast<int>(
                -(std::sin(theta_m) * (r - imgCenter.x)) +
                (std::cos(theta_m) * (c - imgCenter.y)) + imgCenter.y);
            point position = (*f)(x_new, y_new);
            // if pixel location is valid, copy pixel information to output
            // image
            if (r >= 0 && r < imgDims.x && c >= 0 && c < imgDims.y &&
                position.x >= 0 && position.x < imgDims.x && position.y >= 0 &&
                position.y < imgDims.y) {
                cv::Vec3b color = inputImage_m.at<cv::Vec3b>(
                    cv::Point(position.y, position.x));
                outputImage_.at<cv::Vec3b>(cv::Point(c, r)) = color;
            }
        }
    }
    return true;
}

bool ImageManipulate::multithreadRotation() {
    // get the total number of threads available in the system
    unsigned int nThreads = std::thread::hardware_concurrency();
    // declare a vector of threads
    std::vector<std::thread> vecThreads;

    int rb, re;  // decalare variables to store beginning and end row
    // assign task to each thread
    for (int n : boost::irange(0, static_cast<int>(nThreads - 1))) {
        rb = std::ceil(n * imgDims.x / nThreads);
        re = std::floor((n + 1) * imgDims.x / nThreads);
        std::thread t1(&ImageManipulate::rotateImage, this, nearestNeighbor,
                       std::ref(outputImage_m), rb, re);
        vecThreads.push_back(std::move(t1));
    }
    // assign task to process thread
    rb = std::ceil((nThreads - 1) * imgDims.x / nThreads);
    re = imgDims.x;
    rotateImage(nearestNeighbor, outputImage_m, rb, re);
    for (auto& th : vecThreads) {
        if (th.joinable()) {
            th.join();
        }
    }

    // display output image
    std::string displayString = "Output Image";
    displayImage(displayString, outputImage_m);
    // write output image in user defined path
    cv::imwrite(outputImagePath_m, outputImage_m);
    return true;
}

cv::Mat ImageManipulate::getInputImage() { return inputImage_m; }

cv::Mat ImageManipulate::getOutputImage() { return outputImage_m; }

std::string ImageManipulate::getOutputImagePath() { return outputImagePath_m; }

double ImageManipulate::getTheta_m() { return theta_m; }

point ImageManipulate::getImgDims() { return imgDims; }

void ImageManipulate::setInputImage(const cv::Mat& inputImage_) {
    inputImage_m = inputImage_;
}

void ImageManipulate::setOutputImage(const cv::Mat& outputImage_) {
    outputImage_m = outputImage_;
}

void ImageManipulate::setOutputImagePath(std::string outputImagePath_) {
    outputImagePath_m = outputImagePath_;
}

void ImageManipulate::setTheta_m(double theta_) { theta_m = theta_; }

void ImageManipulate::setImgDims(const int& x_, const int& y_) {
    imgDims = {x_, y_};
}
