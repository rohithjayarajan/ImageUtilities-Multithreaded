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
 *  @file    ImageManipulateTest.cpp
 *  @author  rohithjayarajan
 *  @date 06/18/2019
 *  @version 1.0
 *
 *  @brief Tests for class ImageManipulation
 *
 *  @section DESCRIPTION
 *
 *  Tests functions of class ImageManipulation
 *
 */

#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include <vector>
#include "ImageManipulate.hpp"
// OpenCV header
#include "opencv2/opencv.hpp"

/**
 *   @brief test function to test rotateImage function
 *
 *
 *   @param ImgManipTests, the gtest framework
 *   @param rotateImageTest, the name of test
 */
TEST(ImgManipTests, rotateImageTest) {
    ImageManipulate testObject1;
    int x_ = 5;
    int y_ = 5;
    cv::Mat testImage_in = cv::Mat(x_, y_, CV_8UC3, cv::Scalar(0, 0, 0));
    testImage_in.at<cv::Vec3b>(cv::Point(2, 1)) = cv::Vec3b(100, 100, 100);
    cv::Mat testImage_out = cv::Mat(x_, y_, CV_8UC3, cv::Scalar(0, 0, 0));
    testObject1.setImgDims(x_, y_);
    testObject1.setInputImage(testImage_in);
    testObject1.setTheta_m(3.14);
    testObject1.rotateImage(testObject1.nearestNeighbor, testImage_out, 0, x_);
    cv::Vec3b color = testImage_out.at<cv::Vec3b>(cv::Point(1, 2));
    ASSERT_NEAR(100.0, color.val[0], 0.001);
    ASSERT_NEAR(100.0, color.val[1], 0.001);
    ASSERT_NEAR(100.0, color.val[2], 0.001);
}
/**
 *   @brief test function to test multithreadRotation function
 *
 *
 *   @param ImgManipTests, the gtest framework
 *   @param multithreadRotationTest, the name of test
 */
TEST(ImgManipTests, multithreadRotationTest) {
    ImageManipulate testObject2;
    cv::Mat testImage_ = cv::Mat(3, 3, CV_8UC1, cv::Scalar(0));
    testObject2.setInputImage(testImage_);
    ASSERT_EQ(testObject2.multithreadRotation(), true);
}
/**
 *   @brief test function to test inputImage_m setter and getter of class
 * ImageManipulation
 *
 *
 *   @param ImgManipTests, the gtest framework
 *   @param getSetInputImage, the name of test
 */
TEST(ImgManipTests, getSetInputImage) {
    ImageManipulate testObject3;
    cv::Mat inputImage_ = cv::Mat(3, 3, CV_8UC1, cv::Scalar(0));
    testObject3.setInputImage(inputImage_);
    cv::Mat setImage_ = testObject3.getInputImage();
    ASSERT_NEAR(0.0, setImage_.at<double>(0, 0), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(0, 1), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(0, 2), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(1, 0), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(1, 1), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(1, 2), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(2, 0), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(2, 1), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(2, 2), 0.001);
}
/**
 *   @brief test function to test outputImage_m setter and getter of class
 * ImageManipulation
 *
 *
 *   @param ImgManipTests, the gtest framework
 *   @param getSetOutputImage, the name of test
 */
TEST(ImgManipTests, getSetOutputImage) {
    ImageManipulate testObject4;
    cv::Mat outputImage_ = cv::Mat(3, 3, CV_8UC1, cv::Scalar(0));
    testObject4.setOutputImage(outputImage_);
    cv::Mat setImage_ = testObject4.getOutputImage();
    ASSERT_NEAR(0.0, setImage_.at<double>(0, 0), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(0, 1), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(0, 2), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(1, 0), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(1, 1), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(1, 2), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(2, 0), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(2, 1), 0.001);
    ASSERT_NEAR(0.0, setImage_.at<double>(2, 2), 0.001);
}
/**
 *   @brief test function to test outputImagePath_m setter and getter of class
 * ImageManipulation
 *
 *
 *   @param ImgManipTests, the gtest framework
 *   @param getSetOutputImagePath, the name of test
 */
TEST(ImgManipTests, getSetOutputImagePath) {
    ImageManipulate testObject5;
    std::string outputImagePath_ = "RandomPath";
    testObject5.setOutputImagePath(outputImagePath_);
    ASSERT_EQ(testObject5.getOutputImagePath(), outputImagePath_);
}
/**
 *   @brief test function to test theta_m setter and getter of class
 * ImageManipulation
 *
 *
 *   @param ImgManipTests, the gtest framework
 *   @param getSetetTheta_m, the name of test
 */
TEST(ImgManipTests, getSetetTheta_m) {
    ImageManipulate testObject6;
    double theta_ = 3.256;
    testObject6.setTheta_m(theta_);
    ASSERT_EQ(testObject6.getTheta_m(), theta_);
}
/**
 *   @brief test function to test imgDims setter and getter of class
 * ImageManipulation
 *
 *
 *   @param ImgManipTests, the gtest framework
 *   @param getSetImgDims, the name of test
 */
TEST(ImgManipTests, getSetImgDims) {
    ImageManipulate testObject7;
    int x_ = 640;
    int y_ = 480;
    testObject7.setImgDims(x_, y_);
    point setVal = testObject7.getImgDims();
    ASSERT_EQ(setVal.x, x_);
    ASSERT_EQ(setVal.y, y_);
}
/**
 *   @brief test function to test displayImage function
 *
 *
 *   @param ImgManipTests, the gtest framework
 *   @param displayImageTest, the name of test
 */
TEST(ImgManipTests, displayImageTest) {
    ImageManipulate testObject8;
    cv::Mat testImage_ = cv::Mat(500, 500, CV_8UC1, cv::Scalar(0));
    std::string windowName_ = "test window <HIT ESCAPE KEY>";
    ASSERT_EQ(testObject8.displayImage(windowName_, testImage_), true);
}
/**
 *   @brief test function to test nearestNeighbor function
 *
 *
 *   @param ImgManipTests, the gtest framework
 *   @param nearestNeighborTest, the name of test
 */
TEST(ImgManipTests, nearestNeighborTest) {
    ImageManipulate testObject9;
    double x_ = 562.345;
    double y_ = 365.787;
    point output_ = testObject9.nearestNeighbor(x_, y_);
    ASSERT_EQ(output_.x, static_cast<int>(x_));
    ASSERT_EQ(output_.y, static_cast<int>(y_));
}
