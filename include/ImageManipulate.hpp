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
 *  @file    ImageManipulate.hpp
 *  @author  rohithjayarajan
 *  @date 06/18/2019
 *  @version 1.0
 *
 *  @brief Image Rotation
 *
 *  @section DESCRIPTION
 *
 *  Class declaration for Image Rotation. Includes rotation by Nearest Neighbor
 * Interpolation
 *
 */
#ifndef INCLUDE_IMAGEMANIPULATE_HPP_
#define INCLUDE_IMAGEMANIPULATE_HPP_
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
// BOOST header
#include <boost/range/irange.hpp>
// OpenCV header
#include "opencv2/opencv.hpp"

/**
 * @brief the definition of point structure
 *
 * Definition of the members of point structure
 */
struct point {
    int x;  // An integer value indicating x coordinate of the point
    int y;  // An integer value indicating y coordinate of the point
};

/**
 * @brief the declaration of ImageManipulate class
 *
 * Declaration of the variable and methods of ImageManipulate class
 */
class ImageManipulate {
   private:
    cv::Mat inputImage_m;   // cv::Mat data structure holding the input image
    cv::Mat outputImage_m;  // cv::Mat data structure holding the output image
    // std::string value of the output image path
    std::string outputImagePath_m;
    // A double value indicating the rotation angle in radians.
    // The rotation angle is defined as
    // ○ positive value : counter clock wise rotation
    // ○ negative value : clock wise rotation
    double theta_m;
    // A point structure holding the dimensions of the input image
    point imgDims;

   public:
    /**
     *   @brief Default constructor for ImageManipulate
     *
     *   @param nothing
     *   @return nothing
     */
    ImageManipulate();
    /**
     *   @brief Constructor for ImageManipulate
     *
     *   @param std::string value of input image path
     *   @param std::string value of output image path
     *   @param double value of angle to rotate by (in radian)
     *   @return nothing
     */
    explicit ImageManipulate(std::string inputImageName_,
                             std::string outputImageName_,
                             double rotationInRad_);
    /**
     *   @brief Default destructor for ImageManipulate
     *
     *   @param nothing
     *   @return nothing
     */
    ~ImageManipulate();
    /**
     *   @brief helper function to display image
     *
     *   @param std::string& value of the name of image to be displayed on
     *          window
     *   @param cv::Mat& value of image to be displayed
     *   @return nothing
     */
    bool displayImage(const std::string& imageName_,
                      const cv::Mat& image_) const;
    /**
     *   @brief function to perform nearest neighbor interpolation for given
     *          point
     *
     *   @param double value of x coordinate
     *   @param double value of y coordinate
     *   @return point structure of interpolated coordinate
     */
    static point nearestNeighbor(const double& x_, const double& y_);
    /**
     *   @brief function to rotate image
     *
     *   @param function pointer of type of interpolation
     *   @param output image
     *   @param beginning row
     *   @param ending row
     *   @return nothing
     */
    bool rotateImage(point (*f)(const double&, const double&),
                     cv::Mat& outputImage_, int rb, int re);
    /**
     *   @brief function to multithread image rotation
     *
     *   @param nothing
     *   @return cv::Mat value of image data
     */
    bool multithreadRotation();
    /**
     *   @brief function to get input image
     *
     *   @param nothing
     *   @return cv::Mat value of image data
     */
    cv::Mat getInputImage();
    /**
     *   @brief function to get output image
     *
     *   @param nothing
     *   @return cv::Mat value of image data
     */
    cv::Mat getOutputImage();
    /**
     *   @brief function to get output image path
     *
     *   @param nothing
     *   @return std::string value of output image path
     */
    std::string getOutputImagePath();
    /**
     *   @brief function to get rotation angle in radians
     *
     *   @param nothing
     *   @return double value of rotation angle in radians
     */
    double getTheta_m();
    /**
     *   @brief function to get image dimensions
     *
     *   @param nothing
     *   @return point structure of image dimensions
     */
    point getImgDims();
    /**
     *   @brief function to set input image
     *
     *   @param cv::Mat value of image data
     *   @return nothing
     */
    void setInputImage(const cv::Mat& inputImage_);
    /**
     *   @brief function to set output image
     *
     *   @param cv::Mat value of image data
     *   @return nothing
     */
    void setOutputImage(const cv::Mat& outputImage_);
    /**
     *   @brief function to set output image path
     *
     *   @param std::string value of output image path
     *   @return nothing
     */
    void setOutputImagePath(std::string outputImagePath_);
    /**
     *   @brief function to set rotation angle in radians
     *
     *   @param double value of rotation angle in radians
     *   @return nothing
     */
    void setTheta_m(double theta_);
    /**
     *   @brief function to set image dimensions
     *
     *   @param double value of x coordinate
     *   @param double value of y coordinate
     *   @return nothing
     */
    void setImgDims(const int& x_, const int& y_);
};

#endif  // INCLUDE_IMAGEMANIPULATE_HPP_
