/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef PGRCAMERA_H
#define PGRCAMERA_H

#include "flycapture/FlyCapture2.h"
#include <ros/ros.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <string>

using namespace std;

namespace pgr_camera
{

  void init ();
  size_t numCameras ();

  class Camera
  {
  public:
    Camera ();
    Camera (unsigned int serNo);

    void initCam ();
    void start ();
    void stop ();
//      void frameDone(FlyCapture2::Image *frame, void *pCallbackData);
    void setFrameCallback (boost::function < void (FlyCapture2::Image *) > callback);
    void SetVideoModeAndFramerate (unsigned int width, unsigned int height, string format, double rate);
    void SetExposure (bool _auto, bool onoff, unsigned int value = 50);
    void SetGain (bool _auto, float value = 0.0);
    void SetShutter (bool _auto, float value = 0.015);

    // FIXME: following should really be private, but I can't see how to make the compiler
    // happy if they are..
      boost::function < void (FlyCapture2::Image *) > userCallback_;
      boost::mutex frameMutex_;

  private:
    int camIndex;
    unsigned int camSerNo;
      FlyCapture2::Camera camPGR;
      FlyCapture2::Image rawPGRImage;
      FlyCapture2::FrameRate frameRate;
    bool setup ();

  };

}                               // namespace pgrcamera

#endif                          // PGRCAMERA_H
