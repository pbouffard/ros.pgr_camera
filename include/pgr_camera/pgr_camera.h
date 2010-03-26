#ifndef PGRCAMERA_H
#define PGRCAMERA_H

#include "flycapture/FlyCapture2.h"
#include <ros/ros.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <string>

using namespace std;

namespace pgr_camera {

void init();
size_t numCameras();

class Camera {
public:
	Camera();
	Camera(unsigned int serNo);

	void initCam();
	void start();
	void stop();
//	void frameDone(FlyCapture2::Image *frame, void *pCallbackData);
	void setFrameCallback(boost::function<void (FlyCapture2::Image*)> callback);
	void SetVideoModeAndFramerate(unsigned int width, unsigned int height, string format, unsigned int rate);

	// FIXME: following should really be private, but I can't see how to make the compiler
	// happy if they are..
	boost::function<void (FlyCapture2::Image*)> userCallback_;
	boost::mutex frameMutex_;

private:
	int camIndex;
	unsigned int camSerNo;
	FlyCapture2::Camera camPGR;
	FlyCapture2::Image rawPGRImage;
	FlyCapture2::FrameRate frameRate;
	bool setup();

};

} // namespace pgrcamera

#endif // PGRCAMERA_H
