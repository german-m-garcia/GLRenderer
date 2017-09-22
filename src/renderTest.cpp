#include "glRenderer.h"
#include "glm.h"
#include "cvCamera.h"
#include "markerDetector.h"
#include "timer.h"

int main(int argc, char **argv)
{
	// offline calibrated camera parameters
	float fxy = 832.560809f;
	float cx = 319.5f;
	float cy = 239.5f;
	float distortions[5] = { 0.066338f, -1.51067f, 0.f, 0.f, 7.626152f };
	Camera cam(fxy, fxy, cx, cy, distortions);


	// load mesh model
	GLMmodel *bmdl = glmReadOBJ("../data/bunny.obj");
	glmFacetNormals(bmdl);
	glmVertexNormals(bmdl, 90.0f);
	

	int frameWidth = 640;
	int frameHeight = 480;

	// initialize a renderer
	float nearPlane = 1.0f, farPlane = 1000.0f;
	GLRenderer renderer;
	renderer.init(argc, argv, frameWidth, frameHeight, nearPlane, farPlane, cam, bmdl);


	cv::Mat frame, frameDrawing, depth32, depth8;

	frame= cv::Mat::zeros(480, 640, CV_8UC3);
	depth8 = cv::Mat::zeros(frame.size(), CV_8UC1);
	frameDrawing = cv::Mat::zeros(frame.size(), CV_8UC3);


	cv::Mat markerTrans = cv::Mat::eye(4,4,CV_32FC1);
	markerTrans.at<float>(0,3) = 3.;
	markerTrans.at<float>(1,3) = 2.;
	markerTrans.at<float>(2,3) = 10.;
		

	renderer.camera.setExtrinsic(markerTrans);
	renderer.bgImg = frameDrawing;
	renderer.bgImgUsed = true;
	renderer.render();
	frameDrawing = renderer.bgrImg;
	depth32 = renderer.depthMap;
	cv::normalize(depth32, depth8, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	cv::imshow("Show Marker", frameDrawing);
	//cv::imshow("d", depth8);
	cv::waitKey(0);

	glmDelete(bmdl);
	return 0;
}
