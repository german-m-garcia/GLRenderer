#include "GLRenderer/glRenderer.h"
#include "GLRenderer/glm.h"
#include "GLRenderer/cvCamera.h"
#include "GLRenderer/markerDetector.h"
#include "GLRenderer/timer.h"
#include "GLRenderer/renderLib.h"

int main(int argc, char **argv){

	// offline calibrated camera parameters
	float fxy = 832.560809f;
	float cx = 319.5f;
	float cy = 239.5f;
	float distortions[5] = { 0.066338f, -1.51067f, 0.f, 0.f, 7.626152f };
	int frameWidth = 640;
	int frameHeight = 480;
	cv::Mat pose = cv::Mat::eye(4,4,CV_32FC1);
	pose.at<float>(0,3) = -0.053976193;;
	pose.at<float>(1,3) = 0.053444523;
	pose.at<float>(2,3) = atof(argv[1]);
	std::cout <<"pose="<<std::endl<<pose<<std::endl;
/*
 * -0.053976193;
 0, 1, 0, 0.0053444523;
 0, 0, 1, 0.93838161;
 *
 *
 */

	std::string mesh_path("/home/tomatito/ws_slam/src/system_setup/meshes/kaffee_aligned_scaled_meters.obj");
	RenderLib renderer(fxy, fxy, cx, cy,  distortions, frameWidth, frameHeight,mesh_path);
	cv::Mat frameDrawing;


	renderer.render(frameDrawing, pose);
	cv::imshow("Show Marker", frameDrawing);
	cv::waitKey(0);
}


int _main(int argc, char **argv)
{
	// offline calibrated camera parameters
	float fxy = 832.560809f;
	float cx = 319.5f;
	float cy = 239.5f;
	float distortions[5] = { 0.066338f, -1.51067f, 0.f, 0.f, 7.626152f };
	Camera cam(fxy, fxy, cx, cy, distortions);


	// load mesh model
	GLMmodel *bmdl = glmReadOBJ("/home/tomatito/ws_slam/src/system_setup/meshes/kaffee_aligned_scaled.obj");
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
	markerTrans.at<float>(0,3) = 0.;
	markerTrans.at<float>(1,3) = 0.;
	markerTrans.at<float>(2,3) = 0.9;
		

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
