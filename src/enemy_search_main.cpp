#include "myHead.hpp"
#include <opencv2/opencv.hpp>
#include "fstream"
//#include "opticalFlow.h"
#include "getConfig.h"
#include <sstream>
#include <opticalFlow.h>

#include "pthread.h"

using namespace cv;
using namespace std;

#define IMAGE_WIDTH 640    //图像宽度
#define IMAGE_HEIGHT 480   //图像高度
#define ZOOM_FACTOR 1      //为加快处理速度，对图像进行等比例缩小，此处为0.5倍
#define DEBUG            //图像显示
#define BLUE
#define CAM
#define DARK
#define SEND_NUM
//#define SEND_ANGLE

#define DRONE_DETECT

#define minContour 15
#define maxContour 600

bool stopProc = false;
bool undistortMode = true;
bool imgCatched = false;
bool thresholdMode = false;
bool paused = false;

int sendPeriod = 20; //发送周期
const char *g_szTitle = "Camera";
map<string, string> config;

#ifdef SEND_ANGLE
double anglePitch = 0, angleYaw = 0;
#else
char regionNum[1] = { 0xFF };
#endif

void *sendData(void *)
{
#ifndef WIN32
	Serialport stm32("/dev/ttyS0");
#ifdef SEND_ANGLE
	while (!stopProc)
	{
		//cout<<angleYaw<<"    "<<anglePitch<<endl;
		stm32.sendAngle(angleYaw, anglePitch);
		usleep(sendPeriod * 1000);
	}
#else
	while (!stopProc)
	{
		if (!paused)
		{
			stm32.send(regionNum);
			cout << hex << (int)regionNum[0] << endl;
		}
		usleep(sendPeriod * 1000);
	}
#endif // SEND_ANGLE
#endif // LINUX
	cout << "send data pthread exit\n" << endl;
}

void *droneDetect(void *)
{
#ifdef DRONE_DETECT
	VideoCapture cap(1);
	while (!cap.isOpened() && !stopProc)
	{
		//cout << "can't open the drone detection camera!\n";
		//cout << "drone detection pthread exit\n" << endl;
		cap.open(1);
		waitKey(5000);
	}
	Mat src, result;
	OpticalFlow droneOF(50, 0.01, 10);
	int numSend = 0;
	while (!stopProc)
	{
		int64 t0 = getTickCount();
		cap >> src;
		if (src.empty())
		{
			cerr << " --(!) No captured frame -- Break!\n";
			break;
		}

		droneOF.tracking(src);
		result = droneOF.output;
		// 			Point2f vertex[4];
		// 			droneOF.motionObject.points(vertex);
		// 			for (int i = 0; i < 4; ++i)
		// 				line(result, vertex[i], vertex[(i + 1) % 4], Scalar(0, 0, 200),2);

		if (droneOF.isDroneAppear())
		{
#ifdef DEBUG
			vector<Point2f> points = droneOF.movingPoints;
			vector<int> hull;
			convexHull(Mat(points), hull, true);
			int hullcount = (int)hull.size();//Í¹°üµÄ±ßÊý
			Point point0 = points[hull[hullcount - 1]];//Á¬œÓÍ¹°ü±ßµÄ×ø±êµã
			for (int i = 0; i < hullcount; i++)
			{
				Point point = points[hull[i]];
				line(result, point0, point, Scalar(255, 0, 0), 2, CV_AA);
				point0 = point;
			}
#endif // DEBUG
			numSend++;
			if (numSend >= 2)
			{
				cout << "target emerged\n";
				regionNum[0] = 0xee;
			}
		}
		else
		{
			numSend = 0;
			//cout << "no drones\n";
		}
		int key = waitKey(1);
#ifdef DEBUG
		//int64 t = getTickCount() - t0;
		//cout << t * 1000 / getTickFrequency() << endl;
		//imshow("optical flow", result);
		if ((char)key == 27)
			break;
#endif // DEBUG

	}
	cap.~VideoCapture();
#endif // DRONE_DETECT
	regionNum[0] = 0xf1;
	cout << "drone detection pthread exit\n" << endl;
}

int main()
{
	ReadConfig("video.cfg", config);
	int t = atoi(config["t"].c_str());
	VideoCapture cap(0);
	//VideoCapture cap("D:\\基地\\西安交通\\45.avi");
	//cap.set(CV_CAP_PROP_POS_FRAMES, 25*60);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);
	if (!cap.isOpened())
	{
		cerr << "can't open the armor detection camera!\n";
		return 0;
	}

#ifndef WIN32
	pthread_t id, id2;
//	int ret = pthread_create(&id, NULL, sendData, NULL);
	int ret2 = pthread_create(&id2, NULL, droneDetect, NULL);
#endif //创建线程

#ifdef DEBUG
	namedWindow(g_szTitle);
	createTrackbar("t", g_szTitle, &t, 256, 0);
#else
	string videoName;
	videoName += config["num"];
	videoName += ".avi";
	VideoWriter writer(videoName, CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(640, 480));
	config["num"] = to_string(atoi(config["num"].c_str()) + 1);
	WriteConfig("video.cfg", config);
#endif // DEBUG

	Mat src, imgThresholded;
	cap >> src;

	Mat imgOriginal = src(Rect(0, (int)IMAGE_HEIGHT / 3, IMAGE_WIDTH, (int)IMAGE_HEIGHT * 2 / 3));
	Rect roiImg = Rect(0, 0, imgOriginal.cols * ZOOM_FACTOR, imgOriginal.rows * ZOOM_FACTOR);

	vector<RotatedRect> vEllipse;//符合条件的椭圆
	Armors armors(imgOriginal.cols*ZOOM_FACTOR, imgOriginal.rows*ZOOM_FACTOR);
	vector<vector<Point> > contours;

	int sendFilter = 0;
	bool sendBool = false;
	while (true)
	{
		String noTargetReason;
		int64 t0 = getTickCount();
		if (!paused)
		{
			if (!cap.read(src))
				break;
#ifndef DEBUG
			writer << src;
#endif // DEBUG
			imgOriginal = src(Rect(0, (int)IMAGE_HEIGHT / 3, IMAGE_WIDTH, (int)IMAGE_HEIGHT * 2 / 3));
			resize(imgOriginal, imgOriginal, Size(imgOriginal.cols * ZOOM_FACTOR, imgOriginal.rows * ZOOM_FACTOR));//等比例缩小
			//imgOriginal -= Scalar(B,G,R);
#ifndef DARK
#ifdef BLUE
			GetDiffImage(imgOriginal, imgThresholded, t, blue, roiImg);//蓝色
#else
			GetDiffImage(imgOriginal, imgThresholded, t, red, roiImg);//红色
#endif
#else//Dark
			GetBrightImage(imgOriginal, imgThresholded, t, roiImg);
#endif // DARK

			preProcess(imgThresholded);
			findContours(imgThresholded, contours, RETR_CCOMP, CHAIN_APPROX_NONE);

#ifdef DEBUG
			drawContours(imgThresholded, contours, -1, Scalar(255, 255, 255));
#endif // DEBUG

			for (vector<vector<Point> >::const_iterator itContours = contours.begin(); itContours != contours.end(); ++itContours)
			{
				vector<Point> points = *itContours;//将一个轮廓的所有点存入points
				if (itContours->size() > minContour && itContours->size() < maxContour)//筛选轮廓上的点大于100的轮廓
				{
					RotatedRect s = fitEllipse(Mat(points));//拟合椭圆
					//RotatedRect s = minAreaRect((points));//拟合椭圆
					if ((s.size.height < s.size.width) || (s.size.height * s.size.width > 2000))//|| (s.size.height / s.size.width > 12))
					{
						//cout<<s.size<<endl;
						noTargetReason += "size don't cater to standard.\n";
						continue;
					}
#ifdef DARK
					Rect colorRect = (s.boundingRect() - Point(7, 7) + Size(14, 14)) & roiImg;
					int color = colorJudge(imgOriginal(colorRect));
#ifdef BLUE
					if (color == blue)
						vEllipse.push_back(s);
					else
						continue;
#endif // BLUE
#ifdef RED
					if (color == red || color == purple)
						vEllipse.push_back(s);
					else
						continue;

#endif // RED
#else//Bright
					vEllipse.push_back(s);
#endif // DARK
#ifdef DEBUG
					ellipse(imgOriginal, s, Scalar(255, 255, 66), 2);
#endif // DEBUG
				}
				else
				{
					noTargetReason += "size of contours is too small or too big.\n";
				}

				points.clear();//points.swap(vector<Point>());
			}
			armors.inputEllipse(vEllipse);//输入将测到的椭圆，寻找装甲
			Point2f target = armors.getTarget();//求目标坐标
			//target = armors.track();//追踪

			if (armors.number() > 0)//目标没有丢失
			{
				roiImg = armors.getROIbox(imgOriginal);
#ifdef DEBUG
				armors.drawAllArmors(imgOriginal);
				circle(imgOriginal, target*ZOOM_FACTOR, 5, Scalar(0, 255, 255), 1, 8, 3);
				line(imgOriginal, target, target + armors.getVelocity(), Scalar(240, 33, 22), 2, CV_AA);
#endif//DEBUG


				if (sendFilter >= 2)
				{
					sendBool = true;
				}
				else
				{
					sendFilter++;
				}
				if (sendBool)
				{
					regionNum[0] = armors.getTargetRegion();
				}
				else
					regionNum[0] = 0xff;


			}
			else//目标丢失
			{
#ifdef DEBUG
				//noTargetReason = armors.error;
				cout << noTargetReason << endl;
				putText(imgOriginal, "Looking for the enemy.......", Point(90 * ZOOM_FACTOR, 90 * ZOOM_FACTOR), FONT_HERSHEY_PLAIN, 2 * ZOOM_FACTOR, Scalar(0, 0, 255), 2, 8);
#endif // DEBUG

				roiImg = Rect(0, 0, imgOriginal.cols * ZOOM_FACTOR, imgOriginal.rows * ZOOM_FACTOR);
#ifdef SEND_ANGLE
				anglePitch = 0;
				angleYaw = 0;
#else
				sendFilter--;
				if (sendFilter < 0)
				{
					sendFilter = 0;
					sendBool = false;
					regionNum[0] = 0xff;
				}

#endif // SEND_ANGLE
			}
			vEllipse.clear();
			contours.clear();//contours.swap(vector<vector<Point>>());
		}
		char key = (char)waitKey(1);

#ifdef DEBUG
		if (key == 27)  break;  //esc键退出
#ifdef SEND_NUM
		drawGrid(imgOriginal);
#endif // SEND_NUM
		if (thresholdMode)
			imshow(g_szTitle, imgThresholded);
		else
			imshow(g_szTitle, imgOriginal);
		switch (key)
		{
		case 'b':
			thresholdMode = !thresholdMode;
			break;
		case 'p':
			paused = !paused;
			break;
		case 'd':
			undistortMode = !undistortMode;
			break;
		default:
			;
		}
		int64 t = getTickCount() - t0;
		//cout << "主线程：" <<1000 * t/getTickFrequency() << "ms" << "\n";
#endif // DEBUG
	}

#ifndef WIN32
	stopProc = true;
	pthread_join(id, NULL);//wait the end of sendData
	pthread_join(id2, NULL);//wait the end of sendData
#endif // LINUX
	cap.~VideoCapture();
	destroyAllWindows();
	config["t"] = to_string(t);
	WriteConfig("video.cfg", config);
	cout << "main pthread exit\n" << endl;
	regionNum[0] = 0xf0;
	return 0;
}
