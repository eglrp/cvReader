#include "functions.h"
#include "StereoVision.h"

#include <opencv2/viz.hpp>

//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/common/common_headers.h>
//
// 宏定义
#define WINNAME    "Armor Recognition"
#define WINNAME1 "Binary Image"
#define MaxContourArea 450        // 面积大于该值的轮廓不是装甲的灯条
#define MinContourArea 20        // 面积小于该值的轮廓不是装甲的灯条
#define RED 0                    // 0代表红色
#define BLUE 1                    // 1代表蓝色
#define DEBUG

// 双目相机参数
Mat cameraMatrixL = (Mat_<double>(3, 3) << 5.0484366501418896e+02, 0., 3.1825013014114188e+02,
        0., 5.0377604847756851e+02, 2.4937812035308363e+02, 0., 0., 1.);
Mat distCoeffL = (Mat_<double>(1, 5) << 1.3939280830750547e-01, -5.8046874025808826e-01,
        6.3162331897060652e-03, -5.2155045805109441e-03, 1.1149066042238609e+00);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 5.0149028606911679e+02, 0., 3.7221361800666864e+02,
        0., 5.0061684086229377e+02, 2.4279999667360542e+02, 0., 0., 1.);
Mat distCoeffR = (Mat_<double>(1, 5) << 9.5627057175548646e-02, -2.6355268585825314e-01,
        -3.4759260457199919e-03, -1.6600935943118656e-03, 1.9364700494030165e-01);

Mat T = (Mat_<double>(3, 1) << -3.0166846668326889e+02, 5.9076636778872977e+00, -9.9926839177569837e-01);
Mat R = (Mat_<double>(3, 3) << 9.9998251145554928e-01, 5.4010035812116402e-03, -2.4095525244369636e-03,
        -5.2847340658908329e-03, 9.9893213779210044e-01, 4.5898318835843048e-02,
        2.6548764387621594e-03, -4.5884782296744427e-02, 9.9894321079062109e-01);

// 全局变量
map<string, string> config;
Mat frameL, grayL;                // 视频帧及其灰度图
Mat frameR, grayR;                // 视频帧及其灰度图
Mat binaryImage, hsvImage;        // 二值图及HSV图，使用cvtColor得到
int m_threshold;                // 阈值
bool showBinaryImage = false;    // 是否显示二值图
Mat element;                    // 开运算参数
int Width, Height;                // 视频宽高
int detectColor;                // 敌军装甲的颜色：0-红色，1-蓝色
string fileNameL, fileNameR;    // 视频的文件名

// 计算直方图需要的参数
Mat hMat, sMat, vMat;            // HSV单通道图
int channels = 0;                // 计算第0个通道的直方图，calcHist参数
int sizeHist = 180;                // 180个色度，calcHist参数
MatND dstHist;                    // calcHist结果

int main() {
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr(
//            new pcl::visualization::PCLVisualizer("view point cloud"));
//    viewer_ptr->addCoordinateSystem(1.0);

    showText();


    cv::viz::Viz3d win3d("3d show");
    /// Add coordinate axes
    win3d.showWidget("Coordinate Widget", viz::WCoordinateSystem());

    /// Let's assume camera has the following properties
    Vec3f cam_pos(3.0f, 3.0f, 3.0f), cam_focal_point(3.0f, 3.0f, 2.0f), cam_y_dir(-1.0f, 0.0f, 0.0f);

    /// We can get the pose of the cam using makeCameraPose
    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    Affine3f transform = viz::makeTransformToGlobal(Vec3f(0.0f, -1.0f, 0.0f),
                                                    Vec3f(-1.0f, 0.0f, 0.0f),
                                                    Vec3f(0.0f, 0.0f, -1.0f),
                                                    cam_pos);
    /***************************************
            读取 video.cfg 里面的键值对
    ****************************************/
    bool read = ReadConfig("video.cfg", config);    // 把video.cfg读入config键值对，成功返回true
    if (!read) {
        cout << "无法读取 video.cfg" << endl;
#ifdef WIN32
        system("pause");
#endif
        return -1;
    }

    /***************************************
                全局变量初始化
    ****************************************/
    m_threshold = atoi(config["THRESHOLD"].c_str());
    Width = atoi(config["WIDTH"].c_str());
    Height = atoi(config["HEIGHT"].c_str());
    detectColor = atoi(config["DETECTCOLOR"].c_str());
    fileNameL = config["FILENAMEL"];
    fileNameR = config["FILENAMER"];

    element = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
    hMat.create(Size(Width, Height), CV_8UC1);
    sMat.create(Size(Width, Height), CV_8UC1);
    vMat.create(Size(Width, Height), CV_8UC1);
    Mat chan[3] = {hMat, sMat, vMat};
    float hranges[] = {0, 180};
    const float *ranges[] = {hranges};

    /* 声明StereoVision对象 */
    StereoVision stereoVision(cameraMatrixL, distCoeffL,
                              cameraMatrixR, distCoeffR,
                              T, R, Width, Height);

    namedWindow(WINNAME, WINDOW_AUTOSIZE);
    createTrackbar("Threshold", WINNAME, &m_threshold, 255, 0);
    std::cout << fileNameL << std::endl;
    std::cout << fileNameR << std::endl;

    fileNameL = "./v3/outputLeft.avi";
    fileNameR = "./v3/outputRight.avi";
    VideoCapture capL(fileNameL);
    VideoCapture capR(fileNameR);
    std::cout << "./v1/outputLeft.avi" << std::endl;
    std::cout << fileNameL.find("./v1/outputLeft.avi") << std::endl;
//    VideoCapture capL("./v1/outputLeft.avi");
//    VideoCapture capR("./v1/outputRight.avi");

//	if (!capL.isOpened())
//	{
//		cout << "can't open the video file." << endl;
//#ifdef WIN32
//		system("pause");
//#endif
//		return -1;
//	}
//
    //cap.set(CAP_PROP_POS_FRAMES, 375);

    /***************************************
                开始处理对每一帧
    ****************************************/
    while (true) {
        double time0 = static_cast<double>(getTickCount());

        capL >> frameL;
        capR >> frameR;

        if (!frameL.data)
            return -1;

        imshow("frameL", frameL);

        remap(frameL, frameL, stereoVision.mapLx, stereoVision.mapLy,
              INTER_LINEAR);
        remap(frameR, frameR, stereoVision.mapRx, stereoVision.mapRy, INTER_LINEAR);

#ifdef DEBUG
        Mat frame_ = frameL.clone();        // 帧图像备份，调试用
#endif // DEBUG

        cvtColor(frameL, grayL, COLOR_BGR2GRAY);
        cvtColor(frameL, hsvImage, COLOR_BGR2HSV);

        cvtColor(frameR, grayR, COLOR_BGR2GRAY);

        /* 对双目图进行立体匹配 */
        stereoVision.stereoMatch(grayL, grayR);

//		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

//		pc_ptr->push_back()
//		pcl::PointXYZRGB point(20,20,200);
//		point.x = 1.0;
//		point.y = 20.0;
//		point.z = 10.0;
//		pc_ptr->push_back(point);
//
//		viewer.removePointCloud("cloud");
//        viewer.addPointCloud(pc_ptr,"cloud");




        Mat distImg;
        stereoVision.getDisparityImage(distImg);
        imshow("distImg", distImg);

        cv::Mat tmp3d = stereoVision.getXYZIMG();


        cv::Mat cloudmat(tmp3d.rows, tmp3d.cols, CV_32FC3);
        Point3f *data = cloudmat.ptr<cv::Point3f>();
        for (int i(0); i < tmp3d.rows; ++i) {
            for (int j(0); j < tmp3d.cols; ++j) {
                cv::Vec3f tvec;
                stereoVision.getXYZ(cv::Point2f(i, j), tvec);
                data[i * tmp3d.cols + j].x = tvec(0);
                data[i * tmp3d.cols + j].y = tvec(1);
                data[i * tmp3d.cols + j].z = tvec(2);

                std::cout << data[i * tmp3d.cols + j].x << data[i * tmp3d.cols + j].y
                          << data[i * tmp3d.cols + j].z
                          << std::endl;
            }
        }


        win3d.setBackgroundColor();
        cv::viz::WCloud cloud_widget(tmp3d, viz::Color::green());

        std::cout << tmp3d.rows << " : " << tmp3d.cols << std::endl;
//        cloud_widget.setPose(cam_pose);

        viz::WCameraPosition cpw(0.5); // Coordinate axes
        viz::WCameraPosition cpw_frustum(Vec2f(0.889484, 0.523599)); // Camera frustum
        win3d.showWidget("CPW", cpw, cam_pose);
        win3d.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
//        win3d.showImage(tmp3d);
        win3d.showWidget("deep", cloud_widget, cam_pose);
//        win3d.setViewerPose(cam_pose);
        win3d.spinOnce();


        threshold(grayL, binaryImage, m_threshold, 255, THRESH_BINARY);

        morphologyEx(binaryImage, binaryImage, MORPH_OPEN, element);    // 开运算，先腐蚀，后膨胀。去掉小的白色轮廓

#ifdef DEBUG
        Mat binaryImage_ = binaryImage.clone();  // 二值图像备份，调试用
#endif // DEBUG

        vector<vector<Point>> contours;        // 所有轮廓，findContours函数的结果
        findContours(binaryImage, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);    // 寻找轮廓

        vector<vector<Point>> contoursInAreaRange;    // 面积在(MinContourArea, MaxContourArea)范围内的轮廓
        for (int i = 0; i < contours.size(); i++) {
            double areaTemp = contourArea(contours[i]);
            if (areaTemp > MinContourArea && areaTemp < MaxContourArea)
                contoursInAreaRange.push_back(contours[i]);
        }

        vector<RotatedRect> rotatedRects;    // 对面积在指定范围内的轮廓拟合椭圆，得到相应的旋转矩形
        for (int i = 0; i < contoursInAreaRange.size(); i++)
            rotatedRects.push_back(fitEllipse(contoursInAreaRange[i]));

        /* 为每一个符合条件的RotatedRect制作一个掩模，然后计算掩模区域的直方图，
         * 通过直方图判断该RotatedRect的主要颜色是红色还是蓝色
         */
        vector<RotatedRect> rotatedRectsOfLights;    // 蓝色/红色灯条的RotatedRect
        split(hsvImage, chan);                        // 把HSV图像分为3个通道，用于计算直方图
        for (int i = 0; i < contoursInAreaRange.size(); i++) {
            Point2f pointTemp[4];
            rotatedRects[i].points(pointTemp);
            vector<Point> corners;
            for (int j = 0; j < 4; j++)
                corners.push_back(pointTemp[j]);

            vector<vector<Point>> corners_;
            corners_.push_back(corners);
            Mat mask(Height, Width, CV_8UC1, Scalar::all(0));
            drawContours(mask, corners_, -1, Scalar(255), -1, LINE_AA);
            dilate(mask, mask, element);

            calcHist(&hMat, 1, &channels, mask, dstHist, 1, &sizeHist, ranges);

            if (JudgeColor(dstHist) == detectColor)
                rotatedRectsOfLights.push_back(rotatedRects[i]);
        }

        for (int i = 0; i < rotatedRectsOfLights.size(); i++) {
            ellipse(frame_, rotatedRectsOfLights[i], Scalar(0, 255, 0), 2, LINE_AA);
        }

        // 装甲灯条的世界坐标
        vector<Vec3f> coordinateOfLights;
        for (int i = 0; i < rotatedRectsOfLights.size(); i++) {
            Vec3f temp;
            stereoVision.getXYZ(rotatedRectsOfLights[i].center, temp);
            coordinateOfLights.push_back(temp);
        }

        for (int i = 0; i < coordinateOfLights.size(); i++) {
            cout << coordinateOfLights[i] << endl;
        }
        cout << "--------------" << endl;

        for (int i = 0; i < coordinateOfLights.size() - 1; i++) {
            for (int j = i + 1; j < coordinateOfLights.size(); j++) {
                if (isPair(coordinateOfLights[i], coordinateOfLights[j])) {

                    circle(frame_, rotatedRectsOfLights[i].center, 3, Scalar(0, 0, 255), -1, LINE_AA);
                    circle(frame_, rotatedRectsOfLights[j].center, 3, Scalar(0, 255, 255), -1, LINE_AA);

                    Point p = centerOf2Points(rotatedRectsOfLights[i].center,
                                              rotatedRectsOfLights[j].center);
                    circle(frame_, p, 10, Scalar(0, 0, 255), 2, LINE_AA);
                    char distance[20];
                    sprintf(distance, "%fmm", coordinateOfLights[i][2]);
                    putText(frame_, distance, Point(10, 30), CV_FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 255), 1,
                            LINE_AA);
                }
            }
        }

        imshow(WINNAME, frame_);

        if (showBinaryImage)
            imshow(WINNAME1, binaryImage_);

        int key = waitKey(1);
        if (key == 27) {
            break;
        } else if (key == int('0')) {
            if (showBinaryImage)    // 如果这时候窗口是显示的，则关闭它
                destroyWindow(WINNAME1);
            showBinaryImage = !showBinaryImage;
        } else if (key == int('1')) {
            waitKey(0);
        }

        time0 = ((double) getTickCount() - time0) / getTickFrequency();
        cout << "用时" << time0 * 1000 << "毫秒" << endl;
    }

    return 0;
}

void showText() {
    cout << "操作提示：" << endl;
    cout << "\t按【0】键显示阈值分割的结果" << endl;
    cout << "\t按【1】键暂停视频" << endl;
}

// 计算直方图中分布最多的颜色（白色除外），选取前20个分布最多对的颜色值，看哪个区域最多
int JudgeColor(MatND hist) {
    // 声明两个数组，color表示颜色值0~179
    int color[180];
    for (int i = 0; i < 180; i++)
        color[i] = i;

    // value表示直方图中第i个颜色对应的值
    int value[180];
    for (int i = 0; i < 180; i++)
        value[i] = static_cast<int>(hist.at<float>(i));

    choise(value, color, 180);

    int red = 0;
    int blue = 0;
    int otherColor = 0;

    for (int i = 0; i < 15; i++) {
        int binValue = color[i];
        if (binValue > 99 && binValue < 125) {
            blue++;
        } else if ((binValue > -1 && binValue < 11) || (binValue > 155 && binValue < 181)) {
            red++;
        } else
            otherColor++;
    }

    if (red > blue && red > otherColor) {
        return RED;
    } else if (red < blue && blue > otherColor) {
        return BLUE;
    } else
        return -1;
}

void choise(int *a, int *b, int n) {
    int i, j, k, temp;

    for (i = 0; i < n - 1; i++) {
        k = i;    // 给记号赋值

        for (j = i + 1; j < n; j++)
            if (a[k] < a[j]) k = j;    // 是k总是指向最小元素

        // 当k!=i是才交换，否则a[i] 即为最小
        if (i != k) {
            temp = a[i];
            a[i] = a[k];
            a[k] = temp;

            temp = b[i];
            b[i] = b[k];
            b[k] = temp;
        }
    }
}

bool isPair(Vec3f v1, Vec3f v2) {
    float f0 = abs(v1[0] - v2[0]);
    float f1 = abs(v1[1] - v2[1]);
    float f2 = abs(v1[2] - v2[2]);

    if (f0 < 200 && f1 < 50 && f2 < 40)
        return true;
    else
        return false;
}

Point centerOf2Points(Point p1, Point p2) {
    Point p;
    p.x = (p1.x + p2.x) / 2;
    p.y = (p1.y + p2.y) / 2;
    return p;
}
