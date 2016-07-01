#include <Windows.h>
#include <Kinect.h>
#include <chrono>
#include "Coordinate2D.h"
#include <cmath>
#include <string>
#include <stdio.h>
#include <iostream>
#include <direct.h>


#include "aruco.h"
#include "cvdrawingutils.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2\opencv.hpp>

#include "dirent.h"


#include <fstream>

using namespace cv;

using namespace std;
using namespace aruco; 

int main() {
	cv::setUseOptimized( true );
	int obsNum = 52;
	string observationNumber = std::to_string(obsNum);
	string aviExtension = ".avi";
	string ymlExtension = ".yml";
	string csvExtension = ".csv";

	// Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor( &pSensor );
	if( FAILED( hResult ) ){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open();
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	MarkerDetector MDetector;
    vector<Marker> Markers;
	vector<ColorSpacePoint> colorsps;
	vector<Coordinate2D> jointCoordinates2d;

	string colorCapturePath = "C:/Users/efe/Desktop/Videos-HOP/Color/" + observationNumber + aviExtension;
	string ymlCapturePath = "C:/Users/efe/Desktop/Videos-HOP/YML-Depth/" + observationNumber + "/";
	string observationDataPath = "C:/Users/efe/Desktop/Videos-HOP/ObservationData/" + observationNumber + csvExtension; 

	VideoCapture colorCapture(colorCapturePath); // open the video file for reading

    if ( !colorCapture.isOpened() ) {  // if not success, exit program
		cout << "Cannot open the video file" << endl;
        return -1;
    }

    double fps = colorCapture.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Frame per seconds : " << fps << endl;
    namedWindow("ColorCapture", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
	//namedWindow("DepthCapture", CV_WINDOW_AUTOSIZE);

	int startFlag = 0;
	int stopFlag = 0;
	int stopMarkerExists = 0;
	cv::Mat depthBufferMat;
	int colorHeight = 1080;
	int colorWidth = 1920;
	int depthHeight = 424;
	int depthWidth = 512;

	int frameCounter = 0;
	while(true)
    {
        Mat colorFrame;
		
        bool colorSuccess = colorCapture.read(colorFrame); // read a new frame from video
        if (!colorSuccess) { //if not success, break loop
			cout << "Cannot read the frame from video files" << endl;
            break;
        }

		MDetector.detect(colorFrame,Markers);
		ColorSpacePoint wristMarkerPosition;
		for (unsigned int i=0;i<Markers.size();i++) {

			if(Markers[i].id == 500) {
				startFlag = 1;
			}

			if(Markers[i].id == 567) {
				stopFlag = 0;
			}
			else {
				stopFlag = 1;
			}
			//cout<<"Marker id:"<<Markers[i].id<<endl;
			//cout<<"Start flag:"<<startFlag<<endl;
			//cout<<"Stop flag:"<<stopFlag<<endl;

			if(Markers[i].id == 462) {
				
				wristMarkerPosition.X = static_cast<int>(Markers[i].getCenter().x);
				wristMarkerPosition.Y = static_cast<int>(Markers[i].getCenter().y);
				
				//colorsps.push_back(tmpColor);
			}
            Markers[i].draw(colorFrame,Scalar(0,0,255),2);
        }

		string folderPath = ymlCapturePath;
		string fileName = folderPath + std::to_string(frameCounter) + ".yml";
		cv::FileStorage fs2(fileName, FileStorage::READ);
		fs2["yourMat"] >> depthBufferMat;
		
		std::vector<CameraSpacePoint> cameraSpacePoints( colorHeight*colorWidth );
		hResult = pCoordinateMapper->MapColorFrameToCameraSpace( depthHeight*depthWidth, reinterpret_cast<UINT16*>( depthBufferMat.data ), colorWidth*colorHeight, &cameraSpacePoints[0] );
		
		if(SUCCEEDED(hResult)) {
			if(startFlag == 1 && stopFlag == 0) {
				cout<<"Inside"<<endl;
				int colorX = static_cast<int>(wristMarkerPosition.X + 0.5f);
				int colorY = static_cast<int>(wristMarkerPosition.Y + 0.5f);
				long colorIndex = (long)(colorY*colorWidth + colorX);
				CameraSpacePoint csp = cameraSpacePoints[colorIndex];
				CameraSpacePoint tmp;
				Coordinate2D tmp2D;
				tmp.X = -csp.X;
				tmp.Y = csp.Y;
				tmp.Z = csp.Z;
	
				//cout<<"X: "<<tmp.X<<" Y: "<<tmp.Y<<" Z: "<<tmp.Z<<endl;

				tmp2D.setX((float) tmp.X/tmp.Z);
				tmp2D.setY((float) tmp.Y/tmp.Z);
				cout<<"X: "<<tmp2D.X<<" Y: "<<tmp2D.Y<<endl;
				jointCoordinates2d.push_back(tmp2D);
			}
		}

        imshow("ColorCapture", colorFrame); //show the frame in "MyVideo" window
		//imshow("DepthCapture", depthFrame);
        if(waitKey(5) == 27) { //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
			cout << "esc key is pressed by user" << endl; 
            break; 
		}

		frameCounter++;
	} // end of main while loop

	// write the jointCoordinates2d vector to observationDataPath

	string fileName = observationDataPath;

	ofstream myCsvFile;
	myCsvFile.open(fileName, ios::out | ios::app);
					
	for(int q=0; q<jointCoordinates2d.size(); q++) {
		myCsvFile<<jointCoordinates2d[q].getX()<<","<<jointCoordinates2d[q].getY()<<",";
		myCsvFile<<"\n";
	}
	myCsvFile.close();

	system("PAUSE");
	return 0;
}