//phenox part
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <sys/mman.h>
#include <signal.h>
#include <sys/time.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

//AR part
#include "ar_detect.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <aruco.h>
#include <vector>
#include <time.h>

using namespace Eigen;
using namespace std;
using namespace cv;
using namespace aruco;

AR_DETECT::AR_DETECT(){
	zero = Vector3f::Zero();
	F  = Matrix3f::Identity();
	G  = Matrix3f::Identity();
	Q  = Matrix3f::Identity();
	Sigma = Matrix3f::Identity()*0.5;
	y     = VectorXf(N3);
	map<int,Vector3f> AR_id;
        CameraParameters params;
	MarkerDetector detector;
	vector<Marker> markers;
	//追加だよ
	//Vector3f mu_tmp(0.0,0.0,0.0);		

}

CameraParameters AR_DETECT::CameraLoad(Mat frame){
		// カメラパラメタのロード
		//CameraParameters params;
		params.readFromXMLFile("./intrinsics.yml");
		params.resize(frame.size());
		return params;
	}

vector<Marker> AR_DETECT::ARDetect(Mat frame, CameraParameters params){
		// マーカを認識する
		//MarkerDetector detector;
		//vector<Marker> markers;
		const float markerSize = 0.185f; //マーカーサイズに合わせて変更する
		detector.detect(frame, markers, params, markerSize);
	  return markers;
	}

Mat AR_DETECT::outPut(Mat frame){
		 Mat outputImage=frame.clone();
		 return outputImage;
	}

Vector3f AR_DETECT::LKF(Mat outputImage,vector<Marker> markers,CameraParameters params,map<int,Vector3f> AR_id,Vector3f u)
	{
		//定数
		//clock_t start;
		//clock_t end;
		vector<float> Y;
		vector<int>  ID;
		for (auto&& marker : markers) {
			ID.push_back(marker.id);
			Y.push_back(marker.Tvec.at<float>(0,0)*(-1.0));
			Y.push_back(marker.Tvec.at<float>(0,1));
			Y.push_back(marker.Tvec.at<float>(0,2));
			marker.draw(outputImage, cv::Scalar(0, 0, 255), 2);
			aruco::CvDrawingUtils::draw3dCube(outputImage, marker, params);
			}
			//vector→Vector変換
			int N3=Y.size();
			//観測があるときのみ更新を行う
			if (N3 !=0){
				VectorXf y(N3);
				VectorXf id(int(ID.size()));
				int j=0;
				for (j;j<int(N3/3);j++){
					y(3*j)=Y[3*j];
					y(3*j+1)=Y[3*j+1];
					y(3*j+2)=Y[3*j+2];
					id(j)=ID[j];
					}
				//予測
				end = clock();
				t=double(end - start)/CLOCKS_PER_SEC;
				cout << "duration"<< t << endl;
				//速度の取得
				//ダミーデータ
				//u(0,0)=0.0;
				//u(1,0)=0.0;
				//u(2,0)=0.0;
				//u(0,0)=px_selfstate.vision_vx;
				//u(1,0)=px_selfstate.vision_vy;
				//u(2,0)=px_selfstate.vision_vz	;
				mu_=F*mu+G*u*t; //tは取得時間間隔
				Sigma_=Q+F*Sigma*F.transpose();
				start =clock();
				//更新
				//H　観測行列 3N×3, R 観測ノイズの共分散行列　3N×3N
				VectorXf 	yi(N3);
				VectorXf  yres(N3);
				MatrixXf	H(N3,3);
				MatrixXf R = MatrixXf::Identity(N3,N3);
				MatrixXf  S(N3,N3);
				MatrixXf	K(3,N3);
				int i=0;
				for (i;i<(N3/3);i++){
					H(3*i,0)=1;
					H(3*i,1)=0;
					H(3*i,2)=0;
					H(3*i+1,0)=0;
					H(3*i+1,1)=1;
					H(3*i+1,2)=0;
					H(3*i+2,0)=0;
					H(3*i+2,1)=0;
					H(3*i+2,2)=1;
					yres(3*i)  =(AR_id[id(i)])[0];
					yres(3*i+1)= (AR_id[id(i)])[1];
					yres(3*i+2)=(AR_id[id(i)])[2];
				}
			yi=y-H*mu_+yres;
			S=H*Sigma_*H.transpose()+R;
			K=Sigma_*H.transpose()*S.inverse();
			mu=mu_+K*yi;
			Sigma=Sigma_-K*H*Sigma_;
			cout << mu(0) << endl;
			if ( -1.2<mu(0) && mu(0)<1.2 && -0.5<mu(1) && mu(1)<4.0){
			//	cout<<"ok"<<mu(0)<<endl;
				mu_tmp=mu;
			}
		//	mu_tmp=mu;
	}
//	return mu;
//	cout <<mu_tmp<<endl;
	return mu_tmp;	
	}
