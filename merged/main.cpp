/*
Copyright (c) 2015 Ryo Konomura

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <termios.h>
#include <pthread.h>
#include <errno.h>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "pxlib.h"
#include "parameter.h"
#include <iostream>
#include <fstream>
#include <turbojpeg.h>
#include "boundary_detector.h"
#include "control.h"
#include "ar_detect.h"

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

#include "src/SioClientWrapper.h"
#include "src/Parser.h"//受信データのsio::message::ptrから値を抽出するためのサンプル
#include "src/DataMaker.h"//送信データを生成するためのサンプル


using namespace std;
using namespace cv;
using namespace Eigen;
using namespace aruco;


static void setup_timer();

static char timer_disable = 0;
static void *timer_handler(void *ptr);

pthread_t timer_thread;
pthread_mutex_t mutex;
Vector2f gnorm(0, 0);
Vector2f gnorm2(0, 0);
Vector2f gnorm_start(0, 0);
Vector2f gnorm_start2(0, 0);

Vector3f gmu(0,0,0);
int gboundary_cnt = 0;

// const px_cameraid cameraid = PX_FRONT_CAM;
const px_cameraid cameraid = PX_BOTTOM_CAM;


double get_time() {
    struct timeval  tv;
    gettimeofday(&tv, NULL);

    return (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
}

int main(int argc, char **argv)
{
  int i,j,count;
  count = 0;

  double delta, start_time;
  delta = start_time = get_time();
  int frames_count = 0;
  double fps = 0.0;
  double last_time = get_time();


  // phenox initialization -----------------------------------------------------
  pxinit_chain();
  set_parameter();
  printf("CPU0:Start Initialization. Please do not move Phenox.\n");
  while(!pxget_cpu1ready());
  setup_timer();
  printf("CPU0:Finished Initialization.\n");


  // image ---------------------------------------------------------------------
  IplImage *testImage;
  count = 0;

  pxset_led(0,1);
  const int ftmax = 200;
  px_imgfeature *ft =(px_imgfeature *)calloc(ftmax,sizeof(px_imgfeature));
  int ftstate = 0;

  // for boundary detector ----------------------------------------------------
  Mat mat;
  BoundaryDetector bd;
  Vector2f norm, norm_start, norm_end, norm2, norm_start2, norm_end2;



  //for ar detection ----------------------------------------------------------
  map<int,Vector3f> AR_id;
  Vector3f v0(-0.70, 0.56,0.0);
  Vector3f v20(-0.35, 0.56,0.0);
  Vector3f v40( 0.35, 0.56,0.0);
  Vector3f v60( 0.70, 0.56,0.0);

  Vector3f v80(-0.70,  1.06,0.0);
  Vector3f v110(-0.35, 1.06,0.0);
  Vector3f v120( 0.35, 1.06,0.0);
  Vector3f v130( 0.70, 1.06,0.0);

  Vector3f v140(-0.70, 1.56,0.0);
  Vector3f v160(-0.35, 1.56,0.0);
  Vector3f v170( 0.35, 1.56,0.0);
  Vector3f v180( 0.70, 1.56,0.0);

  Vector3f v190(-0.70, 2.06,0.0);
  Vector3f v220(-0.35, 2.06,0.0);
  Vector3f v230( 0.35, 2.06,0.0);
  Vector3f v240( 0.70, 2.06,0.0);

  Vector3f v250(-0.70, 2.56,0.0);
  Vector3f v260(-0.35, 2.56,0.0);
  Vector3f v270( 0.35, 2.56,0.0);
  Vector3f v280( 0.70, 2.56,0.0);

  Vector3f v290(-0.70, 3.06,0.0);
  Vector3f v300(-0.35, 3.06,0.0);
  Vector3f v310( 0.35, 3.06,0.0);
  Vector3f v320( 0.70, 3.06,0.0);

  Vector3f v330(-0.70, 3.56,0.0);
  Vector3f v340(-0.35, 3.56,0.0);
  Vector3f v350( 0.35, 3.56,0.0);
  Vector3f v360( 0.70, 3.56,0.0);

  Vector3f v370(-0.70, 0.06,0.0);
  Vector3f v380(-0.35, 0.06,0.0);
  Vector3f v390( 0.35, 0.06,0.0);
  Vector3f v400( 0.70, 0.06,0.0);

  Vector3f v410( -1.05, 0.06,0.0);
  Vector3f v420( -1.05, 0.56,0.0);
  Vector3f v430( -1.05, 1.06,0.0);
  Vector3f v440( -1.05, 1.56,0.0);
  Vector3f v450( -1.05, 2.06,0.0);
  Vector3f v460( -1.05, 2.56,0.0);
  Vector3f v470( -1.05, 3.06,0.0);

  Vector3f v480( -1.05, -0.46,0.0);
  Vector3f v490( -0.70, -0.46,0.0);
  Vector3f v500( 0.70, -0.46,0.0);

  AR_id[0] = v0;
  AR_id[20]=  v20;
  AR_id[40]=  v40;
  AR_id[60]=  v60;
  AR_id[80]=  v80;
  AR_id[110]=  v110;
  AR_id[120] = v120;
  AR_id[130]=  v130;
  AR_id[140]=  v140;
  AR_id[160]=  v160;
  AR_id[170]=  v170;
  AR_id[180]=  v180;
  AR_id[190] = v190;
  AR_id[220]=  v220;
  AR_id[230]=  v230;
  AR_id[240]=  v240;
  AR_id[250]=  v250;
  AR_id[260]=  v260;
  AR_id[270] = v270;
  AR_id[280]=  v280;
  AR_id[290]=  v290;
  AR_id[300]=  v300;
  AR_id[310]=  v310;
  AR_id[320]=  v320;
  AR_id[330] = v330;
  AR_id[340]=  v340;
  AR_id[350]=  v350;
  AR_id[360]=  v360;
  AR_id[370] = v370;
  AR_id[380]=  v380;
  AR_id[390]=  v390;
  AR_id[400]=  v400;

  AR_id[410]=  v410;
  AR_id[420]=  v420;
  AR_id[430]=  v430;
  AR_id[440]=  v440;
  AR_id[450]=  v450;
  AR_id[460]=  v460;
  AR_id[470]=  v470;

  AR_id[480]=  v480;
  AR_id[490]=  v490;
  AR_id[500]=  v500;


  Vector3f mu,u,px_v;
  double t;
  clock_t start;
  clock_t end;
  start =0.0;
  AR_DETECT *ad;
  ad = new AR_DETECT;

  // main loop ----------------------------------------------------------------
  while(1) {
    if(pxget_imgfullwcheck(cameraid,&testImage) == 1) {
      frames_count++;
      //static int image_count = 0;
      //cout << frames_count << endl;
      mat = cvarrToMat(testImage);
      //stringstream ss;
      //ss << "px_image" << image_count << ".png";
      //imwrite(ss.str(), mat);
      //image_count++;

      int gn = bd.get_norm(&mat, &norm_start, &norm, &norm_start2, &norm2);
      static px_selfstate st;
      pxget_selfstate(&st);
      px_v = Vector3f(st.vision_vx,st.vision_vy,st.vision_vx);
      u=px_v;
      CameraParameters params = ad->CameraLoad(mat);
      vector<Marker> markers  = ad->ARDetect(mat,params);
      Mat outputImage =ad->outPut(mat);
      mu = ad->LKF(outputImage,markers,params,AR_id,u);
      // critical section start--------------------------------------------
      pthread_mutex_lock(&mutex);
      gboundary_cnt = gn;
      gnorm = norm;
      gnorm2 = norm2;
      gnorm_start = norm_start;
      gnorm_start2 = norm_start2;
      gboundary_cnt = gn;
      gmu = mu ;
      pthread_mutex_unlock(&mutex);
      // critical section end--------------------------------------------
      //
      if(pxset_imgfeature_query(cameraid) == 1) {
	      ftstate = 1;
      }
      usleep(1000);
    }
  }
  delete ad;
}

static void setup_timer() {
    struct sigaction action;
    struct itimerval timer;
    timer_thread = pthread_create(&timer_thread, NULL, timer_handler, NULL);
    pthread_mutex_init(&mutex, NULL);
}


void *timer_handler(void *ptr) {
    if(timer_disable == 1) {
        return NULL;
    }
    struct timespec _t;
    static struct timeval now, prev;
    double dt = 0;
    clock_gettime(CLOCK_REALTIME, &_t);
    static PxController ctrlr;

    // --------------------------------------------------------------------------
    // SioClient initialization -------------------------------------------------
    // --------------------------------------------------------------------------
    SioClientWrapper client;
    sio::message::ptr data;
    //発生するイベント名一覧をstd::vector<std::string>としてclientに渡す
    std::vector<std::string> eventList(0);
    eventList.push_back("landing");
    eventList.push_back("direction");
    eventList.push_back("px_bounce");
    eventList.push_back("px_start");
    eventList.push_back("px_position");
    eventList.push_back("px_velocity");
    eventList.push_back("px_ready");
    client.setEventList(eventList);

    //自身を表す部屋名を設定する(Phenoxなら例えば"Phenox"と決める)
    client.setMyRoom("Phenox");
    //データの送信宛先となる部屋名を設定する(Gameサーバなら例えば"Game")
    client.setDstRoom("Game");
    //URLを指定して接続開始
    client.start("http://192.168.1.58:8000");


    while(1) {
        pxset_keepalive();
        pxset_systemlog();
        pxset_img_seq(cameraid);

        static unsigned long msec_cnt = 0;
        msec_cnt++;

        gettimeofday(&now, NULL);
        dt = (now.tv_sec - prev.tv_sec) +
                (now.tv_usec - prev.tv_usec) * 1.0E-6;
        if(dt < 0){
            cout << "dt < 0" << endl;
            continue;
        }
        prev = now;

        // vision control
        static px_selfstate st;
        static Vector2f pos;
        pxget_selfstate(&st);
        pos << st.vision_tx, st.vision_ty;

        Vector2f norm, norm_start;
        Vector2f norm2, norm_start2;
        Vector3f mu;
        static Vector2f input(0,0);
        int bounded;

        bool lineisVisible;
        static Vector2f pre_norm;

        // -------------------------------------------------------------------
        // get boundary norm -------------------------------------------------
        // -------------------------------------------------------------------
        int boundary_cnt = 0;
        if(pthread_mutex_trylock(&mutex) != EBUSY){
            norm = gnorm;
            norm_start = gnorm_start;
            norm2 = gnorm2;
            norm_start2 = gnorm_start2;
            boundary_cnt = gboundary_cnt;
            mu=gmu;
            //cout << "boundary cnt = " << boundary_cnt << endl;
            //cout << "norm = \n" << norm << endl;
            //cout << "norm2 = \n" << norm2 << endl;
            pthread_mutex_unlock(&mutex);

            bounded = ctrlr.boundHandler(boundary_cnt,norm,norm2,pos);
        }else{
            bounded = 1;
        }

        if(pre_norm == norm) {
            lineisVisible = false;
        }else{
            cout << " line is visible ! " << endl;
            lineisVisible = true;
        }
        pre_norm = norm;


        // --------------------------------------------------------------------
        // get landing and direction-------------------------------------------
        // --------------------------------------------------------------------

        //"landing"に対応するデータが来ているかチェック
        if (client.isUpdated("landing")){
                data = client.getData("landing");//データをsio::message::ptrとして取得
                parseLanding(data);//データ抽出用関数に渡す
                std::cout << "landing=" << landing << std::endl;
                client.sendData("px_ready","Manager", makePxReady(false));
                cout << "----- Landing -----" << endl;
                cout << "  landing phenox...     " << endl;
                pxset_operate_mode(PX_DOWN);
                exit(0);
        }
        //"direction"に対応するデータが来ているかチェック
        if (client.isUpdated("direction")){
                data = client.getData("direction");//データをsio::message::ptrとして取得
                parseDirection(data);//データ抽出用関数に渡す
                std::cout << "direction = [" << direction[0] << ", " << direction[1] << "]" << std::endl;
                if(!lineisVisible || 100*mu[1] < 120) {
                    ctrlr.changeVel(direction, pos);
                }
        }

        // --------------------------------------------------------------------
        // Sample of data sending ---------------------------------------------
        // --------------------------------------------------------------------
        // 送りたいところに移動してね
        Vector2f px_position(100*mu[0], 100*mu[1]);
        cout << "  px_position :" << mu[0] << "," << mu[1] << endl;

        if(bounded == 0){
            cout << "----------send bounce----- " << endl;
            client.sendData("px_bounce", makePxBounce());
        }

        if(msec_cnt % 10 == 0){
            client.sendData("px_position", makePxPosition(px_position.x(), px_position.y(), st.height));
            client.sendData("px_velocity", makePxVelocity(ctrlr.vx(), ctrlr.vy()));
        }

        //宛先指定でデータを送信
        if(msec_cnt % 100 == 0 && !ctrlr.isStarted()){
            client.sendData("px_ready","Manager", makePxReady(true));
        }

        //cout << ctrlr.vx() << "," << ctrlr.vy() << endl;

        // save log
        static ofstream ofs_deg("output_deg");
            ofs_deg << st.degx << "," << st.degy << endl;
        static ofstream ofs_ctl("output_v");
            ofs_ctl << ctrlr.vx() << "," << ctrlr.vy() << "," << norm.x() << "," << norm.y() << endl;
        static ofstream ofs_vision("output_vision");
            ofs_vision << st.vision_tx << "," << st.vision_ty << "," << input.x() << "," << input.y() << endl;
	      static ofstream ofs_mu("output_mu");
            ofs_mu << mu[0] << "," << mu[1] << "," << mu[2]  << endl;
        // if(!(msec_cnt % 30)){
        //     printf("%.2f %.2f %.2f | %.2f %.2f %.2f | %.2f | \n",st.degx,st.degy,st.degz,st.vision_tx,st.vision_ty,st.vision_tz,st.height);
        // }

        static int hover_cnt = 0;
        static Vector2f origin(0,0);
        if(pxget_operate_mode() == PX_UP){
            pxset_visualselfposition(0, 0);
            hover_cnt = 0;
        }

        if(pxget_operate_mode() == PX_HOVER) {
            if(hover_cnt < 500){
                hover_cnt++;
            }
            else if (hover_cnt == 500) {
                cout << "start control" << endl;
                client.sendData("px_start", makePxStart());
                ctrlr.init(0,0.1,origin,pos);
                hover_cnt++;
            }
            else{
                input = ctrlr.controlStep(pos, dt);
                pxset_visioncontrol_xy(input.x(),input.y());
            }
        }

        static int prev_operatemode = PX_HALT;
        if((prev_operatemode == PX_UP) && (pxget_operate_mode() == PX_HOVER)) {
            origin << st.vision_tx, st.vision_ty;
            pxset_visioncontrol_xy(origin.x(),origin.y());
        }
        prev_operatemode = pxget_operate_mode();

        if(pxget_whisle_detect() == 1) {
            if(pxget_operate_mode() == PX_HOVER) {
                pxset_operate_mode(PX_DOWN);
            }
            else if(pxget_operate_mode() == PX_HALT) {
                pxset_rangecontrol_z(150);
                pxset_operate_mode(PX_UP);
            }
        }
        if(pxget_battery() == 1) {
            client.sendData("px_start", makePxStart());
            timer_disable = 1;
            system("shutdown -h now\n");
            exit(1);
        }

        struct timespec remains;
        _t.tv_nsec += 10000000;
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &_t, &remains);
//        while (get_time() - t < 10.0) {
//            usleep(500);
//        }
        clock_gettime(CLOCK_REALTIME, &_t);
    }
}
