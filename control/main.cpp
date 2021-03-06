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

#include "src/SioClientWrapper.h"
#include "src/Parser.h"//受信データのsio::message::ptrから値を抽出するためのサンプル
#include "src/DataMaker.h"//送信データを生成するためのサンプル


using namespace std;
using namespace cv;
using namespace Eigen;

static void setup_timer();

static char timer_disable = 0;
static void *timer_handler(void *ptr);

pthread_t timer_thread;
pthread_mutex_t mutex;
Vector2f gnorm(0, 0);
Vector2f gnorm2(0, 0);
Vector2f gnorm_start(0, 0);
Vector2f gnorm_start2(0, 0);
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

  // node image server settings ------------------------------------------------
  int client_sockfd;
  int len ;
  struct sockaddr_un address;
  int result ;
  client_sockfd = socket(AF_UNIX,SOCK_STREAM,0);
  address.sun_family = AF_UNIX ;
  strcpy(address.sun_path , "/root/nodejs/projects/imgserver/mysocket");
  len = sizeof(address);
  result = connect(client_sockfd , (struct sockaddr *)&address , len);
  if(result != 0) {exit(-1);}

  int param[]={CV_IMWRITE_JPEG_QUALITY,70};

  long unsigned int buffer_size = 128000;
  unsigned char compressed_buffer_memory[128000];
  unsigned char *compressed_buffer = (unsigned char*)compressed_buffer_memory;
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

      // critical section start--------------------------------------------
      pthread_mutex_lock(&mutex);
      gboundary_cnt = gn;
      gnorm = norm;
      gnorm2 = norm2;
      gnorm_start = norm_start;
      gnorm_start2 = norm_start2;
      gboundary_cnt = gn;
      pthread_mutex_unlock(&mutex);
      // critical section end--------------------------------------------
      //
    if(gn == 1){
        norm_end = norm_start + 100 * norm;
        cvLine(testImage, cvPoint(norm_start.x(), norm_start.y()), cvPoint(norm_end.x(), norm_end.y()), CV_RGB(0, 255, 255), 3, 4 );
    }
    else if(gn == 2){
        norm_end = norm_start + 100 * norm;
        norm_end2 = norm_start2 + 100 * norm2;
        cvLine(testImage, cvPoint(norm_start.x(), norm_start.y()), cvPoint(norm_end.x(), norm_end.y()), CV_RGB(0, 255, 255), 3, 4 );
        cvLine(testImage, cvPoint(norm_start2.x(), norm_start2.y()), cvPoint(norm_end2.x(), norm_end2.y()), CV_RGB(0, 255, 255), 3, 4 );
    }
      if(pxset_imgfeature_query(cameraid) == 1) {
	      ftstate = 1;
      }

      tjhandle tj_compressor = tjInitCompress();
      buffer_size = 128000;
      unsigned char *buffer = (unsigned char*)testImage->imageData;
      
      double encoding_time = get_time();
      if (tjCompress2(tj_compressor, buffer, 320, 0, 240, TJPF_BGR,
                &compressed_buffer, &buffer_size, TJSAMP_420, 30,
                TJFLAG_NOREALLOC) == -1) {
          printf("%s\n", tjGetErrorStr());
      } else {
          encoding_time = get_time() - encoding_time;
          write(client_sockfd, compressed_buffer, buffer_size);
          count++;
      }
      
      tjDestroy(tj_compressor);
      delta = get_time();
      
      if (get_time() - last_time > 3000.0) {
          fps = frames_count * 1000.0/(get_time() - last_time);
          last_time = get_time();
          frames_count = 0;
      }
    }             
  }
  usleep(2000);

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
    client.setEventList(eventList);
    //自身を表す部屋名を設定する(Phenoxなら例えば"Phenox"と決める)
    client.setMyRoom("Phenox");
    //データの送信宛先となる部屋名を設定する(Gameサーバなら例えば"Game")
    client.setDstRoom("Game");
    //URLを指定して接続開始
    client.start("http://ailab-mayfestival2016-base2.herokuapp.com");

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
        static Vector2f input(0,0);
        
        bool bounded = false;

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
            //cout << "boundary cnt = " << boundary_cnt << endl;
            //cout << "norm = \n" << norm << endl;
            //cout << "norm2 = \n" << norm2 << endl;
            pthread_mutex_unlock(&mutex);

            bounded = ctrlr.boundHandler(boundary_cnt,norm,norm2,pos);
        }

        // --------------------------------------------------------------------
        // get landing and direction-------------------------------------------
        // --------------------------------------------------------------------
       
        //"landing"に対応するデータが来ているかチェック
        if (client.isUpdated("landing")){
                data = client.getData("landing");//データをsio::message::ptrとして取得
                parseLanding(data);//データ抽出用関数に渡す
                std::cout << "landing=" << landing << std::endl;
                cout << "----- Game Over -----" << endl;
                cout << "  landing...     " << endl;
                timer_disable = 1;
                exit(0);
        }
        //"direction"に対応するデータが来ているかチェック
        if (client.isUpdated("direction")){
                data = client.getData("direction");//データをsio::message::ptrとして取得
                parseDirection(data);//データ抽出用関数に渡す
                std::cout << "direction = [" << direction[0] << ", " << direction[1] << "]" << std::endl;
                ctrlr.changeVel(direction, pos)
        }

        // --------------------------------------------------------------------
        // Sample of data sending ---------------------------------------------
        // --------------------------------------------------------------------
        // 送りたいところに移動してね
        Vector2f px_position(0, 0);

	if(msec_cnt % 10 == 0){
        if(bounded){
            client.sendData("px_bounce", makePxBounce());
        }
		client.sendData("px_position", makePxPosition(px_position.x(), px_position.y()));
		client.sendData("px_velocity", makePxVelocity(ctrlr.vx(), ctrlr.vy());
	}
        


        //cout << ctrlr.vx() << "," << ctrlr.vy() << endl;

        // save log
        static ofstream ofs_deg("output_deg");
            ofs_deg << st.degx << "," << st.degy << endl;
        static ofstream ofs_ctl("output_v");
            ofs_ctl << ctrlr.vx() << "," << ctrlr.vy() << "," << norm.x() << "," << norm.y() << endl;
        static ofstream ofs_vision("output_vision");
            ofs_vision << st.vision_tx << "," << st.vision_ty << "," << input.x() << "," << input.y() << endl;

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
                ctrlr.init(0,0,origin,pos);
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
            client.sendData("px_start", makePxStart());//
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
