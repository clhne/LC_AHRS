#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <android/sensor.h>
#include <android/looper.h>
#include <android/log.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <MahonyAHRS.h>
#include <fstream>
using namespace std;
uint64_t performanceCounter() {
  uint64_t result = 0;
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  result = (uint64_t)ts.tv_sec * 1000000000LL + (uint64_t)ts.tv_nsec;
  return result;
}

uint64_t performanceFrequency() {
  uint64_t result = 1;
  result = 1000000000LL;
  return result;
}

double getDurationSec(uint64_t prev) {
  return (double)(performanceCounter() - prev)/performanceFrequency();
}

void* SENSOR_WORKER_THREAD(void* param) {
  int priority = getpriority(PRIO_PROCESS, 0);
  printf("sensor work thread old_priority = %d\n", priority);
  setpriority(PRIO_PROCESS, 0, -20); // ANDROID_PRIORITY_URGENT_DISPLAY = -8
  //set_sched_policy(0, 1); // SP_FOREGROUND = 1
  priority = getpriority(PRIO_PROCESS, 0);
  printf("sensor work thread new_priority = %d\n", priority);
  // if android, create thread to recieve sensor data
  ALooper* looper = ALooper_forThread();
  if (looper == NULL) {
    looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
  }
  assert(looper);
  ASensorManager* mgr = ASensorManager_getInstance();
  assert(mgr);
  ASensorEventQueue* queue = ASensorManager_createEventQueue(mgr, looper, 1, NULL, NULL);
  assert(queue);
  const ASensor* gyro = ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_GYROSCOPE);
  const ASensor* acc = ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_ACCELEROMETER);
  const ASensor* mag = ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_MAGNETIC_FIELD);
  if (gyro && acc && mag) {
    ASensorEventQueue_enableSensor(queue, gyro);
    ASensorEventQueue_enableSensor(queue, acc);
    ASensorEventQueue_enableSensor(queue, mag);
    ASensorEventQueue_setEventRate(queue, gyro, ASensor_getMinDelay(gyro));
    ASensorEventQueue_setEventRate(queue, acc, ASensor_getMinDelay(acc));
    ASensorEventQueue_setEventRate(queue, mag, ASensor_getMinDelay(mag));
    int ident, events;
    int accCounter = 0;
    int gyroCounter = 0;
    int magCounter =0;
    bool a_ready = false;
    bool m_ready = false;
    double gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0, mx = 0, my = 0, mz = 0;
    double timestep;
    Mahony pose;
    ofstream fout;
    fout.open("/data/local/tmp/Mag_data.txt");
    while (ident = ALooper_pollAll(-1, NULL, &events, NULL) >= 0) {
      if (ident == 1) {
        ASensorEvent event;
        long long int timestep_begin = event.timestamp;
        while(ASensorEventQueue_getEvents(queue, &event, 1) > 0){
            if(event.type == ASENSOR_TYPE_GYROSCOPE){
                /*
                gx = 0.008514;
                gy = -0.004257;
                gz = 0.025541;
                */
                gx = event.data[0];
                gy = event.data[1];
                gz = event.data[2];
            //    printf("gyro(%lld gx=%f, gy=%f, gz=%f)\n", event.timestamp, gx, gy, gz);
            accCounter ++;
            if(accCounter >= 0 && a_ready && m_ready){
                long long int timestep_end_g = event.timestamp;
                timestep = (timestep_end_g - timestep_begin)/10000000.0f;
                
                if(timestep >= 0){
                    //printf("gyro timestep = %lf\n",timestep );
                //    pose.update(-gy, -gx, gz, ay, ax, az, mx, my, mz, timestep); //LC1860 axes
                    pose.update(-gy, -gx, -gz, -ay, -ax, -az, mx, my, mz, timestep);
                    double roll = pose.getRoll();
                    double pitch = pose.getPitch();
                    double yaw = pose.getYaw();
                    printf("roll = %lf,pitch = %lf,yaw = %lf\n",roll, pitch, yaw);
                //    sleep(1);
                }
            }
            }
            else if(event.type == ASENSOR_TYPE_ACCELEROMETER){
                /*
                ax = -5.765238;
                ay = -3.926491;
                az = 7.077260;
                */
                ax = event.data[0];
                ay = event.data[1];
                az = event.data[2];
                a_ready = true;
        //        printf("acc(%lld ax=%f, ay=%f, az=%f)\n", event.timestamp, ax, ay, az);
                long long int timestep_end_a = event.timestamp/10000000;
            //    pose.update(gx, gy, gz, ax, ay, az, mx, my, mz, timestep_end_a);
                //printf("acc(%lld ax = %f,ay=%f,az=%f)\n",event.timestamp,event.acceleration.x,event.acceleration.y,event.acceleration.z);
            }
            else if(event.type == ASENSOR_TYPE_MAGNETIC_FIELD){
                
                /*
                mx = 46.800003;
                my = 28.800001;
                mz = 68.400002;
                */
                mx = event.data[0];
                my = event.data[1];
                mz = event.data[2];
                m_ready = true;
        //        printf("mag(%lld mx=%f, my=%f, mz=%f)\n",event.timestamp, mx, my, mz);
                long long int timestep_end_m = event.timestamp;//10000000;
                //pose.update(gx, gy, gz, ax, ay, az, mx, my, mz, timestep_end_m);
             
             fout<<mx<<" \t"<<my<<" \t"<<mz<<endl;
            }
        }
      }
    }
  } else {
    printf("start sensor thread failed, no gyroscope or ACCELEROMETER or MAGNETIC_FIELD is found.\n");
  }
}

int main(int argc, char* argv[]) {
  pthread_t thread_id;
  pthread_create(&thread_id, NULL, &SENSOR_WORKER_THREAD, NULL);
  pthread_join(thread_id, NULL);

  return 0;
}