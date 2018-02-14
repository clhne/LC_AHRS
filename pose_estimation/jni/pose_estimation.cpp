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
    while (ident = ALooper_pollAll(-1, NULL, &events, NULL) >= 0) {
      if (ident == 1) {
        ASensorEvent event;
        while (ASensorEventQueue_getEvents(queue, &event, 1) > 0) {
          if (event.type == ASENSOR_TYPE_GYROSCOPE) {
            printf("gyro(%lld gx=%f, gy=%f, gz=%f)\n", event.timestamp, event.data[0], event.data[1], event.data[2]);
          } else if(event.type == ASENSOR_TYPE_ACCELEROMETER) {
            printf("acc(%lld ax=%f, ay=%f, az=%f)\n", event.timestamp, event.data[0], event.data[1], event.data[2]);
          } else if(event.type == ASENSOR_TYPE_MAGNETIC_FIELD) {
            printf("mag(%lld mx=%f, my=%f, mz=%f)\n", event.timestamp, event.data[0], event.data[1], event.data[2]);
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