rem @echo off
adb push libs/armeabi-v7a/pose_estimation_Madgwick /data/local/tmp
adb shell chmod 777 /data/local/tmp/pose_estimation_Madgwick
adb shell LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/system/lib/egl:/data/local/tmp /data/local/tmp/pose_estimation_Madgwick
