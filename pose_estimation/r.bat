echo off
cls
rem call ndk-build
call ndk-build.cmd
adb push libs/armeabi-v7a/pose_estimation /data/local/tmp
adb shell chmod 777 /data/local/tmp/pose_estimation
adb shell LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/system/lib/egl:/data/local/tmp /data/local/tmp/pose_estimation
adb pull /data/local/tmp/Mag_data.txt .