echo off
cls
rem call ndk-build
call ndk-build.cmd
adb -s 8a534b70 push libs/armeabi-v7a/pose_estimation /data/local/tmp
adb -s 8a534b70 shell chmod 777 /data/local/tmp/pose_estimation
adb -s 8a534b70 shell LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/system/lib/egl:/data/local/tmp /data/local/tmp/pose_estimation
adb -s 8a534b70 pull /data/local/tmp/Mag_data.txt .