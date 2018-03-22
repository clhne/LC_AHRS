echo off
cls
rem call ndk-build
call ndk-build.cmd
adb -s 0123456789ABCDEF push libs/armeabi-v7a/pose_estimation /data/local/tmp
adb -s 0123456789ABCDEF shell chmod 777 /data/local/tmp/pose_estimation
adb -s 0123456789ABCDEF shell LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/system/lib/egl:/data/local/tmp /data/local/tmp/pose_estimation
adb -s 0123456789ABCDEF pull /data/local/tmp/Mag_data.txt .