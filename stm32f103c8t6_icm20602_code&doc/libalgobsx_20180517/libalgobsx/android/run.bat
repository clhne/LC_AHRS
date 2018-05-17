@echo off
adb push libs/armeabi-v7a/test /data/local/tmp
adb shell chmod 777 /data/local/tmp/test
adb shell LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/system/lib/egl:/data/local/tmp /data/local/tmp/test