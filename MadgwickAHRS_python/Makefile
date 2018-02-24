orig: MadgwickAHRS.c MadgwickAHRS.h
	swig -python MadgwickAHRS.i
	gcc -fPIC -c MadgwickAHRS.c MadgwickAHRS_wrap.c -I/usr/include/python2.7
	gcc -shared MadgwickAHRS.o MadgwickAHRS_wrap.o -o _MadgwickAHRS.so
	python MadgwickAHRS_run.py
