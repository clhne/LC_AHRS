from __future__ import print_function
import MadgwickAHRS

ax = 0.
ay = 2.
az = 5.

gx = -6.2
gy = 7.8
gz = 0.

mx = 0.0
my = 0.
mz = 24.

MadgwickAHRS.MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz)
cv = MadgwickAHRS.cvar
q0, q1, q2, q3 = cv.q0, cv.q1, cv.q2, cv.q3
print(q0, q1, q2, q3)
