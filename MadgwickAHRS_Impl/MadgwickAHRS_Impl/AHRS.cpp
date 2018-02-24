#include "AHRS.h"
//#include "MadgwickAHRS.h"
#include <math.h>
#include "GKEARHS.h"
#define pi 3.141592653589793
#define quaternion_h
#include "quaternion.h"
#include "madgwick_internal_report.h"

typedef struct student {
	float gx, gy, gz, ax, ay, az, mx, my, mz, time;
} stu;

int main(int argc, char* argv[])
{
	//���ļ� 
	FILE * r = fopen("SampleData.txt", "r");
	assert(r != NULL);
	FILE * w = fopen("A.txt", "w");
	assert(w != NULL);
	FILE * quaternion = fopen("quaternion.txt", "w");
	assert(quaternion != NULL);

	q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;  // quaternion elements representing the estimated orientation
	
	//��д�ļ� 
	stu a[10000];
	int i = 0;
	while (fscanf(r, "%f%f%f%f%f%f%f%f%f%f", &a[i].gx, &a[i].gy, &a[i].gz, &a[i].ax, &a[i].ay, &a[i].az, &a[i].mx, &a[i].my, &a[i].mz, &a[i].time) != EOF)
	{
		//�������ʾ����Ļ 
		//printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", a[i].gx, a[i].gy, a[i].gz, a[i].ax, a[i].ay, a[i].az, a[i].mx, a[i].my, a[i].mz, a[i].time);
		
		//������ļ�A.txt 
		fprintf(w, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", a[i].gx, a[i].gy, a[i].gz, a[i].ax, a[i].ay, a[i].az, a[i].mx, a[i].my, a[i].mz, a[i].time);
		//�������ʾ����Ļ 
		//printf("%f\t%f\t%f\t%f\t%f\n", q0, q1, q2, q3, a[i].time);

		

		//�����ǵ������Ҫ������ת��Ϊ����
		a[i].gx = a[i].gx * pi / 180;
		a[i].gy = a[i].gy * pi / 180;
		a[i].gz = a[i].gz * pi / 180;
		//MadgwickAHRSupdate(a[i].gx, a[i].gy, a[i].gz, a[i].ax, a[i].ay, a[i].az, a[i].mx, a[i].my, a[i].mz);
		//DoMadgwickAHRS(a[i].gx, a[i].gy, a[i].gz, a[i].ax, a[i].ay, a[i].az, a[i].mx, a[i].my, a[i].mz);
		filterUpdate(a[i].gx, a[i].gy, a[i].gz, a[i].ax, a[i].ay, a[i].az, a[i].mx, a[i].my, a[i].mz);


		//������ļ�quaternion.txt 
		fprintf(quaternion, "%f\t%f\t%f\t%f\t%f\n", q0, q1, q2, q3, a[i].time);
		i++;
	}

	//�ر��ļ� 
	fclose(r);
	fclose(w);
	fclose(quaternion); 

	system("pause");
	return 0;
}