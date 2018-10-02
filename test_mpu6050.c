#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>

short x_accel, y_accel, z_accel;
short x_gyro, y_gyro, z_gyro;

int main()
{
	char buffer[128];
	short *time;
	int in, out;
	int nread;
	
	in = open("/dev/MPU6050", O_RDONLY);
	if (!in) {
		printf("ERROR: %d, Open /dev/MPU6050 nod failed.\n", -1);
		return -1;	
	}	
	nread = read(in, buffer, 12);
	close(in);	
	if (nread < 0) {
		printf("ERROR: %d, A read error has occurred\n", nread);
		return -1;	
	}

	time = (short*)buffer;
	x_accel = *(time);
	y_accel = *(time + 1);
	z_accel = *(time + 2);
	x_gyro =  *(time + 3);
	y_gyro =  *(time + 4);
	z_gyro =  *(time + 5);
	printf("x accel is: %d \n", x_accel);
	printf("y accel is: %d \n", y_accel);
	printf("z accel is: %d \n", z_accel);	
	printf("x gyro is: %d \n", x_gyro);
	printf("y gyro is: %d \n", y_gyro);
	printf("z gyro is: %d \n", z_gyro);

	exit(0);
}
