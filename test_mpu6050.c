#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>

int main()
{
	char buffer[128];
	int in, out;
	int nread;
	unsigned int x_accel, y_accel, z_accel;
	unsigned int x_gyro, y_gyro, z_gyro;

	in = open("/dev/MPU6050", O_RDONLY);
	if (!in) {
		printf("ERROR: %d, Open /dev/MPU6050 nod failed.\n", -1);
		return -1;	
	}	
	nread = read(in, buffer, 12);
	if (nread < 0) {
		printf("ERROR: %d, A read error has occurred\n", nread);
		return -1;	
	}
	close(in);
	x_accel = buffer[1] + (buffer[0] << 8);
	y_accel = buffer[3] + (buffer[2] << 8);
	z_accel = buffer[5] + (buffer[4] << 8);	
	x_gyro = buffer[7] + (buffer[6] << 8);
	y_gyro = buffer[9] + (buffer[8] << 8);
	z_gyro = buffer[11] + (buffer[10] << 8);	
	printf("x accel is: %d \n", x_accel);
	printf("y accel is: %d \n", y_accel);
	printf("z accel is: %d \n", z_accel);	
	printf("x gyro is: %d \n", x_gyro);
	printf("y gyro is: %d \n", y_gyro);
	printf("z gyro is: %d \n", z_gyro);

	exit(0);
}
