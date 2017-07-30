// #define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <strings.h>
#include <math.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "quaternion.c"

// Common defs {{{
void print_hex (uint8_t * buf, size_t len) {
	for (size_t i = 0; i < len; i++)
		printf("%02x ", buf[i]);
	}

// }}}
// Matt's I2C Register Library {{{

// Call this function first to initialize an i2c bus
// It will give you a fd handle to use to talk to the bus
int i2c_init (int i2c_bus_id) {
	char filename[20];
	sprintf(filename, "/dev/i2c-%d", i2c_bus_id);
	int fd = open(filename, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Failed to open I2C bus #%d.  Are you sure it exists?\n", i2c_bus_id);
		exit(1);
	} else {
		return fd;
	}
}

// This internal function reconfigures the fd within the kernel
// Use it to change which slave we want to talk to on the i2c bus
void _i2c_choose_slave (int fd, int sad) {
	// Switch this to I2C_SLAVE_FORCE if you want to aggressively seize control of the bus
	if (ioctl(fd, I2C_SLAVE_FORCE, sad) < 0) {
		fprintf(stderr, "Tried to configure fd %d to talk to slave 0x%x, but the ioctl operation failed.\n", fd, sad);
	}
}

// write one byte directly to specified slave on specified fd
void i2c_write_slave (int fd, int sad, uint8_t value) {
	_i2c_choose_slave(fd, sad);

  if(i2c_smbus_write_byte(fd, value) == -1) {
		fprintf(stderr, "Tried to write one byte to the i2c slave at address 0x%x via fd %d, but the smbus write operation failed.\n", sad, fd);
		exit(1);
	}
}

// write one byte into specified register of specified slave on specified fd
void i2c_write_slave_reg (int fd, int sad, uint8_t reg, uint8_t value) {
	_i2c_choose_slave(fd, sad);

  if(i2c_smbus_write_byte_data(fd, reg, value) == -1) {
		fprintf(stderr, "Tried to write one byte into register 0x%x of the i2c slave at address 0x%x via fd %d, but the smbus write operation failed.\n", reg, sad, fd);
		exit(1);
	}
}

// read <size> bytes from specified register of specified slave from specified fd into buf
void i2c_read_slave_reg (int fd, int sad, uint8_t reg, uint8_t size, uint8_t * buf) {
	_i2c_choose_slave(fd, sad);

	// Original version (didn't work with LSM6DS33):
	// if (i2c_smbus_read_i2c_block_data(fd, 0x80 | reg, size, buf) != size) {
	if (i2c_smbus_read_i2c_block_data(fd, reg, size, buf) != size) {
		fprintf(stderr, "Tried to read %d byte(s) from register 0x%x of the i2c slave at address 0x%x via fd %d, but the smbus block-read operation failed.\n", size, reg, sad, fd);
		exit(1);
	}
}

// }}}
// Matt's New BNO055-via-TCA9548A Library {{{

typedef struct {
	int i2c_bus_id;
	int tca9548a_sad;
	uint8_t tca9548a_idx;
	int bno055_sad;
} tcabno_t;

void _tcabno_readnreg(tcabno_t imu, uint8_t reg, uint8_t size, uint8_t * buf) {
	//printf("um\n");
	//int fd = i2c_init(imu.i2c_bus_id);
	////i2c_write_slave_reg(fd, imu.tca9548a_sad, 0, ((uint8_t) (1 << imu.tca9548a_idx)));
	////close(fd);
	//char muxbuf = 1;
	//struct i2c_msg muxmsg = { (__u16) 0x70, (unsigned short) 0, (short) 1, (char *) &muxbuf };
	//struct i2c_rdwr_ioctl_data muxrdwr= { (struct i2c_msg *) &muxmsg, (int) 1 };
	//ioctl(fd, I2C_RDWR, muxrdwr);
	//printf("yes\n");
	//close(fd);

	//char * cmdstr;
	//asprintf(&cmdstr, "i2cset -y %d 0x%02x 0 %u", imu.i2c_bus_id, imu.tca9548a_sad, ((uint8_t) (1 << imu.tca9548a_idx)));
	//system(cmdstr);
	//free(cmdstr);
	char * devfn;
	asprintf(&devfn, "/dev/i2c-%d", imu.i2c_bus_id);
	int fd = open(devfn, O_RDWR);
	free(devfn);
	ioctl(fd, I2C_SLAVE, imu.tca9548a_sad);
	int res = i2c_smbus_write_byte_data(fd, 0, ((uint8_t) (1 << imu.tca9548a_idx)));
	close(fd);

	fd = i2c_init(imu.i2c_bus_id);
	ioctl(fd, I2C_PEC, 0);
	i2c_read_slave_reg(fd, imu.bno055_sad, reg, size, buf);
	close(fd);
}

uint8_t _tcabno_read1reg(tcabno_t imu, uint8_t reg) {
	uint8_t value;
	_tcabno_readnreg(imu, reg, 1, &value);
	return value;
}

void _tcabno_writereg(tcabno_t imu, uint8_t reg, uint8_t value) {
	//printf("um\n");
	//int fd = i2c_init(imu.i2c_bus_id);
	////i2c_write_slave_reg(fd, imu.tca9548a_sad, 0, ((uint8_t) (1 << imu.tca9548a_idx)));
	////close(fd);
	//char muxbuf = 1;
	//struct i2c_msg muxmsg = { (__u16) 0x70, (unsigned short) 0, (short) 1, (char *) &muxbuf };
	//struct i2c_rdwr_ioctl_data muxrdwr= { (struct i2c_msg *) &muxmsg, (int) 1 };
	//ioctl(fd, I2C_RDWR, muxrdwr);
	//printf("yes\n");
	//close(fd);

	//char * cmdstr;
	//asprintf(&cmdstr, "i2cset -y %d 0x%02x 0 %u", imu.i2c_bus_id, imu.tca9548a_sad, ((uint8_t) (1 << imu.tca9548a_idx)));
	//system(cmdstr);
	//free(cmdstr);

	char * cmdstr;
	asprintf(&cmdstr, "i2cset -y %d 0x%02x 0 %u", imu.i2c_bus_id, imu.tca9548a_sad, ((uint8_t) (1 << imu.tca9548a_idx)));
	system(cmdstr);
	free(cmdstr);

	int fd = i2c_init(imu.i2c_bus_id);
	i2c_write_slave_reg(fd, imu.bno055_sad, reg, value);
	close(fd);
}

tcabno_t tcabno_init (int i2c_bus_id, int tca9548a_sad, uint8_t tca9548a_idx, int bno055_sad) {
	//printf("Bringing up a bno055 at %01x:%02x:%01x:%02x...",
	//	i2c_bus_id, tca9548a_sad, tca9548a_idx, bno055_sad);
	printf("Bringing up a bno055 at %01x:%01x...",
		i2c_bus_id, tca9548a_idx);
	tcabno_t imu;
	imu.i2c_bus_id = i2c_bus_id;
	imu.tca9548a_sad = tca9548a_sad;
	imu.tca9548a_idx = tca9548a_idx;
	imu.bno055_sad = bno055_sad;

	printf("  Success. Chip=%02x, Acc=%02x, Mag=%02x, Gyro=%02x\n",
		_tcabno_read1reg(imu, 0x00),
		_tcabno_read1reg(imu, 0x01),
		_tcabno_read1reg(imu, 0x02),
		_tcabno_read1reg(imu, 0x03)
	);

	_tcabno_writereg(imu, 0x3d, 0x00); // set opmode conf
	//_tcabno_writereg(imu, 0x3f, 0x20); // set sys trigger
	_tcabno_writereg(imu, 0x3e, 0x00); // pwr mode normal
	_tcabno_writereg(imu, 0x3d, 0x0c); // set opmode ndof
	_tcabno_writereg(imu, 0x07, 0x00); // set page id reg
	_tcabno_writereg(imu, 0x3f, 0x00); // rst sys trigger

	return imu;
}

quaternion_t tcabno_sample (tcabno_t imu) {
	uint8_t quatbuf[8];
	bzero(quatbuf, 8);
	_tcabno_readnreg(imu, 0x20, 8, (uint8_t *) &quatbuf);
	
	int16_t x, y, z, w;
	w = (((uint16_t) quatbuf[1]) << 8) | ((uint16_t) quatbuf[0]);
	x = (((uint16_t) quatbuf[3]) << 8) | ((uint16_t) quatbuf[2]);
	y = (((uint16_t) quatbuf[5]) << 8) | ((uint16_t) quatbuf[4]);
	z = (((uint16_t) quatbuf[7]) << 8) | ((uint16_t) quatbuf[6]);

	double c = (1.0 / (1 << 14)); // conversion factor
	
	quaternion_t quat = {c*w, c*x, c*y, c*z};

	return quat;
}

quaternion_t tcabno_safesample (tcabno_t imu) {
	quaternion_t sample = tcabno_sample(imu);
	double mag = quaternion_mag(sample);
	while (! (0.98 < mag && mag < 1.02)) {
		sample = tcabno_sample(imu); // resample
		mag = quaternion_mag(sample);
	}

	return sample;
}

// }}}
// Matt's New BNO055-direct Library {{{

typedef struct {
	int i2c_bus_id;
	int bno055_sad;
} bno_t;

bno_t bno_init(int i2c_bus_id, int bno055_sad) {
	bno_t bno = {i2c_bus_id, bno055_sad};
	int fd = i2c_init (i2c_bus_id);

	i2c_write_slave_reg(fd, bno055_sad, 0x3d, 0x00); // set opmode conf
	i2c_write_slave_reg(fd, bno055_sad, 0x3e, 0x00); // pwr mode normal
	i2c_write_slave_reg(fd, bno055_sad, 0x3d, 0x0c); // set opmode ndof
	i2c_write_slave_reg(fd, bno055_sad, 0x07, 0x00); // set page id reg
	i2c_write_slave_reg(fd, bno055_sad, 0x3f, 0x00); // rst sys trigger

	close(fd);

	return bno;
}

quaternion_t bno_sample(bno_t bno) {
	uint8_t quatbuf[8];
	bzero(quatbuf, 8);

	int fd = i2c_init (bno.i2c_bus_id);
	i2c_read_slave_reg(fd, bno.bno055_sad, 0x20, 8, (uint8_t *) &quatbuf);
	close(fd);
	
	int16_t x, y, z, w;
	w = (((uint16_t) quatbuf[1]) << 8) | ((uint16_t) quatbuf[0]);
	x = (((uint16_t) quatbuf[3]) << 8) | ((uint16_t) quatbuf[2]);
	y = (((uint16_t) quatbuf[5]) << 8) | ((uint16_t) quatbuf[4]);
	z = (((uint16_t) quatbuf[7]) << 8) | ((uint16_t) quatbuf[6]);

	double c = (1.0 / (1 << 14)); // conversion factor
	
	quaternion_t quat = {c*w, c*x, c*y, c*z};
	return quat;
}

quaternion_t bno_safesample (bno_t bno) {
	quaternion_t sample = bno_sample(bno);
	double mag = quaternion_mag(sample);
	while (! (0.98 < mag && mag < 1.02)) {
		sample = bno_sample(bno); // resample
		mag = quaternion_mag(sample);
	}

	return sample;
}


// }}}
// Convenience functions {{{

void print_sample(quaternion_t q) {
	printf("% 1.5f, % 1.5f, % 1.5f, % 1.5f", q.w, q.x, q.y, q.z);
}
// }}}
// // Main {{{
// 
// int main () {
// 	tcabno_t imu0 = tcabno_init(1, 0x70, 0, 0x29);
// 	tcabno_t imu1 = tcabno_init(1, 0x70, 1, 0x29);
// 	tcabno_t imu2 = tcabno_init(1, 0x70, 2, 0x29);
// 
// 	while (1) {
// 		quaternion_t quat0 = tcabno_sample(imu0);
// 		quaternion_t quat1 = tcabno_sample(imu1);
// 		quaternion_t quat2 = tcabno_sample(imu2);
// 
// 		print_sample(quat0);
// 		printf("      ");
// 		print_sample(quat1);
// 		printf("      ");
// 		print_sample(quat2);
// 		printf("\n");
// 
// 		//printf("X: %02x%02x\n", quatbuf[3], quatbuf[2]);
// 		//uint8_t tempbuf[2];
// 		//i2c_read_slave_reg(fd, 0x29, 0x1a, 2, (uint8_t *) &tempbuf);
// 		//printf("Temp: %d\n", *((int16_t *) tempbuf));
// 
// 		//usleep(20000);
// 	}
// 
// 	return 0;
// }
// 
// // }}}
