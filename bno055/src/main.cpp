#include <iostream>
#include "unistd.h"
#include <stdint.h>
#include <string>
#include <fcntl.h>
#include <cmath>

extern "C" {
#include "bno055.h"
#include <linux/i2c-dev.h>
#include "smbus_functions.h"
}

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int8MultiArray.h"

using std::isnan;

bno055_t bno055{};
int fd = -1;
const uint8_t address = 0x28;

int8_t write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    auto status = _i2c_smbus_write_i2c_block_data(fd, reg_addr, cnt, reg_data);
    if (status == -1) return BNO055_ERROR;
    else return BNO055_SUCCESS;
}

int8_t read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    auto read = _i2c_smbus_read_i2c_block_data(fd, reg_addr, cnt, reg_data);
    if (read != cnt) return BNO055_ERROR;
    else return BNO055_SUCCESS;
}

void print_acc_offsets() {
    bno055_accel_offset_t acc;
    bno055_read_accel_offset(&acc);
    std::cout << "x: " << acc.x << std::endl;
    std::cout << "y: " << acc.y << std::endl;
    std::cout << "z: " << acc.z << std::endl;
    std::cout << "r: " << acc.r << std::endl;
    std::cout << std::endl;
}

void print_gyro_offsets() {
    bno055_gyro_offset_t gyro;
    bno055_read_gyro_offset(&gyro);
    std::cout << "x: " << gyro.x << std::endl;
    std::cout << "y: " << gyro.y << std::endl;
    std::cout << "z: " << gyro.z << std::endl;
    std::cout << std::endl;
}

void print_mag_offsets() {
    bno055_mag_offset_t mag;
    bno055_read_mag_offset(&mag);
    std::cout << "x: " << mag.x << std::endl;
    std::cout << "y: " << mag.y << std::endl;
    std::cout << "z: " << mag.z << std::endl;
    std::cout << std::endl;
}

void bno_init() {
    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) exit(1);
    if (ioctl(fd, I2C_SLAVE, address) < 0) exit(1);

    bno055.bus_write = write;
    bno055.bus_read = read;
    bno055.delay_msec = (void (*) (uint32_t)) usleep;
    bno055.dev_addr = address;

    bno055_init(&bno055);
    bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
    //bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF_FMC_OFF);
    //bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
    //bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
    //bno055_set_euler_unit(BNO055_EULER_UNIT_RAD);
    //bno055_set_tilt_unit(0x1);
    //bno055_set_data_output_format(0x1);
}

void set_diagonal_mat(boost::array<double, 9>& mat, double value) {
    for (int i = 0; i < 9; i++) {
	mat[i] = 0;
    }
    mat[0] = value;
    mat[4] = value;
    mat[8] = value;
}

int main(int argc, char **argv){
    bno_init();

    ros::init(argc, argv, "bno055");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Publisher calib_pub = n.advertise<std_msgs::Int8MultiArray>("calib", 1);

    ros::Rate loop_rate(100);
    uint32_t seq = 0;

    sensor_msgs::Imu imu_msg;
    set_diagonal_mat(imu_msg.orientation_covariance, 1);
    set_diagonal_mat(imu_msg.angular_velocity_covariance, 9.50420e-07);
    set_diagonal_mat(imu_msg.linear_acceleration_covariance, 2.84408e-04);

    std_msgs::Int8MultiArray calib_msg;
    calib_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    calib_msg.layout.dim[0].size = 3;
    calib_msg.layout.dim[0].stride = 1;
    calib_msg.layout.dim[0].label = "mag, gyro e acc";

    while (ros::ok()) {
	imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_frame";
        imu_msg.header.seq = seq;

//	Le e publica quaternion
	bno055_quaternion_t quat{};
	if (bno055_read_quaternion_wxyz(&quat)
		!= BNO055_SUCCESS) continue;
	if (isnan(quat.w) || isnan(quat.x) || isnan(quat.y) || isnan(quat.z))
		continue;
	const double scale = (1.0 / (1 << 14));
	imu_msg.orientation.w = scale * quat.w;
	imu_msg.orientation.x = scale * quat.x;
	imu_msg.orientation.y = scale * quat.y;
	imu_msg.orientation.z = scale * quat.z;

//	Le e publica velocidade angular
	bno055_gyro_double_t angular_vel{};
	if (bno055_convert_double_gyro_xyz_rps(&angular_vel)
		!= BNO055_SUCCESS) continue;
	if (isnan(angular_vel.x) || isnan(angular_vel.y) || isnan(angular_vel.z))
		continue;
	imu_msg.angular_velocity.x = angular_vel.x;
	imu_msg.angular_velocity.y = angular_vel.y;
	imu_msg.angular_velocity.z = angular_vel.z;

//	Le e publica aceleracao
	bno055_linear_accel_double_t accel{};
	if (bno055_convert_double_linear_accel_xyz_msq(&accel)
		!= BNO055_SUCCESS) continue;
	if (isnan(accel.x) || isnan(accel.y) || isnan(accel.z))
		continue;
	imu_msg.linear_acceleration.x = accel.x;
	imu_msg.linear_acceleration.y = accel.y;
	imu_msg.linear_acceleration.z = accel.z;

	//print_acc_offsets();

	//bno055_gravity_double_t gravity;
	//if (bno055_convert_double_gravity_xyz_msq(&gravity)
	//	!= BNO055_SUCCESS) continue;
	//imu_msg.linear_acceleration.x = gravity.x;
	//imu_msg.linear_acceleration.y = gravity.y;
	//imu_msg.linear_acceleration.z = gravity.z;

	imu_pub.publish(imu_msg);

//	Publica nível de calibração (1 a 3)
	uint8_t mag_calib;
	bno055_get_mag_calib_stat(&mag_calib);
	uint8_t accel_calib;
	bno055_get_accel_calib_stat(&accel_calib);
	uint8_t gyro_calib;
	bno055_get_gyro_calib_stat(&gyro_calib);
	calib_msg.data.clear();
	calib_msg.data.push_back(mag_calib);
	calib_msg.data.push_back(gyro_calib);
	calib_msg.data.push_back(accel_calib);
	calib_pub.publish(calib_msg);

	ros::spinOnce();
	loop_rate.sleep();
	seq++;
    }
}
