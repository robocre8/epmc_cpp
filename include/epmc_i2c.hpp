#ifndef EPMC_I2C_HPP
#define EPMC_I2C_HPP

#include <cstdint>
#include <string>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

inline double round_to_dp(double value, int decimal_places) {
  const double multiplier = std::pow(10.0, decimal_places);
  return std::round(value * multiplier) / multiplier;
}

class EPMC
{
public:
  EPMC() = default;

  void connect(int slave_addr, const std::string device);
  void connect(int slave_addr);
  void disconnect();
  bool connected() const;

  bool writeSpeed(float v0, float v1);
  bool writePWM(int pwm0, int pwm1);
  void readPos(float &pos0, float &pos1);
  void readVel(float &v0, float &v1);
  void readUVel(float &v0, float &v1);
  float getMaxVel(int motor_no);
  bool setCmdTimeout(int timeout_ms);
  int getCmdTimeout();
  bool setPidMode(int mode);
  int getPidMode();
  bool clearDataBuffer();
  void readMotorData(float &pos0, float &pos1, float &v0, float &v1);

private:
    int fd;
    int slaveAddr;
    std::string device;

    uint8_t computeChecksum(uint8_t *packet, uint8_t length);
    void send_packet_without_payload(uint8_t cmd);
    void write_data1(uint8_t cmd, uint8_t pos, float val);
    void write_data2(uint8_t cmd, float val0, float val1);
    void read_data1(float &val0);
    void read_data2(float &val0, float &val1);
    void read_data4(float &val0, float &val1, float &val2, float &val3);

    // Serial Protocol Command IDs -------------
    const uint8_t START_BYTE = 0xAA;
    const uint8_t WRITE_VEL = 0x01;
    const uint8_t WRITE_PWM = 0x02;
    const uint8_t READ_POS = 0x03;
    const uint8_t READ_VEL = 0x04;
    const uint8_t READ_UVEL = 0x05;
    const uint8_t GET_MAX_VEL = 0x14;
    const uint8_t SET_PID_MODE = 0x15;
    const uint8_t GET_PID_MODE = 0x16;
    const uint8_t SET_CMD_TIMEOUT = 0x17;
    const uint8_t GET_CMD_TIMEOUT = 0x18;
    const uint8_t READ_MOTOR_DATA = 0x2A;
    const uint8_t CLEAR_DATA_BUFFER = 0x2C;
    //---------------------------------------------
};

// ----------------------------------------------------


void EPMC::connect(int slave_addr, std::string dev)
{
    slaveAddr = slave_addr;
    device = dev;

    fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Failed to open I2C device");
        exit(1);
    }

    if (ioctl(fd, I2C_SLAVE, slaveAddr) < 0) {
        perror("Failed to set I2C slave address");
        close(fd);
        exit(1);
    }
}

void EPMC::connect(int slave_addr)
{
    slaveAddr = slave_addr;
    device = "/dev/i2c-1";

    fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Failed to open I2C device");
        exit(1);
    }

    if (ioctl(fd, I2C_SLAVE, slaveAddr) < 0) {
        perror("Failed to set I2C slave address");
        close(fd);
        exit(1);
    }
}

void EPMC::disconnect()
{
    close(fd);
}

bool EPMC::connected() const
{
    return (fd >= 0);
}

uint8_t EPMC::computeChecksum(uint8_t *packet, uint8_t length)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++)
        sum += packet[i];
    return sum & 0xFF;
}

void EPMC::send_packet_without_payload(uint8_t cmd)
{
    uint8_t packet[4];
    packet[0] = START_BYTE;
    packet[1] = cmd;
    packet[2] = 0;
    packet[3] = computeChecksum(packet, 3);

    if (write(fd, packet, sizeof(packet)) != sizeof(packet))
        perror("I2C send_packet_without_payload failed");
    usleep(5000);
}

void EPMC::write_data1(uint8_t cmd, uint8_t pos, float val)
{
    // Build packet: start_byte + cmd + length + pos + float + checksum
    uint8_t packet[1 + 1 + 1 + 1 + 4 + 1];
    packet[0] = START_BYTE;
    packet[1] = cmd;
    packet[2] = 5; // msg is uint8 + float = 5byte length
    packet[3] = pos;
    std::memcpy(&packet[4], &val, sizeof(float));
    
    // Compute checksum
    uint8_t checksum = computeChecksum(packet, 8);
    packet[8] = checksum;

    if (write(fd, packet, sizeof(packet)) != sizeof(packet))
        perror("I2C write_data1 failed");
    usleep(5000);
}

void EPMC::write_data2(uint8_t cmd, float val0, float val1)
{
    // Build packet: start_byte + cmd + length + float*4 + checksum
    uint8_t packet[1 + 1 + 1 + 8 + 1];
    packet[0] = START_BYTE;
    packet[1] = cmd;
    packet[2] = 8; // msg is 2 float = 8byte length
    memcpy(&packet[3], &val0, sizeof(float));
    memcpy(&packet[7], &val1, sizeof(float));

    // Compute checksum
    uint8_t checksum = computeChecksum(packet, 11);
    packet[11] = checksum;

    if (write(fd, packet, sizeof(packet)) != sizeof(packet))
        perror("I2C write_data2 failed");
    usleep(5000);
}

void EPMC::read_data1(float &val0)
{
    uint8_t buffer[4];
    if (read(fd, buffer, 4) != 4) {
        perror("I2C read_data1 failed");
        val0 = 0.0f;
        return;
    }
    std::memcpy(&val0, buffer, sizeof(float));
}

void EPMC::read_data2(float &val0, float &val1)
{
    uint8_t buffer[8];
    if (read(fd, buffer, 8) != 8) {
        perror("I2C read_data2 failed");
        // val0 = val1 = 0.0f;
        return;
    }
    std::memcpy(&val0, &buffer[0], 4);
    std::memcpy(&val1, &buffer[4], 4);
}

void EPMC::read_data4(float &val0, float &val1, float &val2, float &val3)
{
    uint8_t buffer[16];
    if (read(fd, buffer, 16) != 16) {
        perror("I2C read_data4 failed");
        // val0 = val1 = val2 = val3 = 0.0f;
        return;
    }
    std::memcpy(&val0, &buffer[0], 4);
    std::memcpy(&val1, &buffer[4], 4);
    std::memcpy(&val2, &buffer[8], 4);
    std::memcpy(&val3, &buffer[12], 4);
}

// ---- High-level public API ----

bool EPMC::writeSpeed(float v0, float v1) {
    float res;
    write_data2(WRITE_VEL, v0, v1);
    read_data1(res);
    return ((int)res == 1);
}

bool EPMC::writePWM(int pwm0, int pwm1) { 
    float res;
    write_data2(WRITE_PWM, (float)pwm0, (float)pwm1);
    read_data1(res);
    return ((int)res == 1);
}

void EPMC::readPos(float &pos0, float &pos1) {
    send_packet_without_payload(READ_POS);
    read_data2(pos0, pos1);
    pos0 = round_to_dp(pos0, 4);
    pos1 = round_to_dp(pos1, 4);
}

void EPMC::readVel(float &v0, float &v1){
    send_packet_without_payload(READ_VEL);
    read_data2(v0, v1);
    v0 = round_to_dp(v0, 4);
    v1 = round_to_dp(v1, 4);
}

void EPMC::readUVel(float &v0, float &v1){
    send_packet_without_payload(READ_UVEL);
    read_data2(v0, v1);
    v0 = round_to_dp(v0, 4);
    v1 = round_to_dp(v1, 4);
}

void EPMC::readMotorData(float &pos0, float &pos1, float &v0, float &v1) {
    send_packet_without_payload(READ_MOTOR_DATA);
    read_data4(pos0, pos1, v0, v1);
    pos0 = round_to_dp(pos0, 4);
    pos1 = round_to_dp(pos1, 4);
    v0 = round_to_dp(v0, 4);
    v1 = round_to_dp(v1, 4);
}

float EPMC::getMaxVel(int motor_no) {
    float max_vel;
    write_data1(GET_MAX_VEL, motor_no, 0.0);
    read_data1(max_vel);
    return max_vel;
}

bool EPMC::setCmdTimeout(int timeout_ms) {
    float res;
    write_data1(SET_CMD_TIMEOUT, 100, (float)timeout_ms);
    read_data1(res);
    return ((int)res == 1);
}

int EPMC::getCmdTimeout() {
    float timeout_ms;
    write_data1(GET_CMD_TIMEOUT, 100, 0.0);
    read_data1(timeout_ms);
    return (int)timeout_ms;
}

bool EPMC::setPidMode(int mode) {
    float res;
    write_data1(SET_PID_MODE, 100, (float)mode);
    read_data1(res);
    return ((int)res == 1);
}

int EPMC::getPidMode() {
    float mode;
    write_data1(GET_PID_MODE, 100, 0.0);
    read_data1(mode);
    return (int)mode;
}

bool EPMC::clearDataBuffer() {
    float res;
    write_data1(CLEAR_DATA_BUFFER, 100, 0.0);
    read_data1(res);
    return ((int)res == 1);
}

#endif