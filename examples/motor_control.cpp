
#include <sstream>
#include <iostream>
#include <unistd.h>

#include <chrono>

#include <iomanip>

#include "epmc.hpp"

EPMC epmc;

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

int main(int argc, char **argv)
{
  // variable for communication
  bool success; float val0, val1, val2, val3;

  // [4 rev/sec, 2 rev/sec, 1 rev/sec, 0.5 rev/sec]
  float targetVel[] = {1.571, 3.142, 6.284, 12.568}; // in rad/sec
  float vel = targetVel[1]; // in rad/sec
  float v = 0.0;

  float pos0, pos1, pos2, pos3;
  float vel0, vel1, vel2, vel3;

  auto cmdTime = std::chrono::system_clock::now();
  std::chrono::duration<double> cmdDuration;
  float cmdTimeInterval = 5.0;

  auto readTime = std::chrono::system_clock::now();
  std::chrono::duration<double> readDuration;
  float readTimeInterval = 0.01; // 100Hz

  std::string port = "/dev/ttyACM0";
  // std::string port = "/dev/ttyUSB0";
  epmc.connect(port);

  for (int i=0; i<4; i+=1){
    delay_ms(1000);
    std::cout << "waiting for epmc controller: " << i+1 << " sec" << std::endl;
  }
  

  success = epmc.clearDataBuffer();
  epmc.writeSpeed(v, v);

  int motor_cmd_timeout_ms = 10000;
  epmc.setCmdTimeout(motor_cmd_timeout_ms); // set motor command timeout
  std::tie(success, val0) = epmc.getCmdTimeout();
  if (success) {
    motor_cmd_timeout_ms = val0;
    std::cout << "motor command timeout: " << motor_cmd_timeout_ms << " ms" << std::endl;
  } else {
    std::cerr << "ERROR: could not read motor command timeout" << std::endl;
  }

  bool sendHigh = true;

  cmdTime = std::chrono::system_clock::now();
  readTime = std::chrono::system_clock::now();

  while (true)
  {

    cmdDuration = (std::chrono::system_clock::now() - cmdTime);
    if (cmdDuration.count() > cmdTimeInterval)
    {
      if (sendHigh)
      {
        v = vel;
        epmc.writeSpeed(v, v);
        vel *= -1;
        sendHigh = false;
      }
      else
      {
        v = 0.0;
        epmc.writeSpeed(v, v);
        sendHigh = true;
      }

      cmdTime = std::chrono::system_clock::now();
    }

    readDuration = (std::chrono::system_clock::now() - readTime);
    if (readDuration.count() > readTimeInterval)
    {

      // epmc.writeSpeed(v, v);
      std::tie(success, val0, val1, val2, val3) = epmc.readMotorData();
      if (success) { // only update if read was successfull
        pos0 = val0; pos1 = val1;
        vel0 = val2; vel1 = val3;
      }
      std::cout << "----------------------------------" << std::endl;
      std::cout << "motor0_readings: [" << pos0 << "," << vel0 << "]" << std::endl;
      std::cout << "motor1_readings: [" << pos1 << "," << vel1 << "]" << std::endl;
      std::cout << "----------------------------------" << std::endl;
      std::cout << std::endl;

      readTime = std::chrono::system_clock::now();
    }
  }
}