
#include <sstream>
#include <iostream>
#include <unistd.h>

#include <chrono>

#include <iomanip>

#include "epmc_i2c.hpp"

EPMC epmc;

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

int main(int argc, char **argv)
{

  // [4 rev/sec, 2 rev/sec, 1 rev/sec, 0.5 rev/sec]
  float targetVel[] = {1.571, 3.142, 6.284, 12.568}; // in rad/sec
  float vel = targetVel[1]; // in rad/sec
  float v = 0.0;

  float pos0, pos1;
  float vel0, vel1;

  auto cmdTime = std::chrono::system_clock::now();
  std::chrono::duration<double> cmdDuration;
  float cmdTimeInterval = 5.0;

  auto readTime = std::chrono::system_clock::now();
  std::chrono::duration<double> readDuration;
  float readTimeInterval = 0.015;

  int i2cAddress = 85; // 0x55
  epmc.connect(i2cAddress);
  // std::string device = "/dev/i2c-1";
  // epmc.connect(i2cAddress, device);

  for (int i=0; i<4; i+=1){
    delay_ms(1000);
    std::cout << "waiting for epmc controller: " << i+1 << " sec" << std::endl;
  }
  

  epmc.writeSpeed(v, v);
  epmc.clearDataBuffer();

  int motor_cmd_timeout_ms = 10000;
  epmc.setCmdTimeout(motor_cmd_timeout_ms); // set motor command timeout
  motor_cmd_timeout_ms = epmc.getCmdTimeout();
  std::cout << "motor command timeout: " << motor_cmd_timeout_ms << " ms" << std::endl;

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
        // epmc.writeSpeed(v, v);
        vel *= -1;
        sendHigh = false;
      }
      else
      {
        v = 0.0;
        // epmc.writeSpeed(v, v);
        sendHigh = true;
      }

      cmdTime = std::chrono::system_clock::now();
    }

    readDuration = (std::chrono::system_clock::now() - readTime);
    if (readDuration.count() > readTimeInterval)
    {
      try
      {
        epmc.writeSpeed(v, v);
        epmc.readMotorData(pos0, pos1, vel0, vel1);
        std::cout << "----------------------------------" << std::endl;
        std::cout << "motor0_readings: [" << pos0 << "," << vel0 << "]" << std::endl;
        std::cout << "motor1_readings: [" << pos1 << "," << vel1 << "]" << std::endl;
        std::cout << "----------------------------------" << std::endl;
        std::cout << std::endl;
      }
      catch(const std::exception& e)
      {
        std::cout << "Error occurred: " << e.what() << std::endl;
      }

      readTime = std::chrono::system_clock::now();
    }
  }
}