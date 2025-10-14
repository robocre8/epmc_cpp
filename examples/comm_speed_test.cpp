
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

  bool sendHigh = false;

  float lowTargetVel = -10.00; // in rad/sec
  float highTargetVel = 10.00; // in rad/sec

  float p0, p1, v0, v1;

  auto prevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> duration;
  auto currentTime = std::chrono::system_clock::now();

  // std::string port = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0";
  std::string port = "/dev/ttyUSB0";
  epmc.connect(port);

  for (int i=0; i<4; i+=1){
    delay_ms(1000);
    std::cout << "configuring controller: " << i+1 << " sec" << std::endl;
  }
  

  epmc.writeSpeed(0.0, 0.0);
  epmc.clearDataBuffer();

  int motor_cmd_timeout_ms = 5000;
  epmc.setCmdTimeout(motor_cmd_timeout_ms); // set motor command timeout
  motor_cmd_timeout_ms = epmc.getCmdTimeout();
  std::cout << "motor command timeout: " << motor_cmd_timeout_ms << " ms" << std::endl;

  epmc.writeSpeed(lowTargetVel, lowTargetVel);

  sendHigh = true;

  prevTime = std::chrono::system_clock::now();

  while (true)
  {
    currentTime = std::chrono::system_clock::now();

    epmc.writeSpeed(highTargetVel, highTargetVel);
    float pos0, pos1;
    float vel0, vel1;
    bool success = epmc.readMotorData(pos0, pos1, vel0, vel1);

    if (success){
      p0 = pos0;
      p1 = pos1;
      v0 = vel0;
      v1 = vel1;
    }

    duration = currentTime - prevTime;
    std::cout << "duration: " << duration.count() << std::endl;

    prevTime = currentTime;
  }
    
}