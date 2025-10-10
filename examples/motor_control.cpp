
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

  float lowTargetVel = 0.00; // in rad/sec
  float highTargetVel = 3.142; // in rad/sec

  auto prevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> duration;
  float sampleTime = 0.02;

  auto ctrlPrevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> ctrlDuration;
  float ctrlSampleTime = 4.0;

  // std::string port = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0";
  std::string port = "/dev/ttyUSB0";
  epmc.connect(port);

  delay_ms(4000);

  epmc.writeSpeed(0.0, 0.0);
  epmc.clearDataBuffer();

  int motor_cmd_timeout_ms = 5000;
  epmc.setCmdTimeout(motor_cmd_timeout_ms); // set motor command timeout
  motor_cmd_timeout_ms = epmc.getCmdTimeout();
  std::cout << "motor command timeout: " << motor_cmd_timeout_ms << " ms" << std::endl;

  epmc.writeSpeed(lowTargetVel, lowTargetVel);

  sendHigh = true;

  prevTime = std::chrono::system_clock::now();
  ctrlPrevTime = std::chrono::system_clock::now();

  while (true)
  {

    ctrlDuration = (std::chrono::system_clock::now() - ctrlPrevTime);
    if (ctrlDuration.count() > ctrlSampleTime)
    {
      if (sendHigh)
      {
        epmc.writeSpeed(highTargetVel, highTargetVel);

        sendHigh = false;
      }
      else
      {
        epmc.writeSpeed(lowTargetVel, lowTargetVel);

        sendHigh = true;
      }

      ctrlPrevTime = std::chrono::system_clock::now();
    }

    duration = (std::chrono::system_clock::now() - prevTime);
    if (duration.count() > sampleTime)
    {
      try
      {
        float pos0, pos1;
        float v0, v1;
        epmc.readMotorData(pos0, pos1, v0, v1);

        std::cout << "----------------------------------" << std::endl;
        std::cout << "motorA_readings: [" << pos0 << std::fixed << std::setprecision(4) << "," << v0 << std::fixed << std::setprecision(4) << "]" << std::endl;
        std::cout << "motorB_readings: [" << pos1 << std::fixed << std::setprecision(4) << "," << v1 << std::fixed << std::setprecision(4) << "]" << std::endl;
        std::cout << "----------------------------------" << std::endl;
        std::cout << std::endl;
      }
      catch (...)
      {
        
      }

      prevTime = std::chrono::system_clock::now();
    }
  }
}