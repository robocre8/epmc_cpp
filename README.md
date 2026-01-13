## Easy PID Motor Controller (EPMC) C++ Library
This library helps communicate with the **`Easy PID Motor Controller Module`** (i.e **`L298N EPMC MODULE`** or a **`CUSTOM EPMC INTERFACE BOARD`**) in your PC or microcomputer linux c++ robotic project (as it depends on the libserial-dev package), with the [epmc_setup_application](https://github.com/robocre8/epmc_setup_application).

> you can use it in your microcomputer robotics project **running on linux** (e.g Raspberry Pi, PC, etc.)

A simple way to get started is simply to try out and follow the example code in the example folder

## How to Use the Library
- install the libserial-dev package
  > sudo apt-get update
  >
  > sudo apt install libserial-dev

- Ensure you have the **`EPMC MODULE`** interfaced with your preferred motors, setup the encoder and PID parameters with the **`epmc_setup_application`**.

- Download (by clicking on the green Code button above) or clone the repo into your PC using **`git clone`**
> [!NOTE]  
> you can use this command if you want to clone the repo:
> 
> ```git clone https://github.com/robocre8/epmc_cpp.git```

- check the serial port the driver is connected to:
  ```shell
  ls /dev/ttyA*
  ```
  > you should see /dev/ttyACM0 or /dev/ttyACM1 and so on

- A simple way to get started is simply to try out and follow the example `motor_control.cpp` code.

- make, build and run the example code.
  > cd into the root directory
  >
  > mkdir build (i.e create a folder named build)
  >
  > enter the following command in the terminal in the root folder:
    ````
    cmake -B ./build/
    ````
    ````
    cmake --build ./build/
    ````
    ````
    ./build/motor_control
    ````

- You can follow the pattern used in the example `motor_control.cpp` in your own code.

## Basic Library functions and usage

- connect to epmc_driver shield module
  > EPMC epmc
  >
  > epmc.connect("port_name or port_path")
  >
  > epmc.clearDataBuffer() # returns bool -> success

- send target angular velocity command
  > epmc.writeSpeed(motor0_TargetVel, motor1_TargetVel)

- send PWM command
  > epmc.writePWM(motor0_PWM, motor1_PWM)

- set motor command timeout
  > epmc.setCmdTimeout(timeout_ms)

- get motor command timeout
  > epmc.getCmdTimeout() # returns std::tuple -> (success, motor_command_timeout_ms): bool, float

- read motors angular position
  > epmc.readPos() # returns std::tuple -> (success, angPos0, angPos1): bool, float, float

- read motors angular velocity
  > epmc.readVel() # returns std::tuple -> (success, angVel0, angVel1): bool, float, float

- read motorA maximum commandable angular velocity
  > epmc.getMaxVel(motor_no) # returns std::tuple -> (success, max_vel): bool, float, float
  > maxVel0 or maxVel1 based on the specified motor number
