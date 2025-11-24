## Easy PID Motor Controller (EPMC) C++ I2C Library
This library helps communicate with the **`Easy PID Motor Controller Module`**  with your microcomputer linux c++ robotic project via **`I2C protocol`**

> you can use it in your microcomputer robotics project **running on linux** (e.g Raspberry Pi, nvidia, etc.)

A simple way to get started is simply to try out and follow the example code in the example folder

## How to Use the Library
- install the libserial-dev package
  > sudo apt-get update
  >
  > sudo apt install i2c-tools

- For Raspberry Pi, you might need to enable the I2C interface in the configuration.
  Edit `/boot/firmware/config.txt` (or `/boot/config.txt` on older systems) and add or uncomment the following lines:
  > dtparam=i2c_arm=on

- Ensure you have the **`EPMC MODULE`** interfaced with your preferred motors, setup the encoder and PID parameters with the **`epmc_setup_application`**.

- Download (by clicking on the green Code button above) or clone the repo into your PC using **`git clone`**
> [!NOTE]  
> you can use this command if you want to clone the repo:
> 
> ```git clone -b esp-c3-i2c https://github.com/samuko-things-company/epmc_cpp.git```

- check the EPMC i2c address:
  ```shell
  sudo i2cdetect -y 1
  ```

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
  > .connect("port_name or port_path")
  > .clearDataBuffer()

- other functions
  > .writeSpeed(float v0, float v1);
  > .writePWM(int pwm0, int pwm1);
  > .readMotorData(float &pos0, float &pos1, float &v0, float &v1);
  > .getMaxVel(int motor_no);
  > .setCmdTimeout(int timeout_ms);
  > .getCmdTimeout();
  > .setPidMode(int mode);
  > .getPidMode();
  