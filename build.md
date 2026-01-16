- run in the root directory
  ```shell
    rm -rf build

    rm -rf debian/epmc-serial*
  ```

  ```shell
  sudo apt update

  sudo apt install \
    build-essential \
    cmake \
    dpkg-dev \
    debhelper \
    pkg-config \
    libserial-dev
  ```

  ```shell
  cmake -S . -B build

  cmake --build build
  ```

  ```shell
  dpkg-buildpackage -us -uc
  ```