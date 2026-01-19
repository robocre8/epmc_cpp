- run in the root directory
  ```shell
    rm -rf build

    rm -rf debian/epmc-serial*

    rm -rf build debian/epmc-serial* debian/.debhelper* debian/debhelper* debian/epmc-serial* debian/files obj-*
  ```

  ```shell
  sudo apt update

  sudo apt remove epmc-serial-dev

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

- install built .deb package
  ```shell
    sudo apt remove libserial-dev #uninstall
    sudo apt install ../epmc-serial-dev_<version>_amd64.deb #this should install the libserialdev along
  ```

- check if installed
  ```shell
    dpkg -L epmc-serial-dev
  ```

- remove
  ```shell
    rm -rf build debian/epmc-serial* debian/.debhelper* debian/debhelper* debian/epmc-serial* debian/files obj-*
  ```
