# qt-sensors-mpu6050
Baremetal I2C QtSensor Plugin for MPU6050

# Installation

Run the following commands in a terminal session to download and setup the plugin
```console
git clone https://github.com/DigitalArtifex/qt-sensors-mpu6050.git
mkdir qt-sensors-mpu6050/build
cd qt-sensors-mpu6050/build
```

Next, we will need to configure the project against your Qt Installation by executing `qt-cmake` located in `<Qt Path>/<Version>/<Arch>/bin`. Typically, the Qt Path will be `~/Qt`, but can be `/opt/Qt` if installed with sudo, or elsewhere if configured differently. The example assumes Qt Path to be `/opt/Qt`, Qt Version to be `6.9.0` and the arch to be `gcc_arm64`

```
/opt/Qt/6.9.0/gcc_arm64/bin/qt-cmake -S ../ -B ./
```

Once configured, we will switch to the system provided cmake. If Qt is installed to `~/` there is no need to execute `cmake install` with sudo

```
cmake --build ./
sudo cmake --install ./
```

# Usage

## Adding the reference (CMake)

```cmake
target_link_libraries(
  MyApp
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Sensors
  i2c
  qt${QT_VERSION_MAJOR}-sensors-mpu6050
)
```

## Using in code

```cpp
    //The QMPU6050 needs to be initialized before we can use it as
    // an accelerometer or gyroscope. This object can also be used
    // to configure the various properties and filters of the sensor
    mpu6050 = new QMPU6050;
    mpu6050->connectToBackend();
    mpu6050->initialize();

    accel = new QAccelerometer;
    accel->connectToBackend();
    QObject::connect(accel, &QAccelerometer::readingChanged, &accelChanged);

    accel->setDataRate(1);
    //by default the sensor is located at bus i2c-1 with an address of 0x68
    //if this needs to be changed, change the i2c-bus/i2c-address property of
    //the sensor object
    // accel->setProperty("i2c-address", 0x69);
    accel->start();

    gyro = new QGyroscope;
    gyro->connectToBackend();
    QObject::connect(gyro, &QGyroscope::readingChanged, &gyroChanged);

    gyro->setDataRate(1);
    gyro->start();
```
