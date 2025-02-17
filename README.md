# Helmet_MPU6050_iic
## Reference of I2C
https://www.kernel.org/doc/Documentation/i2c/dev-interface

## See what number corresponds to which adapter
```bash
run i2cdetect -l
```

## Install libraries
```bash
sudo apt update
sudo apt install -y libgpiod-dev libi2c-dev
```

## How to use
```bash
cd build/
cmake ..
make
sudo ./iic_test
```
