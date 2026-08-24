# Читання MPU-6050 через I2C

1. Зробити білд з кореня репо:
```bash
make build-debug
```

2. Запустити з кореня репо через I2C симулятор:
```bash
LD_PRELOAD=./homeworks/16_working_with_I2C_device/libi2csim-arm64.so ./build/debug/homeworks/16_working_with_I2C_device/working_with_I2C_device /dev/i2c-1 0x68
```
