# aed-project
This branch implements the program for the third board using STM32 HAL API.

Board model: [NUCLEO-F429ZI](https://www.st.com/en/evaluation-tools/nucleo-f429zi.html) as I2C slave (addr 0x1) that toggles some LEDs depending on the info received from [board 2](https://github.com/ruixiang-shaop/aed-project/tree/placa2), which was sent by [board 1](https://github.com/ruixiang-shaop/aed-project/tree/placa1).

## Pins

| Pin | Description |
|--|--|
| PC13 | Board button (alarm) |
| PB0  | Board LD1 (green) |
| PB7  | Board LD2 (blue) |
| PB14 | Board LD3 (red) |
| PB8  | LED1 |
| PB15 | LED2 |
| PA5  | LED3 |
| PA6  | LED4 |
| PD14 | LED5 |
| PF0  | I2C2_SDA |
| PF1  | I2C2_SCL |

## Development environment

STM32CubeIDE v1.6.1 on Windows 10.
