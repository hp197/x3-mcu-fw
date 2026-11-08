# x3-mcu-fw

Open-source application firmware for the Segway-Ninebot **x3** MCU — the motor-controller
unit — for an Artery **AT32F415-class** ARM Cortex-M4. It is the sibling of
[x3-vcu-fw](https://github.com/hp197/x3-vcu-fw) (the vehicle-control unit) and
[x3-vcu-boot](https://github.com/hp197/x3-vcu-boot) (the 4 KB bootloader); this is the
motor-drive processor, launched by the bootloader at `0x08001000` (60 KB slot 0).

## Scope

The MCU drives the brushless motor and answers to the VCU over **CAN**. It senses phase
current, bus voltage and the throttle command through the **ADC**, drives the three half-bridges
with **TMR1** complementary PWM, tracks the rotor with the **hall sensors**, and cuts the drive
on an over-current / over-voltage / stall fault. Unlike the VCU it has no USART and no IMU — it is
a CAN node. The modules:

- `main` / `clock` — boot, the 144 MHz clock, SysTick, the control super-loop
- `can` — the command / telemetry link to the VCU
- `adc` — phase-current / bus-voltage / throttle sensing (ADC1 + DMA)
- `motor` — TMR1 complementary PWM, the 3-phase bridge
- `timers` — the TMR3 hall-sensor interface
- `control` — the commutation, current and speed loops
- `faultmon` — over-current / over-voltage / stall protection and the current-offset calibration
- `gpio` — gate-driver enables, hall / fault input pins
- `config` — the flash controller, the identity record and the calibration / tune block
- `backup` — a read-only diagnostic dump of the per-unit data, over CAN

## Build

PlatformIO pulls its own copy of the Artery platform and BSP, so no local checkout is needed.

```bash
pio run            # build -> .pio/build/mcu/firmware.bin
pio run -t clean   # clean
```

Flashing is disabled in `platformio.ini`; flash the resulting image to `0x08001000` with the x3
bootloader's OTA path or over SWD.

## License

The work in this repository is licensed under **CC BY-NC-SA 4.0** (see [LICENSE](LICENSE)). The
Artery AT32F415 BSP and CMSIS used at build time keep their own licenses.
