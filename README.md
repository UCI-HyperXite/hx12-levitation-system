# HX11 four-magnet lev pod firmware (STM32H7)

The firmware is a single file, `hx11_lev.c`. All the values you need to fill in or tune are in the block at the top. They are marked `FILL` (unknown, still a guess) or `TUNE` (adjust on hardware).

## Build and flash

1. In STM32CubeIDE, create a project for your Nucleo-144 H7 board. Configure only the clock tree, SYS (Serial Wire debug) and the SysTick timebase.
2. Leave ADC1, TIM7, TIM8, SPI1, USART3 and their DMA **unconfigured**. This file defines their IRQ handlers and HAL callbacks.
3. In `stm32h7xx_hal_conf.h`, enable the ADC, DMA, TIM, SPI, UART and GPIO HAL modules.
4. Give SPI123 a kernel clock (PLL1Q or per_ck). Keep HSI on, because the ADC runs from per_ck.
5. Add `hx11_lev.c` to `Core/Src`. In `main.c`, declare `void hx11_main(void);` and call `hx11_main();` in `USER CODE BEGIN 2`.
6. Build with `-O2` and the hardware FPU, then flash over the Nucleo's ST-LINK.

## Commands (serial over ST-LINK USB, 921600 baud, one line each)

| Command | Effect |
|---|---|
| `ARM` / `DISARM` | Enable / disable coil drive |
| `LIFT` / `LAND` | Lift off / land (from ARMED / while flying) |
| `KILL` | All coils off immediately |
| `ZERO` | Recalibrate current offsets and gyro bias (DISARMED, pod still) |
| `SP <mm>` | Hover gap setpoint |
| `BENCH <coil> <A>` | Drive one coil (0=FL 1=FR 2=RL 3=RR); `0` A to stop |

The firmware sends one telemetry line at 200 Hz:
`T,ms,state,h_um,sp_um,roll_urad,pitch_urad,hdot_um_s,p_mrad_s,q_mrad_s,gFL..gRR_um,irFL..irRR_mA,imFL..imRR_mA,vA,vB,vC_mV`

## Bring-up

1. With coil power off, check the gaps, roll/pitch signs and gyro signs in telemetry.
2. With power on, run `ARM` then `BENCH 0 0.5`. The measured current must read +0.5 A; if it doesn't, flip `CUR_SIGN` or `DIR_FWD` for that coil. Repeat for coils 1–3.
3. Step-test each coil with `BENCH` to get L and tune `CUR_BW_HZ`.
4. Tether the pod, then run `LIFT` and `LAND` with low attitude gains. Tune heave first, then roll and pitch.
