# TM4C123GH6PM FreeRTOS Projects

A series of six embedded systems projects on the TM4C123GH6PM (TM4C123GXL
LaunchPad), built with FreeRTOS and TivaWare, progressing from single-primitive
demonstrations to a closed-loop motor controller communicating with a second,
independent microcontroller over CAN. Each project's own README documents its
architecture, hardware setup, verified behavior (logic analyzer/oscilloscope
captures and UART logs), and a "Design notes / known limitations" section
covering real bugs found and fixed during development, not just a description
of what was built.

## Projects

### [LED Blink Controller](./led_blink_controller)
A debounced GPIO button interrupt cycles an LED's blink rate via a direct
task notification and a non-blocking, "latest value wins" queue pattern.
**Demonstrates:** direct task notifications, non-blocking queue reads,
ISR debounce, `portYIELD_FROM_ISR`.

### [Unified Event Logger](./unified_event_logger)
Two debounced buttons and a recurring software timer act as independent
event sources, funneling through a single queue into one consumer task.
**Demonstrates:** many-producer/one-consumer queue handling, software
timers, mutex-protected shared UART access.

### [Multi Sensor Hub](./multi_sensor_hub)
Two independent ADC producer tasks (a potentiometer and the TM4C's internal
temperature sensor) synchronize through a FreeRTOS event group before a
higher-priority consumer task processes and logs both readings.
**Demonstrates:** event groups (wait-for-ALL), dual ADC sequencer use,
debug GPIO instrumentation, and a real, explained task-priority preemption
effect captured on a logic analyzer.

### [ADC PWM Pipeline](./adc_pwm_pipeline)
A potentiometer is sampled on a fixed period and converted into LED
brightness via hardware PWM, with a third task independently logging
status over UART.
**Demonstrates:** `vTaskDelayUntil()` for precise periodic sampling, a
single-slot `xQueueOverwrite()`/`xQueuePeek()` mailbox for two independent
readers, binary semaphore signaling (with an explained, honest tradeoff
against a task notification in hindsight).

### [I2C MPU6050 Hub](./i2c_mpu6050_hub)
An MPU6050 accelerometer/gyroscope/temperature sensor is sampled over a
fully interrupt-driven I2C master driver built from scratch, with a serial
command interface to pause, resume, and query the pipeline live.
**Demonstrates:** an ISR-driven I2C state machine with binary semaphore
handoff (no polling), a found-and-fixed repeated-START protocol bug
verified against a logic analyzer, and an interrupt-driven UART command
interface using direct task notifications.

### [Closed-Loop Motor Controller](./motor_controller)
The capstone project: a PID-controlled DC motor speed loop using
quadrature encoder feedback, a rebuilt interrupt-driven I2C driver for an
INA260/TMP117 sensor pair, a latching overcurrent/overtemperature safety
fault, and a two-node CAN bus (500kbit/s) to a second, independent
STM32L476RG node built bare-metal after a toolchain change removed
integrated CubeMX support.
**Demonstrates:** everything above, applied together, plus closed-loop
control tuning verified against oscilloscope measurements, a
diagnosed-and-fixed encoder direction-sign bug, cross-vendor CAN
communication verified bidirectionally from both nodes' own receive
lines, and real electrical noise diagnosed and documented from a logic
analyzer capture.

## Progression

The projects build on each other deliberately rather than being six
disconnected exercises, later projects reuse and refine primitives
introduced earlier, and each README cross-references where a design choice
first appeared:

- **Synchronization primitives** — from a single binary semaphore (ADC PWM
  Pipeline) → event groups (Multi Sensor Hub) → direct task notifications
  chosen deliberately over semaphores (LED Blink Controller, then reapplied
  in I2C Hub and Motor Controller's command interfaces)
- **I2C** — I2C Hub builds the first interrupt-driven driver and finds the
  repeated-START bug; Motor Controller rebuilds an equivalent driver from
  scratch, fixing the same class of bug independently before recognizing
  the pattern from I2C Hub
- **Shared-value queues** — ADC PWM Pipeline establishes the
  `xQueueOverwrite()`/`xQueuePeek()` pattern for two independent readers;
  Motor Controller reapplies the exact same fix for the same underlying
  race condition between its motor and CAN tasks

## Common conventions across all six

- **Toolchain**: Code Composer Studio (CCS), TivaWare C Series (2.2.0.295),
  TM4C123GH6PM target
- **Static/pool-only memory allocation**, enforced in every project by
  trapping the standard `malloc()`
- **UART**: 115200 8-N-1 across all projects, mutex-protected in every
  project with more than one printing source
- **A local FreeRTOS source tree** (built against FreeRTOS's TivaWare CCS
  port) is required to build any of these and is not included in this
  repository, download from [freertos.org](https://www.freertos.org)

Each project folder's own README has full build instructions, hardware
wiring, and verified-behavior captures specific to that project.
