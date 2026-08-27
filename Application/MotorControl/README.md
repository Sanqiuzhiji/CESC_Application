# CESC motor-control structure

The module boundaries follow the responsibilities used by VESC while keeping
the existing CESC application directory layout.

## Control

`Control/motor_control_config.h` defines `motor_control_config_t` and
`Control/motor_control_config.c` contains the defaults for the attached motor
and externally visible motion behavior. Motor pole pairs, torque constant,
limits, watchdog timing, encoder PLL gains, current-loop gains and motion
defaults must be changed there rather than scattered through the power-stage
driver.

The constant instance is intentionally shaped like VESC's
`mc_configuration`. It can later be replaced by a validated RAM copy loaded
from non-volatile storage without changing the FOC or hardware APIs.

## FOC

`FOC/foc.c` contains hardware-independent math:

- Clarke and Park transforms;
- d/q current PI control and anti-windup;
- voltage-vector limiting;
- inverse Park and three-phase PWM generation;
- encoder-oriented direct-q voltage generation.

The API uses explicit input/output structures, similar to VESC's
`motor_state_t` and `control_current()`, so algorithm state is visible and can
be unit-tested without STM32 peripherals.

## PowerStage

`PowerStage/power_stage.c` owns hardware and safety:

- synchronized ADC acquisition and offset correction;
- TIM1 compare writes and gate enable/disable;
- DRV8301 and bus-voltage fault handling;
- command watchdog and encoder-validity checks;
- commissioning and motor-parameter measurement state machines;
- dispatch between torque/current FOC and encoder-oriented voltage modes.

No protocol-facing configuration value should be introduced directly in this
file. Identification-only timing constants may remain local because they are
implementation details rather than user motor settings.
