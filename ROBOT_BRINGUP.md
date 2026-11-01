# Ashwing robot implementation tasks

Edit calibration in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotConstants.java`.
The grouped mechanism values are also exposed in FTC Dashboard. Enable the relevant
ShooterTuning/TurretTuning `tuning` flag to apply PID/tolerance edits live. Restart initialization after
hardware, filter-size, or Pedro configuration edits. Runtime modes and targets remain in the subsystems.

## Configure and wire the hardware

- [ ] Configure the turret CR servos as `turretLeft` and `turretRight`.
- [ ] Configure the analog encoder as `turretEncoder` and connect its analog signal.
- [ ] Connect the quadrature encoder to a motor encoder port. The current placeholder is the `intakeLeft` motor encoder socket; this must receive the turret encoder cable, not the intake motor encoder cable. Set `RobotConstants.Sensors.TURRET_QUADRATURE_MOTOR_NAME` to the final motor name before initialization.
- [ ] If that port belongs to another motor, keep that motor's power and run mode managed by its own subsystem. The turret only reads its encoder; turret zeroing uses a software offset.
- [ ] Configure intake motors as `intakeLeft` and `intakeRight`, shooter motors as `shooterLeft` and `shooterRight`, and latch servo as `latch`.
- [ ] Confirm drivetrain names (`lf`, `lr`, `rf`, `rr`) and OctoQuad device (`octoquad`, configured as `OctoQuadFTC`).

## Calibrate and test the turret

- [ ] Measure encoder counts per shaft revolution and shaft-to-turret gearing. Set `RobotConstants.TurretTuning.ENCODER_PPR` and `RobotConstants.TurretTuning.GEAR_RATIO`; counts per turret revolution are their product. Both are currently zero, so turret output remains stopped.
- [ ] Set `RobotConstants.Sensors.TURRET_ENCODER_OFFSET` so the analog reading is zero at the chosen forward angle. The measured 3.24 V full-scale value is used directly in `device/AbsoluteAnalogEncoder.java`; verify angle direction.
- [ ] Verify analog and quadrature angles agree while turning the turret slowly by hand. Confirm quadrature counts represent turret rotation, independently of any motor sharing that port.
- [ ] At low power, verify both opposing CR servos assist the same turret motion. The right servo is reversed in the container; change inversion if the actual mounting differs.
- [ ] Confirm positive output reduces a positive angle error before tuning PID. Check servo neutral behavior at zero command.
- [ ] Confirm or revise the retained travel limit (85 degrees) and analog valid zone (165 degrees) for the new mechanism.
- [ ] Tune turret PID, static feedforward, tolerance and motion compensation; these calibration values start at zero.
- [ ] Test both servo outputs stop for reset, missing encoder calibration, and a mode change through homing.

## Finish robot calibration

- [ ] Measure the signed turret-pivot offset from robot center and set `RobotConstants.Aiming.TURRET_OFFSET`.
- [ ] Refine the blue upper hive aim point, currently approximately `(85, 85)` inches. Verify mirrored lower/red targets and switching at `Y = 72`.
- [ ] Confirm this is an OctoQuad MK2 with onboard localization. Use hardware type `OctoQuadFTC`, and connect to a Control Hub I2C port other than port 0.
- [ ] Enter the actual forward and strafe pod ports in OctoQuad AutoTune (bootstrap defaults are ports 0 and 1). Select Custom for the SWYFT pods.
- [ ] Run OctoQuad AutoTune before Foresight tuning. It identifies heading scalar, custom encoder resolution if needed, pod directions and pod offsets. You do not need to enter measured pod coordinates first.
- [ ] Paste the generated `OctoQuadConfig` into `RobotConstants.Pedro.localizerConfig`. The nominal SWYFT scale uses 4096 counts per revolution and a 38 mm wheel (about 871.5 ticks/inch). Validate it with the Custom-pod distance test; offsets, directions, and heading scale still require calibration.
- [ ] Validate measured translation and heading, then run the mecanum/Foresight procedures and replace the zero drivetrain calibration values.
- [ ] Set intake ingest/discard powers and latch positions, then verify opposing intake directions.
- [ ] Add at least two measured distance/velocity pairs to `RobotConstants.ShooterTuning.DISTANCE_VELOCITY_POINTS` in increasing distance order, then reinitialize. Interpolation returns zero until the table is populated. Tune flywheel PID/feedforward and shooting-on-the-move compensation.
- [ ] Verify both alliances, all intended modes, and stop/reset behavior on the robot. Desktop compilation does not validate wiring or physical tuning.

## SWYFT odometry reference

Manufacturer specifications: 38 mm wheel, 4096 PPR magnetic quadrature encoder.
The official example uses `4096 / (PI * 38)` counts/mm; the code converts this to inches.
Select Custom in the OctoQuad tuner and verify the measured travel before accepting the scale.

- https://shop.swyftrobotics.com/products/swyft-linear-odometry-module
- https://cdn.shopify.com/s/files/1/0875/8486/6598/files/swyft_encoder_gobilda_pinpoint_example_code.java?v=1764901678
