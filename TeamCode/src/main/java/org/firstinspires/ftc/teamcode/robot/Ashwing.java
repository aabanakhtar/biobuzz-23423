package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.RobotConstants.Sensors.*;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.device.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.subsystem.Hubs;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.utilities.BasicFilter;
import org.firstinspires.ftc.teamcode.utilities.RunningAverageFilter;

import java.util.Arrays;
import java.util.List;

@Config
public class Ashwing {
    private static final Ashwing inst = new Ashwing();

    public enum Mode {
        AUTO,
        TELEOP
    }

    public enum Alliance {
        RED, BLUE
    }

    // alliance settings
    public static Alliance alliance = Alliance.BLUE;
    public static Mode mode = Mode.AUTO;

    // hardware (besides dt, managed by pedro)
    public MotorEx shooterLeft, shooterRight;
    public CRServo turretLeft, turretRight;
    // Read only: this motor port supplies quadrature ticks, not turret actuation.
    public DcMotorEx turretQuadratureEncoder;
    public MotorEx intakeLeft, intakeRight;
    public ServoEx latchServo;

    // sensors
    public VoltageSensor batterySensor;
    private final BasicFilter batteryFilter = new RunningAverageFilter(BATTERY_FILTER_SAMPLES);
    public AbsoluteAnalogEncoder turretEncoder;

    // hubs
    public List<LynxModule> lynxModules;
    public HardwareMap hardwareMap;

    // Subsystems registered with the SolversLib scheduler
    public Hubs hubs;
    public MecanumDrive drive;
    public Shooter shooter;
    public Intake intake;
    public Turret turret;

    // for writing logs
    public MultipleTelemetry flightRecorder;

    // for feedforward
    private double lastMeasuredVoltage = IDEAL_VOLTAGE;

    public static Ashwing get() {
        return inst;
    }

    private Ashwing() {}

    public void reset() {
        intake.setMode(Intake.Mode.OFF);
        intake.closeLatch();
        intakeLeft.set(0.0);
        intakeRight.set(0.0);
        Turret.targetPower = 0.0;
        turret.setMode(Turret.Mode.RAW);
        Turret.mode = Turret.Mode.RAW;
        Turret.lastMode = Turret.Mode.RAW;
        turretLeft.set(0.0);
        turretRight.set(0.0);
        shooter.setPower(0.0);
        shooterLeft.set(0.0);
        shooterRight.set(0.0);
    }

    public Ashwing init(Mode mode, Pose pose, HardwareMap map, Telemetry t) {
        CommandScheduler.getInstance().reset();
        batteryFilter.reset();

        Ashwing.mode = mode;
        hardwareMap = map;
        lynxModules = map.getAll(LynxModule.class);
        lynxModules.forEach(x -> {
            x.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        });

        turretEncoder = new AbsoluteAnalogEncoder(hardwareMap, TURRET_ANALOG_ENCODER, TURRET_ENCODER_OFFSET);
        turretQuadratureEncoder = map.get(DcMotorEx.class, TURRET_QUADRATURE_MOTOR_NAME);

        // sensors
        batterySensor = hardwareMap.getAll(VoltageSensor.class)
                .iterator()
                .next();

        // Shooter motors
        shooterLeft = new MotorEx(map, SHOOTER_LEFT);
        shooterRight = new MotorEx(map, SHOOTER_RIGHT);
        // Counter-rotating shooter motors.
        shooterLeft.setInverted(SHOOTER_LEFT_REVERSED);
        shooterRight.setInverted(SHOOTER_RIGHT_REVERSED);

        // Opposing continuous-rotation servos drive the turret together.
        turretLeft = new CRServo(map, TURRET_LEFT);
        turretRight = new CRServo(map, TURRET_RIGHT);
        turretLeft.setInverted(TURRET_LEFT_REVERSED);
        turretRight.setInverted(TURRET_RIGHT_REVERSED);
        turretLeft.set(0.0);
        turretRight.set(0.0);

        // Opposing intake motors share power commands; inversion lives with the hardware.
        intakeLeft = new MotorEx(map, INTAKE_LEFT);
        intakeRight = new MotorEx(map, INTAKE_RIGHT);
        intakeLeft.setInverted(INTAKE_LEFT_REVERSED);
        intakeRight.setInverted(INTAKE_RIGHT_REVERSED);

        latchServo = new ServoEx(map, LATCH);

        // Apply common run modes
        Arrays.asList(shooterLeft, shooterRight, intakeLeft, intakeRight)
                .forEach(x -> {
                    x.setRunMode(Motor.RunMode.RawPower);
                });

        flightRecorder = new MultipleTelemetry(t);
        lastMeasuredVoltage = batterySensor.getVoltage();
        batteryFilter.updateValue(IDEAL_VOLTAGE / Math.max(lastMeasuredVoltage, MIN_FEEDFORWARD_VOLTAGE));

        // subsystem init
        hubs = new Hubs();
        drive = new MecanumDrive(map, pose);
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();
        reset();
        return inst;
    }

    public double getVoltage() {
        return lastMeasuredVoltage;
    }

    public double getVoltageFeedforwardConstant() {
        return batteryFilter.getFilteredOutput();
    }

    public void endLoop() {
        // Clear before any scheduler subsystem reads; subsystem iteration order is unspecified.
        hubs.clearCache();
        flightRecorder.addData(">>>>ALLIANCE", alliance.toString());
        // sensor update
        flightRecorder.addData(">>>>BATTERY STATE", lastMeasuredVoltage);

        lastMeasuredVoltage = batterySensor.getVoltage();
        double safeVoltage = Math.max(lastMeasuredVoltage, MIN_FEEDFORWARD_VOLTAGE);
        batteryFilter.updateValue(IDEAL_VOLTAGE / safeVoltage);

        CommandScheduler.getInstance().run();
        flightRecorder.update();

    }
}
