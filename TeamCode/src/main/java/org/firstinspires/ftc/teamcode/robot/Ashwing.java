package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.RobotConstants.Sensors.*;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.subsystem.Descorer;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.utilities.BasicFilter;
import org.firstinspires.ftc.teamcode.utilities.RunningAverageFilter;

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
    public ServoEx descorerServo;

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
    public Descorer descorer;
    public Turret turret;

    // for writing logs
    public MultipleTelemetry flightRecorder;

    // for feedforward
    private double lastMeasuredVoltage = IDEAL_VOLTAGE;
    private boolean turretTuningOnly;

    public static Ashwing get() {
        return inst;
    }

    private Ashwing() {}

    public void reset() {
        Turret.targetPower = 0.0;
        turret.setMode(Turret.Mode.RAW);
        Turret.mode = Turret.Mode.RAW;
        Turret.lastMode = Turret.Mode.RAW;
        turretLeft.set(0.0);
        turretRight.set(0.0);
        if (!turretTuningOnly) {
            intake.setMode(Intake.Mode.OFF);
            intake.closeLatch();
            descorer.setMode(Descorer.Mode.STORE);
            shooter.setPower(0.0);
        }
    }

    public Ashwing init(Mode mode, Pose pose, HardwareMap map, Telemetry t) {
        return init(mode, pose, map, t, false);
    }

    public Ashwing init(Mode mode, Pose pose, HardwareMap map, Telemetry t,
                        boolean turretTuningOnly) {
        this.turretTuningOnly = turretTuningOnly;
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
        turretQuadratureEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Opposing continuous-rotation servos drive the turret together.
        turretLeft = new CRServo(map, TURRET_LEFT);
        turretRight = new CRServo(map, TURRET_RIGHT);
        turretLeft.setInverted(TURRET_LEFT_REVERSED);
        turretRight.setInverted(TURRET_RIGHT_REVERSED);

        t.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        flightRecorder = new MultipleTelemetry(t);
        hubs = new Hubs();
        turret = new Turret();

        lastMeasuredVoltage = IDEAL_VOLTAGE;

        if (!turretTuningOnly) {
            batterySensor = map.getAll(VoltageSensor.class).iterator().next();
            lastMeasuredVoltage = batterySensor.getVoltage();
            batteryFilter.updateValue(IDEAL_VOLTAGE / Math.max(lastMeasuredVoltage, MIN_FEEDFORWARD_VOLTAGE));

            shooterLeft = new MotorEx(map, SHOOTER_LEFT);
            shooterRight = new MotorEx(map, SHOOTER_RIGHT);
            shooterLeft.setInverted(SHOOTER_LEFT_REVERSED);
            shooterRight.setInverted(SHOOTER_RIGHT_REVERSED);
            shooterLeft.setRunMode(Motor.RunMode.RawPower);
            shooterRight.setRunMode(Motor.RunMode.RawPower);

            intakeLeft = new MotorEx(map, INTAKE_LEFT);
            intakeRight = new MotorEx(map, INTAKE_RIGHT);
            intakeLeft.setInverted(INTAKE_LEFT_REVERSED);
            intakeRight.setInverted(INTAKE_RIGHT_REVERSED);
            intakeLeft.setRunMode(Motor.RunMode.RawPower);
            intakeRight.setRunMode(Motor.RunMode.RawPower);

            latchServo = new ServoEx(map, LATCH);
            descorerServo = new ServoEx(map, DESCORER);
            descorerServo.setInverted(DESCORER_REVERSED);

            drive = new MecanumDrive(map, pose);
            shooter = new Shooter();
            intake = new Intake();
            descorer = new Descorer();
        }
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
        if (!turretTuningOnly) {
            flightRecorder.addData("Alliance", alliance.toString());
            lastMeasuredVoltage = batterySensor.getVoltage();
            flightRecorder.addData("Battery (V)", lastMeasuredVoltage);
            double safeVoltage = Math.max(lastMeasuredVoltage, MIN_FEEDFORWARD_VOLTAGE);
            batteryFilter.updateValue(IDEAL_VOLTAGE / safeVoltage);
        }

        CommandScheduler.getInstance().run();
        flightRecorder.addLine("<b>Loop timing</b>");
        double loopSeconds = hubs.getDeltaTime();
        if (loopSeconds > 0) {
            flightRecorder.addData("Loop rate (Hz)", 1.0 / loopSeconds);
        }

        if (!turretTuningOnly) {
            flightRecorder.addData("Drivetrain average (ms)", drive.getAverageLoopMs());
            flightRecorder.addData("Shooter average (ms)", shooter.getAverageLoopMs());
            flightRecorder.addData("Intake average (ms)", intake.getAverageLoopMs());
            flightRecorder.addData("Descorer average (ms)", descorer.getAverageLoopMs());
        }
        flightRecorder.addData("Turret average (ms)", turret.getAverageLoopMs());
        flightRecorder.update();

    }
}
