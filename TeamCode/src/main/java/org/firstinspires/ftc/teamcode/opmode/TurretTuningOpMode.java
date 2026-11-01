package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.Ashwing;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

/** Alternates between two turret targets using a timer. */
@Config
@TeleOp(name = "Tune: Turret PID", group = "Tuning")
public class TurretTuningOpMode extends OpMode {
    /** Total angular span, centered on zero. */
    public static double RANGE_DEGREES = 20.0;
    // How often to flip
    public static double STEP_SECONDS = 1.5;

    private Ashwing robot;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robot = Ashwing.get().init(Ashwing.Mode.TELEOP, Pose.zero(), hardwareMap,
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        RobotConstants.TurretTuning.tuning = true;
        robot.flightRecorder.addLine("Set turret encoder calibration and PID in RobotConstants.TurretTuning.");
        robot.flightRecorder.addLine("Press START to alternate turret angles; STOP halts all outputs.");
        robot.flightRecorder.update();
    }

    @Override
    public void start() {
        Turret.targetAngle = -RANGE_DEGREES / 2.0;
        timer.reset();
        robot.turret.setMode(Turret.Mode.DEBUG);
    }

    @Override
    public void loop() {
        if (timer.seconds() >= STEP_SECONDS) {
            Turret.targetAngle = -Turret.targetAngle;
            timer.reset();
        }
        robot.turret.setMode(Turret.Mode.DEBUG);
        robot.flightRecorder.addData("Turret target (deg)", Turret.targetAngle);
        robot.flightRecorder.addData("Turret measured (deg)", Turret.getLastAngle());
        robot.flightRecorder.addData("Turret error (deg)", Turret.targetAngle - Turret.getLastAngle());
        robot.endLoop();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.reset();
            robot.drive.follower.stop();
        }
        RobotConstants.TurretTuning.tuning = false;
        CommandScheduler.getInstance().reset();
    }
}
