package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.Ashwing;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

/** Fixed velocity tuning, independent of the distance interpolation table. */
@Config
@TeleOp(name = "Tune: Shooter Velocity", group = "Tuning")
public class ShooterTuningOpMode extends OpMode {
    /** Flywheel encoder ticks per second; editable in FTC Dashboard while running. */
    public static double TARGET_VELOCITY = 0.0;

    private Ashwing robot;

    @Override
    public void init() {
        robot = Ashwing.get().init(Ashwing.Mode.TELEOP, Pose.zero(), hardwareMap,
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        RobotConstants.ShooterTuning.tuning = true;
        robot.flightRecorder.addLine("Edit ShooterTuningOpMode.TARGET_VELOCITY in FTC Dashboard (ticks/sec).");
        robot.flightRecorder.addLine("Edit PID/feedforward in RobotConstants.ShooterTuning; press START to run.");
        robot.flightRecorder.update();
    }

    @Override
    public void loop() {
        double target = Math.max(0.0, TARGET_VELOCITY);
        robot.shooter.setVelocity(target);
        robot.flightRecorder.addData("Shooter target (ticks/sec)", target);
        robot.flightRecorder.addData("Shooter measured (ticks/sec)", Shooter.currentVelo);
        robot.flightRecorder.addData("Shooter error (ticks/sec)", target - Shooter.currentVelo);
        robot.endLoop();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.reset();
            robot.drive.follower.stop();
        }
        RobotConstants.ShooterTuning.tuning = false;
        CommandScheduler.getInstance().reset();
    }
}
