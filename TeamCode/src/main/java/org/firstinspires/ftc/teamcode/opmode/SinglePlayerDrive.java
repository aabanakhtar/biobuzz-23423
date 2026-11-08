package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.RobotConstants.TeleOpTuning.*;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.cmd.Commandlet;
import org.firstinspires.ftc.teamcode.robot.Ashwing;
import org.firstinspires.ftc.teamcode.subsystem.Descorer;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

/** Single-driver controls adapted from DECODE. */
@TeleOp(name = "TeleOp", group = "Ashwing")
public class SinglePlayerDrive extends OpMode {
    private Ashwing robot;
    private FtcDashboard dashboard;
    private Command shootCommand;
    private PIDFController headingController;
    private boolean headingLock;

    @Override
    public void init() {
        Pose startingPose = USE_LAST_POSE ? MecanumDrive.lastPose
                : new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        dashboard = FtcDashboard.getInstance();
        robot = Ashwing.get().init(Ashwing.Mode.TELEOP, startingPose, hardwareMap,
                new MultipleTelemetry(telemetry, dashboard.getTelemetry()));
        headingController = new PIDFController(HEADING_kP, HEADING_kI, HEADING_kD, 0);
        headingLock = false;
        shootCommand = new SequentialCommandGroup(
                run(() -> robot.intake.setMode(Intake.Mode.OFF)),
                Commandlet.shootTeleOp()
        );

        GamepadEx controls = new GamepadEx(gamepad1);
        controls.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(shootCommand);
        controls.getGamepadButton(GamepadKeys.Button.SHARE).whenPressed(
                run(() -> robot.drive.follower.setPose(new Pose(72, 72, 0))));
        controls.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(run(() -> {
            headingLock = !headingLock;
            headingController.reset();
        }));
        controls.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                run(() -> robot.descorer.setMode(Descorer.Mode.DEPLOY)));
        controls.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                run(() -> robot.descorer.setMode(Descorer.Mode.STORE)));
        controls.getGamepadButton(GamepadKeys.Button.X)
               .whenPressed(run(() -> Commandlet.updateIntake(gamepad1.x, gamepad1.b, gamepad1.a, shootCommand.isScheduled()))).whenReleased(run(() -> Commandlet.updateIntake(gamepad1.x, gamepad1.b, gamepad1.a, shootCommand.isScheduled())));
        controls.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(run(() -> Commandlet.updateIntake(gamepad1.x, gamepad1.b, gamepad1.a, shootCommand.isScheduled()))).whenReleased(run(() -> Commandlet.updateIntake(gamepad1.x, gamepad1.b, gamepad1.a, shootCommand.isScheduled())));
        controls.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(run(() -> Commandlet.updateIntake(gamepad1.x, gamepad1.b, gamepad1.a, shootCommand.isScheduled()))).whenReleased(run(() -> Commandlet.updateIntake(gamepad1.x, gamepad1.b, gamepad1.a, shootCommand.isScheduled())));
    }


    @Override
    public void init_loop() {
        robot.hubs.clearCache();
        robot.turretEncoder.offset = RobotConstants.Sensors.TURRET_ENCODER_OFFSET;
        robot.turret.loadAngle(robot.turretEncoder.getCurrentPosition());
        robot.flightRecorder.addData("Alliance", Ashwing.alliance);
        robot.flightRecorder.addData("Starting pose", robot.drive.getPose());
        robot.flightRecorder.addData("Turret angle (deg)", Turret.getLastAngle());
        robot.flightRecorder.update();
    }

    @Override
    public void start() {
        robot.turret.loadAngle(robot.turretEncoder.getCurrentPosition());
        robot.turret.setMode(Turret.Mode.TRACKING);
        robot.shooter.setIdle();
    }

    @Override
    public void loop() {
        robot.drive.follower.manual(
                -gamepad1.left_stick_y * DRIVE_SPEED,
                -gamepad1.left_stick_x * DRIVE_SPEED,
                -gamepad1.right_stick_x
        );

        robot.flightRecorder.addLine("<b>TeleOp</b>");
        robot.flightRecorder.addData("Shooting", shootCommand.isScheduled());
        robot.endLoop();
        sendFieldPacket();
    }

    private void sendFieldPacket() {
        Pose pose = robot.drive.getPose();
        Pose target = MecanumDrive.selectHiveTarget(pose, Ashwing.alliance);

        
        // Dashboard's field frame is centered at (0, 0); Pedro's origin is (0, 0)
        // at the bottom-left. Translate Pedro coordinates by half the 144-inch field.
        TelemetryPacket packet = new TelemetryPacket(false);
        Canvas field = packet.fieldOverlay();
        field.setStroke("#888888").setStrokeWidth(1).strokeRect(-72, -72, 144, 144);
        field.setStroke("#555555").strokeLine(-72, 0, 72, 0);
        field.setTranslation(-72, -72);
        field.setStroke("#00A6FF").strokeCircle(target.x(), target.y(), 3);
        field.setStroke("#FFFFFF").strokeCircle(pose.x(), pose.y(), 9);
        double heading = AngleUnit.normalizeRadians(pose.heading());
        field.strokeLine(pose.x(), pose.y(),
                pose.x() + 12 * Math.cos(heading),
                pose.y() + 12 * Math.sin(heading));
        packet.put("Pose X (in)", pose.x());
        packet.put("Pose Y (in)", pose.y());
        packet.put("Pose heading (deg)", Math.toDegrees(heading));


        dashboard.sendTelemetryPacket(packet);
    }

}
