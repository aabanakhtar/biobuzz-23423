package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.RobotConstants.Aiming.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Ashwing;
import org.firstinspires.ftc.teamcode.utilities.SubsystemLooptimeAverager;

@Config
public class MecanumDrive extends SubsystemBase {
    private final SubsystemLooptimeAverager averager = new SubsystemLooptimeAverager();
    private Ashwing robot = Ashwing.get();

    /** Relocalize at the field origin using the existing heading-reset control. */
    public Command resetHeading() {
        return new InstantCommand(() -> follower.setPose(new Pose(0, 0, 0)), this);
    }

    public static class AimAtTarget {
        public double distance;
        public double heading;

        public AimAtTarget(double distance, double heading) {
            this.distance = distance;
            this.heading = heading;
        }
    }

    public AimAtTarget lastAimTarget = new AimAtTarget(0, 0);

    public Follower follower;
    public static Pose lastPose = new Pose(0, 0, 0);

    public MecanumDrive(HardwareMap map, Pose startingPose) {
        this.follower = Constants.createFollower(map);
        follower.setPose(startingPose);
        follower.update();
    }

    public double getAverageLoopMs() {
        return averager.getAvgMs();
    }

    @Override
    public void periodic() {
        averager.mark();
        follower.update();
        lastPose = follower.pose();

        Pose target = new Pose(TARGET_X, TARGET_Y, 0.0);
        double normalizedHeading = AngleUnit.normalizeRadians(lastPose.heading());
        Pose normalizedPose = new Pose(lastPose.x(), lastPose.y(), normalizedHeading);
        lastAimTarget = calculateAimTarget(normalizedPose, target, TURRET_OFFSET);
        robot.flightRecorder.addLine("<b>Drivetrain</b>");
        robot.flightRecorder.addData("Aim target X (in)", target.x());
        robot.flightRecorder.addData("Aim target Y (in)", target.y());
        robot.flightRecorder.addData("Aim heading (deg)", lastAimTarget.heading);
        robot.flightRecorder.addData("Robot X (in)", lastPose.x());
        robot.flightRecorder.addData("Robot Y (in)", lastPose.y());
        robot.flightRecorder.addData("Robot heading (deg)", Math.toDegrees(normalizedHeading));

        averager.endMark();
    }

    public void setTeleOpDrive(double forward, double strafe, double rotation) {
        follower.manual(forward, strafe, rotation);
    }

    /**
     * Get the current estimated robot pose
     */
    public Pose getPose() {
        return follower.pose();
    }

    public AimAtTarget getAimTarget() {
        return lastAimTarget;
    }

    /** Offset is signed inches along robot forward; distance is feet, heading is robot-relative degrees. */
    public static AimAtTarget calculateAimTarget(Pose robotPose, Pose goal, double turretOffset) {
        double heading = AngleUnit.normalizeRadians(robotPose.heading());
        double turretX = robotPose.x() + turretOffset * Math.cos(heading);
        double turretY = robotPose.y() + turretOffset * Math.sin(heading);
        double dx = goal.x() - turretX;
        double dy = goal.y() - turretY;
        double distanceFeet = Math.hypot(dx, dy) / 12.0;
        // Pedro positive is counterclockwise; the turret encoder is positive clockwise.
        double relativeHeading = AngleUnit.normalizeRadians(heading - Math.atan2(dy, dx));
        return new AimAtTarget(distanceFeet, Math.toDegrees(relativeHeading));
    }
}
