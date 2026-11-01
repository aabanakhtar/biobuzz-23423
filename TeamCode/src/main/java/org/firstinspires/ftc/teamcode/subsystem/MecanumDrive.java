package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.RobotConstants.Aiming.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
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
    private SubsystemLooptimeAverager averager = new SubsystemLooptimeAverager();
    private Ashwing robot = Ashwing.get();

    public Command resetHeading() {
        return new InstantCommand(() -> follower.setHeading(0), this);
    }

    public double getRadialVelocityToGoal() {
        Pose robotPose = getPose();
        Vector2D robotPosition = getTurretPosition(robotPose, TURRET_OFFSET);
        Vector2D goalPosition = selectHiveTarget(robotPose, Ashwing.alliance).toVector2D();

        // Calculate vector from robot to goal
        Vector2D toGoal = goalPosition.minus(robotPosition);

        // Calculate angle to goal (field-relative)
        double angleToGoalField = Math.atan2(toGoal.y(), toGoal.x());

        // Get robot velocity scalar
        double robotSpeed = getVelocity().magnitude();

        if (robotSpeed < VELOCITY_COMPENSATION_DEADBAND) {
            return 0;
        }

        double robotVelocityAngle = Math.atan2(getVelocity().y(), getVelocity().x());

        // Calculate tangential component
        double deltaTheta = AngleUnit.normalizeRadians(robotVelocityAngle - angleToGoalField);
        return robotSpeed * Math.cos(deltaTheta);
    }

    public double getTangentVelocityToGoal() {
        Pose robotPose = getPose();
        Vector2D robotPosition = getTurretPosition(robotPose, TURRET_OFFSET);
        Vector2D goalPosition = selectHiveTarget(robotPose, Ashwing.alliance).toVector2D();

        // Calculate vector from robot to goal
        Vector2D toGoal = goalPosition.minus(robotPosition);

        // Calculate angle to goal (field-relative)
        double angleToGoalField = Math.atan2(toGoal.y(), toGoal.x());

        // Get robot velocity scalar
        double robotSpeed = getVelocity().magnitude();

        if (robotSpeed < VELOCITY_COMPENSATION_DEADBAND) {
            return 0;
        }

        double robotVelocityAngle = Math.atan2(getVelocity().y(), getVelocity().x());

        // Calculate tangential component
        double deltaTheta = AngleUnit.normalizeRadians(robotVelocityAngle - angleToGoalField);
        return robotSpeed * Math.sin(deltaTheta);
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
    // Approximate aim point supplied by the team; measure and refine on the field.

    public MecanumDrive(HardwareMap map, Pose startingPose) {
        this.follower = Constants.createFollower(map);
        follower.setPose(startingPose);
        follower.update();
    }

    @Override
    public void periodic() {
        averager.mark();
        follower.update();
        lastPose = follower.pose();

        Pose hiveTarget = selectHiveTarget(lastPose, Ashwing.alliance);
        lastAimTarget = calculateAimTarget(lastPose, hiveTarget, TURRET_OFFSET);
        robot.flightRecorder.addData("Hive target", lastPose.y() > FIELD_MIDLINE ? "upper" : "lower");
        robot.flightRecorder.addData("Hive X", hiveTarget.x());
        robot.flightRecorder.addData("Hive Y", hiveTarget.y());
        robot.flightRecorder.addLine("======DRIVETRAIN:=======");
        robot.flightRecorder.addData("goal heading", lastAimTarget.heading);
        robot.flightRecorder.addData("X:", lastPose.x());
        robot.flightRecorder.addData("Y:", lastPose.y());
        robot.flightRecorder.addData("Heading", Math.toDegrees(lastPose.heading()));
        robot.flightRecorder.addData("avg ms", averager.getAvgMs());

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

    public Vector2D getVelocity() {
        return follower.velocity().toVector2D();
    }

    public AimAtTarget getAimTarget() {
        return lastAimTarget;
    }

    /**
     * Field origin is bottom left. Above Y=72 selects upper; on or below selects lower.
     * Red reflects across X=72, preserving which vertical half the robot occupies.
     */
    public static Pose selectHiveTarget(Pose robotPose, Ashwing.Alliance alliance) {
        PoseFactory factory = PoseFactory.radians();
        if (alliance == Ashwing.Alliance.RED) {
            factory = factory.mirrorX(FIELD_MIDLINE);
        }
        if (robotPose.y() <= FIELD_MIDLINE) {
            factory = factory.mirrorY(FIELD_MIDLINE);
        }
        return factory.of(BLUE_UPPER_HIVE_X, BLUE_UPPER_HIVE_Y, 0.0);
    }

    public static Vector2D getTurretPosition(Pose robotPose, double turretOffset) {
        return Vector2D.cartesian(
                robotPose.x() + turretOffset * Math.cos(robotPose.heading()),
                robotPose.y() + turretOffset * Math.sin(robotPose.heading()));
    }

    /** Offset is signed inches along robot forward; distance is feet, heading is robot-relative degrees. */
    public static AimAtTarget calculateAimTarget(Pose robotPose, Pose goal, double turretOffset) {
        double heading = robotPose.heading();
        Vector2D turretPosition = getTurretPosition(robotPose, turretOffset);
        double dx = goal.x() - turretPosition.x();
        double dy = goal.y() - turretPosition.y();
        double distanceFeet = Math.hypot(dx, dy) / 12.0;
        double relativeHeading = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - heading);
        return new AimAtTarget(distanceFeet, Math.toDegrees(relativeHeading));
    }
}
