package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.RobotConstants.ShooterTuning.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.Ashwing;
import org.firstinspires.ftc.teamcode.utilities.RunningAverageFilter;
import org.firstinspires.ftc.teamcode.utilities.SubsystemLooptimeAverager;

@Config
public class Shooter extends SubsystemBase {
    public enum Mode {
        RAW,
        FIXED,
        DYNAMIC
    }

    public static Mode mode = Mode.RAW;
    private final RunningAverageFilter velFilter = new RunningAverageFilter(VELOCITY_FILTER_SAMPLES);

    public static double targetVelocityTicks = 0.0;
    public static double targetRawPower = 0.0;
    public static double currentVelo = 0;

    private final PIDFController flywheelVelocityPID = new PIDFController(kP, kI, kD, 0);
    private final Ashwing robot = Ashwing.get();

    private final SubsystemLooptimeAverager averager = new SubsystemLooptimeAverager();

    private final InterpLUT distanceVelocityLookup = new InterpLUT();
    private final boolean velocityLookupReady;

    public Shooter() {
        velocityLookupReady = DISTANCE_VELOCITY_POINTS.length >= 2;
        if (velocityLookupReady) {
            for (double[] point : DISTANCE_VELOCITY_POINTS) {
                distanceVelocityLookup.add(point[0], point[1]);
            }
            distanceVelocityLookup.createLUT();
        }
        flywheelVelocityPID.setTolerance(VELOCITY_TOLERANCE);
    }

    @Override
    public void periodic() {
        averager.mark();

        if (tuning) {
            flywheelVelocityPID.setPIDF(kP, kI, kD, 0);
            flywheelVelocityPID.setTolerance(VELOCITY_TOLERANCE);
        }

        velFilter.updateValue(robot.shooterLeft.getCorrectedVelocity());
        currentVelo = velFilter.getFilteredOutput();

        switch (mode) {
            case RAW:
                rawMode();
                break;
            case FIXED:
                velocityMode();
                break;
            case DYNAMIC:
                dynamicMode();
                break;
            default:
                break;
        }

        averager.endMark();
    }

    public void setMode(Mode mode) {
        Shooter.mode = mode;
    }

    public Mode getMode() { return mode; }

    public double getOptimalVelocityForDist(double distance_ft) {
        return velocityLookupReady ? distanceVelocityLookup.get(distance_ft) : 0.0;
    }

    public void setVelocity(double velo) {
        mode = Mode.FIXED;
        targetVelocityTicks = velo;
    }

    public void setPower(double pow) {
        mode = Mode.RAW;
        targetRawPower = pow;
    }

    public void setIdle() {
        setVelocity(IDLE_VELOCITY);
    }

    private void velocityMode() {
        double currentVelocity = velFilter.getFilteredOutput();
        // voltage comp is necessary to prevent the bot from tweaking towards the end of the match
        double output = flywheelVelocityPID.calculate(currentVelocity, targetVelocityTicks) * robot.getVoltageFeedforwardConstant() +
                kV * targetVelocityTicks * robot.getVoltageFeedforwardConstant();

        Shooter.currentVelo = currentVelocity;
        robot.shooterLeft.set(-output);
        robot.shooterRight.set(-output);

        logData(currentVelocity);
    }

    private void dynamicMode() {
        if (!velocityLookupReady) {
            targetVelocityTicks = 0.0;
            robot.shooterLeft.set(0.0);
            robot.shooterRight.set(0.0);
            robot.flightRecorder.addLine("Shooter interpolation calibration required");
            return;
        }
        // clip to a distance between 1 and 15
        double distOffset = PREDICT_FACTOR * robot.drive.getRadialVelocityToGoal() * robot.getVoltageFeedforwardConstant();
        double distanceToGoal = Range.clip(robot.drive.getAimTarget().distance + distOffset, MIN_SHOT_DISTANCE_FT, MAX_SHOT_DISTANCE_FT);
        // use radial velo comp
        double optimalVelocityForDist = getOptimalVelocityForDist(distanceToGoal) + SHOT_OFFSET;
        targetVelocityTicks = optimalVelocityForDist;

        double currentVelocity = velFilter.getFilteredOutput();

        double output = flywheelVelocityPID.calculate(currentVelocity, optimalVelocityForDist) * robot.getVoltageFeedforwardConstant()
                + kV * optimalVelocityForDist * robot.getVoltageFeedforwardConstant();

        robot.shooterLeft.set(-output);
        robot.shooterRight.set(-output);

        robot.flightRecorder.addData("distance to goal", distanceToGoal);
        logData(currentVelocity);
    }

    private void rawMode() {
        robot.shooterLeft.set(targetRawPower);
        robot.shooterRight.set(targetRawPower);
    }

    private void logData(double velo) {

        double distanceToGoal = Range.clip(robot.drive.getAimTarget().distance, MIN_SHOT_DISTANCE_FT, MAX_SHOT_DISTANCE_FT);
        robot.flightRecorder.addData("distance to goal", distanceToGoal);
        robot.flightRecorder.addLine("========SHOOTER========");
        robot.flightRecorder.addData("Flywheel Target velocity", targetVelocityTicks);
        robot.flightRecorder.addData("Flywheel Current velocity", velo);
        robot.flightRecorder.addData("average ms", averager.getAvgMs());
    }

    public boolean isAtTargetVelocity() {
        return mode != Mode.RAW && (mode != Mode.DYNAMIC || velocityLookupReady)
                && flywheelVelocityPID.atSetPoint();
    }
}
