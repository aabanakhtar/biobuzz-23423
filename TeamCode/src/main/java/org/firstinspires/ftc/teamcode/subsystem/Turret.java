package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.RobotConstants.TurretTuning.*;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.robot.Ashwing;
import org.firstinspires.ftc.teamcode.utilities.SubsystemLooptimeAverager;

@Config
public class Turret extends SubsystemBase {
    private final Ashwing robot = Ashwing.get();

    public enum Mode {
        HOMING,
        RAW,
        TRACKING,
        DEBUG,
    }

    // State machine related things
    public static Mode mode = Mode.RAW;
    public static Mode pendingMode = Mode.RAW;
    public static Mode lastMode = Mode.RAW;

    public static double targetPower = 0.0;
    public static double targetAngle = 0.0;

    private final PIDFController turretAnglePID = new PIDFController(kP, kI, kD, 0);

    // for relocalizing turret
    private double TURRET_HOME_OFFSET = 0;
    private double quadratureZeroPosition = 0;
    private static double lastAngle = 0;
    private final SubsystemLooptimeAverager averager = new SubsystemLooptimeAverager();

    public Turret() {
        turretAnglePID.setTolerance(TURRET_PID_TOLERANCE);
    }

    public void loadAngle(double angle) {
        TURRET_HOME_OFFSET = angle;
        // Software zero avoids resetting a motor encoder that may serve another mechanism.
        quadratureZeroPosition = robot.turretQuadratureEncoder.getCurrentPosition();
        lastAngle = angle;
        turretAnglePID.reset();
    }

    public static double getLastAngle() {
        return lastAngle;
    }

    @Override
    public void periodic() {
        averager.mark();
        TURRET_ENCODER_CPR = ENCODER_PPR * GEAR_RATIO;
        if (!Double.isFinite(TURRET_ENCODER_CPR) || TURRET_ENCODER_CPR <= 0) {
            setPower(0.0);
            robot.flightRecorder.addLine("Turret encoder calibration required");
            averager.endMark();
            return;
        }

        final double absAngle = robot.turretEncoder.getCurrentPosition();
        final double rawQuad = (robot.turretQuadratureEncoder.getCurrentPosition() - quadratureZeroPosition);
        // always use quadrature
        boolean isOutOfSafeRange = Math.abs(absAngle) > TURRET_SAFE_ZONE;

        // ensure that we're safe, not moving, etc.
        if (!isOutOfSafeRange) {
            double rawQuadAngle =
                    (rawQuad / TURRET_ENCODER_CPR) * 360.0;
            TURRET_HOME_OFFSET = absAngle - rawQuadAngle;
        }

        final double quadratureAngle = calculateAngleFromEncoder();
        lastAngle = quadratureAngle;

        robot.flightRecorder.addLine("==========TURRET===========");
        robot.flightRecorder.addData("quadrature angle", quadratureAngle);
        robot.flightRecorder.addData("average ms", averager.getAvgMs());

        if (tuning) {
            turretAnglePID.setPIDF(kP, kI, kD, 0);
            turretAnglePID.setTolerance(TURRET_PID_TOLERANCE);
        }

        // update the queued mode
        mode = pendingMode;
        switch (mode) {
            case RAW:
                setPower(targetPower);
                break;

            // Stop for one cycle before enabling automatic aiming.
            case HOMING: {
                setPower(0.0);
                pendingMode = Mode.TRACKING;
                break;
            }

            // Aim from the current field pose and compensate for motion.
            case TRACKING: {
                if (lastMode != Mode.TRACKING) {
                    turretAnglePID.reset();
                }

                double predictedLeadOffset = Math.toDegrees(robot.drive.getTangentVelocityToGoal() * PREDICT_FACTOR * robot.getVoltageFeedforwardConstant());

                // filter our target
                double rawTarget = robot.drive.getAimTarget().heading;
                double compensatedTarget = rawTarget - predictedLeadOffset;
                robot.flightRecorder.addData("TARGET", rawTarget);

                // constrain our angles
                double constrainedAngleDeg = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, compensatedTarget + offset_angle));
                double power = turretAnglePID.calculate(quadratureAngle, constrainedAngleDeg);
                power += Math.signum(power) * kS;

                if ((power > 0 && quadratureAngle > TURRET_MAX_ANGLE) || (power < 0 && quadratureAngle < -TURRET_MAX_ANGLE) || turretAnglePID.atSetPoint()) {
                    setPower(0.0);
                    break;
                }

                setPower(power);
                break;
            }

            // Debug purposes only
            case DEBUG: {
                double power = turretAnglePID.calculate(quadratureAngle, targetAngle);
                setPower(power);
                break;
            }
        }

        // store this for smooth transitions between the two (rising-edge)
        lastMode = mode;
        averager.endMark();
    }

    private void setPower(double power) {
        double output = Double.isFinite(power) ? Math.max(-1.0, Math.min(1.0, power)) : 0.0;
        // Apply the travel limit in every mode, including raw and debug commands.
        if ((output > 0 && lastAngle >= TURRET_MAX_ANGLE)
                || (output < 0 && lastAngle <= -TURRET_MAX_ANGLE)) {
            output = 0.0;
        }
        robot.turretLeft.set(output);
        robot.turretRight.set(output);
    }

    public void setMode(Mode amode) {
        pendingMode = amode;
    }

    public Mode getMode() {
        return mode;
    }

    public boolean isAtTarget() {
        return turretAnglePID.atSetPoint();
    }

    private double calculateAngleFromEncoder() {
        // results in 90 <---- 0 -----> -90
        double encoderPos = (robot.turretQuadratureEncoder.getCurrentPosition() - quadratureZeroPosition);
        return TURRET_HOME_OFFSET + (encoderPos / TURRET_ENCODER_CPR) * 360.0;
    }
}
