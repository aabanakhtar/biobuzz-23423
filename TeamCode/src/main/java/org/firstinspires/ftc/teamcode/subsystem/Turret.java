package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.RobotConstants.TurretTuning.*;
import static org.firstinspires.ftc.teamcode.RobotConstants.Sensors.TURRET_ENCODER_OFFSET;

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
    private double TURRET_QUAD_HOME_OFFSET = 0;
    private double quadratureZeroPosition = 0;
    private static double lastAngle = 0;
    private final SubsystemLooptimeAverager averager = new SubsystemLooptimeAverager();

    public Turret() {
        turretAnglePID.setTolerance(TURRET_PID_TOLERANCE);
    }

    public void loadAngle(double angle) {
        TURRET_QUAD_HOME_OFFSET = angle;
        // Software zero avoids resetting a motor encoder that may serve another mechanism.
        quadratureZeroPosition = robot.turretQuadratureEncoder.getCurrentPosition();
        lastAngle = angle;
        turretAnglePID.reset();
    }

    public static double getLastAngle() {
        return lastAngle;
    }

    public double getAverageLoopMs() {
        return averager.getAvgMs();
    }

    @Override
    public void periodic() {
        averager.mark();
        robot.flightRecorder.addLine("<b>Turret</b>");

        mode = pendingMode;
        robot.turretEncoder.offset = TURRET_ENCODER_OFFSET;
        final double absAngle = robot.turretEncoder.getCurrentPosition();

        if (mode == Mode.DEBUG) {
            lastAngle = absAngle;
        } else {
            final double rawQuad = robot.turretQuadratureEncoder.getCurrentPosition() - quadratureZeroPosition;
            if (Math.abs(absAngle) <= TURRET_SAFE_ZONE) {
                double rawQuadAngle = (rawQuad / TURRET_ENCODER_CPR) * 360.0;
                TURRET_QUAD_HOME_OFFSET = absAngle - rawQuadAngle;
            }
            lastAngle = calculateAngleFromEncoder();
        }

        robot.flightRecorder.addData("Turret angle (deg)", lastAngle);

        if (tuning) {
            turretAnglePID.setPIDF(kP, kI, kD, 0);
            turretAnglePID.setTolerance(TURRET_PID_TOLERANCE);
        }

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
                robot.flightRecorder.addData("Tracking target (deg)", rawTarget);

                // constrain our angles
                double constrainedAngleDeg = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, compensatedTarget));
                double power = turretAnglePID.calculate(lastAngle, constrainedAngleDeg);
                power += Math.signum(power) * kS;

                if ((power > 0 && lastAngle > TURRET_MAX_ANGLE) || (power < 0 && lastAngle < -TURRET_MAX_ANGLE)) {
                    setPower(0.0);
                    break;
                }

                setPower(power);
                break;
            }

            // Debug purposes only
            case DEBUG: {
                double power = turretAnglePID.calculate(absAngle, targetAngle);
                power += Math.signum(power) * kS;
                robot.flightRecorder.addData("debug power", power);
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
        return TURRET_QUAD_HOME_OFFSET + (encoderPos / TURRET_ENCODER_CPR) * 360.0;
    }
}
