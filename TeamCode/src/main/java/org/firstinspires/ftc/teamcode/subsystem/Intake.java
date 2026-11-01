package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeTuning.*;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.Ashwing;

@Configurable
public class Intake extends SubsystemBase {
    public enum Mode {
        INGEST,
        DISCARD,
        OFF
    }

    public static Mode mode = Mode.OFF;

    public static double intakeLatchTargetPos = INTAKE_LATCH_OPEN_POSITION;

    private void setMotorPower(double power) {
        Ashwing robot = Ashwing.get();
        robot.intakeLeft.set(power);
        robot.intakeRight.set(power);
    }

    public void setMode(Mode mode) {
        Intake.mode = mode;
    }

    @Override
    public void periodic() {
        Ashwing robot = Ashwing.get();
        // update latch
        robot.latchServo.set(intakeLatchTargetPos);

        switch (mode) {
            case INGEST:
                setMotorPower(INGEST_MOTOR_SPEED * robot.getVoltageFeedforwardConstant());
                break;

            case DISCARD:
                setMotorPower(DISCARD_MOTOR_SPEED * robot.getVoltageFeedforwardConstant());
                break;

            case OFF:
                setMotorPower(0.0);
                break;

            default:
                break;
        }
    }

    public void openLatch() {
        intakeLatchTargetPos = INTAKE_LATCH_OPEN_POSITION;
    }

    public void closeLatch() {
        intakeLatchTargetPos = INTAKE_LATCH_CLOSE_POSITON;
    }
}
