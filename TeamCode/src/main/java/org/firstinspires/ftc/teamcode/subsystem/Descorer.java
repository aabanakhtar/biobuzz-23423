package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.RobotConstants.DescorerTuning.DEPLOY_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.DescorerTuning.STORE_POSITION;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.Ashwing;
import org.firstinspires.ftc.teamcode.utilities.SubsystemLooptimeAverager;

public class Descorer extends SubsystemBase {
    private final SubsystemLooptimeAverager averager = new SubsystemLooptimeAverager();

    public enum Mode {
        DEPLOY,
        STORE
    }

    public static Mode mode = Mode.STORE;

    public void setMode(Mode mode) {
        Descorer.mode = mode;
    }

    public double getAverageLoopMs() {
        return averager.getAvgMs();
    }

    @Override
    public void periodic() {
        averager.mark();
        double position = mode == Mode.DEPLOY ? DEPLOY_POSITION : STORE_POSITION;
        Ashwing robot = Ashwing.get();
        robot.descorerServo.set(position);
        robot.flightRecorder.addLine("<b>Descorer</b>");
        robot.flightRecorder.addData("Descorer mode", mode);
        robot.flightRecorder.addData("Descorer commanded position", position);
        averager.endMark();
    }
}
