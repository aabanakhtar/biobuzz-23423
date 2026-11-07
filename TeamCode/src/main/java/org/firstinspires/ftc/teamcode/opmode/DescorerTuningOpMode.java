package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.RobotConstants.DescorerTuning.DEPLOY_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.DescorerTuning.STORE_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.DESCORER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.subsystem.Descorer;

@Config
@TeleOp(name = "Tune: Descorer", group = "Tuning")
public class DescorerTuningOpMode extends OpMode {
    public static boolean down = false;
    private ServoEx actuator;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        actuator = new ServoEx(hardwareMap, DESCORER);
        actuator.setInverted(true);
    }

    @Override
    public void loop() {
        double position = down ? DEPLOY_POSITION : STORE_POSITION;
        actuator.set(position);
        telemetry.addData("Selected mode", down);
        telemetry.addData("Commanded position", position);
        telemetry.update();
    }
}
