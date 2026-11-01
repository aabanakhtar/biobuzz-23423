package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.device.AbsoluteAnalogEncoder;

import java.util.List;

/** Reads only turret feedback. No robot container, scheduler, or actuator commands. */
@TeleOp(name = "Calibrate: Turret Encoders (Read Only)", group = "Tuning")
public class TurretEncoderCalibrationOpMode extends OpMode {
    private AbsoluteAnalogEncoder analogEncoder;
    private DcMotorEx quadraturePort;
    private List<LynxModule> hubs;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        analogEncoder = new AbsoluteAnalogEncoder(hardwareMap,
                RobotConstants.Hardware.TURRET_ANALOG_ENCODER,
                RobotConstants.Sensors.TURRET_ENCODER_OFFSET);
        quadraturePort = hardwareMap.get(DcMotorEx.class,
                RobotConstants.Sensors.TURRET_QUADRATURE_MOTOR_NAME);
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(LynxModule::clearBulkCache);
        reportEncoders();
    }

    @Override
    public void init_loop() {
        reportEncoders();
    }

    @Override
    public void loop() {
        reportEncoders();
    }

    private void reportEncoders() {
        // Earlier OpModes may have left hubs in manual bulk-caching mode.
        hubs.forEach(LynxModule::clearBulkCache);
        analogEncoder.offset = RobotConstants.Sensors.TURRET_ENCODER_OFFSET;
        telemetry.addData("Analog voltage (V)", analogEncoder.getVoltage());
        telemetry.addData("Analog angle (deg)", analogEncoder.getCurrentPosition());
        telemetry.addData("Quadrature ticks", quadraturePort.getCurrentPosition());
        telemetry.update();
    }
}
