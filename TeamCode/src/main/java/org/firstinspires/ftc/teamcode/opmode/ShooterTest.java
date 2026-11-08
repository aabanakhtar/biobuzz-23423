package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Shooter Test", group = "Tuning")
public class ShooterTest extends OpMode {
    private DcMotor shooterLeft;
    private DcMotor shooterRight;

    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    @Override
    public void init() {
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intake1");
        intakeRight = hardwareMap.get(DcMotor.class, "intake2");
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
    }

    @Override
    public void loop() {
        double power = gamepad1.right_trigger;
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
        if (gamepad1.a) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        } else if (gamepad1.b) {
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }
        telemetry.addData("Shooter power", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
    }
}
