package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.RobotConstants.Pedro;

/** Runtime factories. All editable calibration lives in RobotConstants.Pedro. */
public class Constants {
    public static OctoQuadLocalizer createLocalizer(HardwareMap hardwareMap) {
        return new OctoQuadLocalizer(hardwareMap, Pedro.localizerConfig);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        // Pedro's Mecanum constructor sets directions but does not set run modes.
        // Explicitly restore open-loop drive, including after an encoder reset.
        for (String name : new String[] {
                Pedro.drivetrainConfig.frontLeftName.get(),
                Pedro.drivetrainConfig.backLeftName.get(),
                Pedro.drivetrainConfig.frontRightName.get(),
                Pedro.drivetrainConfig.backRightName.get()
        }) {
            hardwareMap.get(DcMotor.class, name).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return new Follower(
                createLocalizer(hardwareMap),
                new Mecanum(hardwareMap, Pedro.drivetrainConfig),
                new Foresight(Pedro.foresightConfig)
        );
    }
}
