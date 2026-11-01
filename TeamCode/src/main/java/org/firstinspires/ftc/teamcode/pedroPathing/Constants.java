package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RobotConstants.Pedro;

/** Runtime factories. All editable calibration lives in RobotConstants.Pedro. */
public class Constants {
    public static OctoQuadLocalizer createLocalizer(HardwareMap hardwareMap) {
        return new OctoQuadLocalizer(hardwareMap, Pedro.localizerConfig);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                createLocalizer(hardwareMap),
                new Mecanum(hardwareMap, Pedro.drivetrainConfig),
                new Foresight(Pedro.foresightConfig)
        );
    }
}
