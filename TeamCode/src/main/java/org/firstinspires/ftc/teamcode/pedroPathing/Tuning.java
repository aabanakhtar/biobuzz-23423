package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import org.firstinspires.ftc.teamcode.RobotConstants.Pedro;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;

/**
 * Pedro Pathing 3.0 AutoTune procedures.
 * <p>
 * Deploy this code, connect to the robot's Wi-Fi and open http://192.168.43.1:10158 in a browser to run them
 * (see https://pedropathing.com/docs/pathing/tuning). Paste the configs that AutoTune generates into
 * {@link org.firstinspires.ftc.teamcode.RobotConstants.Pedro}.
 * <p>
 * The active procedures use a mecanum drivetrain and an OctoQuad MK2 with one forward
 * and one strafe pod. Run the OctoQuad procedure first, paste its generated config into
 * RobotConstants.Pedro, then tune the drivetrain and Foresight algorithm.
 */
public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure octoquadTuner() {
        return new OctoQuadTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                Constants::createLocalizer,
                hardwareMap -> new Mecanum(hardwareMap, Pedro.drivetrainConfig)
        );
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Pedro.drivetrainConfig),
                Constants::createLocalizer,
                () -> new Foresight(Pedro.foresightConfig)
        );
    }
}
