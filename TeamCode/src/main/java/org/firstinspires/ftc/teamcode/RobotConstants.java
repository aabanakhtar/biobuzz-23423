package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Edit robot hardware, mechanism tuning, aiming, and Pedro configuration here.
 * PID/feedforward values remain zero until calibrated. Modes, targets and telemetry stay in subsystems.
 * Mechanism tuning groups are exposed in FTC Dashboard. Reinitialize after hardware or Pedro config edits.
 */
public final class RobotConstants {
    private RobotConstants() {}

    @Config
    public static class Hardware {
        public static String SHOOTER_LEFT = "shooterLeft";
        public static String SHOOTER_RIGHT = "shooterRight";
        public static String TURRET_LEFT = "turret1";
        public static String TURRET_RIGHT = "turret2";
        public static String TURRET_ANALOG_ENCODER = "turretAbs";
        public static String INTAKE_LEFT = "intake1";
        public static String INTAKE_RIGHT = "intake2";
        public static String LATCH = "latch";
        public static String DESCORER = "descorer";
        public static boolean SHOOTER_LEFT_REVERSED = true;
        public static boolean SHOOTER_RIGHT_REVERSED = false;
        public static boolean TURRET_LEFT_REVERSED = true;
        public static boolean TURRET_RIGHT_REVERSED = true;
        public static boolean INTAKE_LEFT_REVERSED = false;
        public static boolean INTAKE_RIGHT_REVERSED = true;
        public static boolean DESCORER_REVERSED = true;
    }

    @Config
    public static class IntakeTuning {
        public static double SLOW_INGEST_MULTIPLIER = 0.6;
        public static double INTAKE_LATCH_DELAY = 150.0; // milliseconds
        public static double INGEST_MOTOR_SPEED = 1.0;
        public static double DISCARD_MOTOR_SPEED = -1.0;
        public static double INTAKE_LATCH_OPEN_POSITION = 0.0;
        public static double INTAKE_LATCH_CLOSE_POSITON = 0.0;
    }

    @Config
    public static class DescorerTuning {
        public static double DEPLOY_POSITION = 0.42;
        public static double STORE_POSITION = 0.0;
    }

    @Config
    public static class TeleOpTuning {
        public static double DRIVE_SPEED = 1.0;
        public static double TURN_SPEED = 0.7;
        // Starting values carried from DECODE; tune for this drivetrain.
        public static double HEADING_LOCK_TARGET_DEG = 165.0;
        public static double HEADING_kP = 0.7;
        public static double HEADING_kI = 0.0;
        public static double HEADING_kD = 0.07;
        // Field coordinates, in inches and degrees, used when USE_LAST_POSE is false.
        public static boolean USE_LAST_POSE = true;
        public static double START_X = 0.0;
        public static double START_Y = 0.0;
        public static double START_HEADING_DEG = 0.0;
    }

    @Config
    public static class ShooterTuning {
        // Rows are {distance in feet, flywheel velocity in ticks/second}, in increasing distance order.
        // Placeholder velocities: replace with measured values, then reinitialize to rebuild interpolation.
        public static double[][] DISTANCE_VELOCITY_POINTS = {
                {0, 0}, {1, 0}, {2, 0}, {3, 0},
                {4, 0}, {5, 0}, {6, 0}, {7, 0},
                {8, 0}, {9, 0}, {10, 0}, {11, 0},
                {12, 0}, {13, 0}, {14, 0}, {15, 0}
        };
        public static int VELOCITY_FILTER_SAMPLES = 3;
        public static double MIN_SHOT_DISTANCE_FT = 0.0;
        public static double MAX_SHOT_DISTANCE_FT = 15.0;
        // Enable to apply Dashboard PID/tolerance edits each loop.
        public static boolean tuning = false;
        public static double IDLE_VELOCITY = 0.0;
        public static double kV = 0.0;
        public static double kP = 0.0;
        public static double kI = -0.005;
        public static double kD = -0.00038;
        public static double VELOCITY_TOLERANCE = 0.0;
        public static double SHOT_OFFSET = 0.0;
    }

    @Config
    public static class TurretTuning {
        // Enable to apply Dashboard PID/tolerance edits each loop.
        public static boolean tuning = false;
        public static double kS = 0;
        public static double kP = 0.0155000067;
        public static double kI = 0.0;
        public static double kD = 0.0003;
        public static double GEAR_RATIO = 1; // encoder-shaft rotations per turret rotation
        public static double ENCODER_PPR = 4000.0;
        public static double TURRET_ENCODER_CPR = ENCODER_PPR * GEAR_RATIO; // Configure for the new encoder and gearing.
        public static double TURRET_MAX_ANGLE = 165; // deg
        public static double TURRET_PID_TOLERANCE = 0.0; //deg
        public static double TURRET_SAFE_ZONE = 175;
    }

    @Config
    public static class Aiming {
        public static double TARGET_X = 0.0;
        public static double TARGET_Y = 0.0;
        public static double TURRET_OFFSET = 0.0;
    }

    @Config
    public static class Sensors {
        public static double MIN_FEEDFORWARD_VOLTAGE = 9.0;
        public static int BATTERY_FILTER_SAMPLES = 3;
        public static final double IDEAL_VOLTAGE = 12.5;
        public static double TURRET_ENCODER_OFFSET = -114.4444444; // Absolute encoder angle at turret zero.
        // Motor whose encoder socket receives the turret's quadrature cable.
        // This selects feedback only; turret motion is driven by the CR servo pair.
        // Change this independently to any configured motor name before initialization.
        public static String TURRET_QUADRATURE_MOTOR_NAME = "frontLeft";
    }

    public static class Pedro {
        // https://shop.swyftrobotics.com/products/swyft-linear-odometry-module
        // Official example: https://cdn.shopify.com/s/files/1/0875/8486/6598/files/swyft_encoder_gobilda_pinpoint_example_code.java?v=1764901678
        public static final double SWYFT_COUNTS_PER_REV = 4096.0;
        public static final double SWYFT_WHEEL_DIAMETER_MM = 38.0;
        public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
            c.frontLeftName.set("frontLeft");
            c.backLeftName.set("backLeft");
            c.frontRightName.set("frontRight");
            c.backRightName.set("backRight");

            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

            c.manualBrakeMode.set(false);
        });

        // Run the OctoQuad Custom-pod tuner; replace this block with its calibrated output.
        public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
            c.name.set("octoquad");
            c.xPodPort.set(1);
            c.yPodPort.set(0);
            c.ticksPerUnit.set(863.0);
            c.xPodOffset.set(-3.0118110236220472);
            c.yPodOffset.set(-6.535433070866143);
            c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
            c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
            c.headingScalar.set(1.0251925925074825);
        });


        public static ForesightConfig foresightConfig = new ForesightConfig(
                c -> {
                    Controller primaryTranslationalForward = Controller.proportional(0.8578487635687276);
                    Controller secondaryTranslationalForward = Controller.proportional(0.31695219599751917);
                    Controller primaryTranslationalLateral = Controller.proportional(3.8998092863708047);
                    Controller secondaryTranslationalLateral = Controller.proportional(1.4408753264907128);

                    c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                    c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                    c.coast.set(Controller.proportionalFeedforward(0.02351254238809543));
                    c.brake.set(Controller.proportionalFeedforward(0.019985661029881117));

                    c.headingFeedback.set(Controller.proportional(16.097172981232497));
                    c.headingBrakeCoefficients.set(Vector2D.cartesian(0.030934149835709503, 0.006769229206772925));

                    c.linearBrakeCoefficients.set(Matrix.diag(0.03497176983080196, 0.019992395296432694));
                    c.quadraticBrakeCoefficients.set(Matrix.diag(0.002140917581560157, 0.002255185682361066));

                    c.maxAchievableForwardVelocity.set(64.45037764281078);
                    c.maxAchievableStrafeVelocity.set(50.42337088596487);
                    c.naturalForwardDeceleration.set(83.78884252853261);
                    c.naturalStrafeDeceleration.set(108.30375940124887);
                }
        );

    }
}
