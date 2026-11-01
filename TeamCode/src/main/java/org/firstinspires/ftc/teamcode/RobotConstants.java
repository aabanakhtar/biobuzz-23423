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
        public static String TURRET_LEFT = "turretLeft";
        public static String TURRET_RIGHT = "turretRight";
        public static String TURRET_ANALOG_ENCODER = "turretEncoder";
        public static String INTAKE_LEFT = "intakeLeft";
        public static String INTAKE_RIGHT = "intakeRight";
        public static String LATCH = "latch";
        public static boolean SHOOTER_LEFT_REVERSED = true;
        public static boolean SHOOTER_RIGHT_REVERSED = false;
        public static boolean TURRET_LEFT_REVERSED = false;
        public static boolean TURRET_RIGHT_REVERSED = true;
        public static boolean INTAKE_LEFT_REVERSED = false;
        public static boolean INTAKE_RIGHT_REVERSED = true;
    }

    @Config
    public static class IntakeTuning {
        public static double INGEST_MOTOR_SPEED = 0.0;
        public static double DISCARD_MOTOR_SPEED = 0.0;
        public static double INTAKE_LATCH_OPEN_POSITION = 0.0;
        public static double INTAKE_LATCH_CLOSE_POSITON = 0.0;
    }

    @Config
    public static class ShooterTuning {
        // Rows are {distance in feet, flywheel velocity in ticks/second}, in increasing distance order.
        // Add at least two measured points, then reinitialize the robot to rebuild interpolation.
        public static double[][] DISTANCE_VELOCITY_POINTS = {};
        public static int VELOCITY_FILTER_SAMPLES = 3;
        public static double MIN_SHOT_DISTANCE_FT = 1.0;
        public static double MAX_SHOT_DISTANCE_FT = 15.0;
        // Enable to apply Dashboard PID/tolerance edits each loop.
        public static boolean tuning = false;
        public static double IDLE_VELOCITY = 0.0;
        public static double kV = 0.0;
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double VELOCITY_TOLERANCE = 0.0;
        public static double PREDICT_FACTOR = 0.0; // Shooting-on-the-move radial compensation.
        public static double SHOT_OFFSET = 0.0;
    }

    @Config
    public static class TurretTuning {
        // Enable to apply Dashboard PID/tolerance edits each loop.
        public static boolean tuning = false;
        public static double PREDICT_FACTOR = 0.0;
        public static double offset_angle = 0.0;
        public static double kS = 0.0;
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double GEAR_RATIO = 0.0; // encoder-shaft rotations per turret rotation
        public static double ENCODER_PPR = 0.0;
        public static double TURRET_ENCODER_CPR = ENCODER_PPR * GEAR_RATIO; // Configure for the new encoder and gearing.
        public static double TURRET_MAX_ANGLE = 85; // deg
        public static double TURRET_PID_TOLERANCE = 0.0; //deg
        public static double TURRET_SAFE_ZONE = 165;
    }

    @Config
    public static class Aiming {
        public static double VELOCITY_COMPENSATION_DEADBAND = 3.0; // inches/second
        public static double TURRET_OFFSET = 0.0;
        public static double BLUE_UPPER_HIVE_X = 85.0;
        public static double BLUE_UPPER_HIVE_Y = 85.0;
        public static final double FIELD_MIDLINE = 72.0;
    }

    @Config
    public static class Sensors {
        public static double MIN_FEEDFORWARD_VOLTAGE = 9.0;
        public static int BATTERY_FILTER_SAMPLES = 3;
        public static final double IDEAL_VOLTAGE = 12.5;
        public static double TURRET_ENCODER_OFFSET = 0.0;
        // Motor whose encoder socket receives the turret's quadrature cable.
        // This selects feedback only; turret motion is driven by the CR servo pair.
        // Change this independently to any configured motor name before initialization.
        public static String TURRET_QUADRATURE_MOTOR_NAME = "intakeLeft";
    }

    public static class Pedro {
        // https://shop.swyftrobotics.com/products/swyft-linear-odometry-module
        // Official example: https://cdn.shopify.com/s/files/1/0875/8486/6598/files/swyft_encoder_gobilda_pinpoint_example_code.java?v=1764901678
        public static final double SWYFT_COUNTS_PER_REV = 4096.0;
        public static final double SWYFT_WHEEL_DIAMETER_MM = 38.0;
        public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
            c.frontLeftName.set("lf");
            c.backLeftName.set("lr");
            c.frontRightName.set("rf");
            c.backRightName.set("rr");

            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

            c.powerThreshold.set(0.0);
            c.manualBrakeMode.set(false);
        });

        // Run the OctoQuad Custom-pod tuner; replace this block with its calibrated output.
        public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
            c.name.set("octoquad");
            c.xPodPort.set(0); // Forward pod; change to match the physical OctoQuad port.
            c.yPodPort.set(1); // Strafe pod; change to match the physical OctoQuad port.
            c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0); // Generated by OctoQuad AutoTune.
            c.yPodOffset.set(0.0); // Generated by OctoQuad AutoTune.
            c.offsetUnits.set(DistanceUnit.INCH);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.encoderResolutionUnit.set(DistanceUnit.INCH);
            // SWYFT's official example uses 4096 / (PI * 38 mm); validate with a measured push.
            c.ticksPerUnit.set(SWYFT_COUNTS_PER_REV / (Math.PI * SWYFT_WHEEL_DIAMETER_MM / 25.4));
            c.headingScalar.set(1.0);
            c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        });

        public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
            // The end constraints (timeoutConstraint etc.) that decide when FollowPathCommand and HoldPointCommand finish
            // also go here, see https://pedropathing.com/docs/pathing/reference/endconstraints

            Controller primaryTranslationalForward = Controller.proportional(0.0);
            Controller secondaryTranslationalForward = Controller.proportional(0.0);
            Controller primaryTranslationalLateral = Controller.proportional(0.0);
            Controller secondaryTranslationalLateral = Controller.proportional(0.0);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(0.0, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(0.0, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.0));
            c.brake.set(Controller.proportionalFeedforward(0.0));

            c.headingFeedback.set(Controller.proportional(0.0));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));

            c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0, 0.0));

            c.maxAchievableForwardVelocity.set(0.0);
            c.maxAchievableStrafeVelocity.set(0.0);
            c.naturalForwardDeceleration.set(0.0);
            c.naturalStrafeDeceleration.set(0.0);
        });

    }
}
