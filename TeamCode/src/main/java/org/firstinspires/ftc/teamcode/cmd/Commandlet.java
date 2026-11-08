package org.firstinspires.ftc.teamcode.cmd;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.Ashwing;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

import java.util.function.BooleanSupplier;

/*
Static class containing commonly used commands
 */
@Config
public class Commandlet {
    private static final Ashwing robot = Ashwing.get();
    public static long TRANSFER_DELAY = (long) 1200.0;

    public static Command If(Command toRunTrue, Command toRunFalse, BooleanSupplier b) {
        return new ConditionalCommand(toRunTrue, toRunFalse, b);
    }

    public static Command go(Follower follower, Path p, double maxPow) {
        return new FollowPathCommand(follower, p, maxPow);
    }

    public static Command run(Runnable r) {
        return new InstantCommand(r);
    }

    public static Command intakeSet(Intake.Mode mode) {
        return run(() -> robot.intake.setMode(mode));
    }

    public static void updateIntake(boolean discard, boolean slowIngest, boolean ingest,
                                    boolean shooting) {
        if (shooting) {
            return;
        }
        if (discard) {
            robot.intake.setMode(Intake.Mode.DISCARD);
        } else if (slowIngest) {
            robot.intake.setMode(Intake.Mode.SLOW_INGEST);
        } else if (ingest) {
            robot.intake.setMode(Intake.Mode.INGEST);
        } else {
            robot.intake.setMode(Intake.Mode.OFF);
        }
    }

    public static Command shootTeleOp() {
        return new SequentialCommandGroup(
                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                run(() -> robot.intake.openLatch()),
                waitFor(800),
                intakeSet(Intake.Mode.INGEST),
                waitFor(TRANSFER_DELAY),
                intakeSet(Intake.Mode.OFF),
                // turn off after doing everything
                run(() -> Ashwing.get().shooter.setIdle()),
                waitFor(200),
                run(() -> robot.intake.closeLatch())
        );
    }


    public static Command shoot(long transfer_delay) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    // open the latch
                    waitFor((long)RobotConstants.IntakeTuning.INTAKE_LATCH_DELAY),
                    run(() -> robot.intake.openLatch())
                ),
                // run the intake
                new ParallelCommandGroup(
                    intakeSet(Intake.Mode.INGEST),
                    waitFor(transfer_delay)
                ),
                // turn off after doing everything
                run(() -> Ashwing.get().shooter.setIdle()),
                intakeSet(Intake.Mode.OFF),
                run(() -> robot.intake.closeLatch())
        );
    }

    public static Command shootFar(long transfer_delay) {
        // TODO: use distance sensors to gauge success
        return new SequentialCommandGroup(
                run(() -> RobotConstants.IntakeTuning.INGEST_MOTOR_SPEED = 0.8),
                new ParallelCommandGroup(
                        // open the latch
                        waitFor((long) RobotConstants.IntakeTuning.INTAKE_LATCH_DELAY),
                        run(() -> robot.intake.openLatch())
                ),
                // run the intake
                new ParallelCommandGroup(
                        intakeSet(Intake.Mode.INGEST),
                        waitFor(transfer_delay)
                ),
                // turn off after doing everything
                run(() -> Ashwing.get().shooter.setIdle()),
                intakeSet(Intake.Mode.OFF),
                run(() -> robot.intake.closeLatch()),
                run(() -> RobotConstants.IntakeTuning.INGEST_MOTOR_SPEED = 1)
        );
    }

    public static Command fork(Command a, Command b) {
        return new ParallelCommandGroup(a, b);
    }

    public static Command waitFor(long duration_ms) {
        return new WaitCommand(duration_ms);
    }

    public static Command nothing() {
        return run(() -> {});
    }
}
