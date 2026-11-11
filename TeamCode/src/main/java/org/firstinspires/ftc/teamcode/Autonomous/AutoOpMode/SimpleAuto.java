package org.firstinspires.ftc.teamcode.Autonomous.AutoOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.SimplePath;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "SimpleAuto")
public class SimpleAuto extends LinearOpMode {

    private int currentPath;

    private Command followPath(Follower follower, Path path, int pathNumber) {
        return sequential(
                instant(() -> currentPath = pathNumber),
                // Ivy 1.1.1 defaults to the geometric endpoint; wait for Pedro to finish settling.
                follow(follower, path).setDone(() -> !follower.following())
        );
    }

    private Command autoRoutine(Follower follower, SimplePath paths) {
        return sequential(
                followPath(follower, paths.path1(), 1),
                followPath(follower, paths.path2(), 2),
                followPath(follower, paths.path3(), 3),
                followPath(follower, paths.path4(), 4),
                followPath(follower, paths.path5(), 5),
                followPath(follower, paths.penStroke(), 6)
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        currentPath = 0;
        Follower follower = Constants.create(hardwareMap);

        try {
            SimplePath simplePath = new SimplePath();
            follower.setPose(simplePath.path1().get(0.0));
            Command routine = autoRoutine(follower, simplePath);

            telemetry.addLine("SimpleAuto ready. Place robot at the starting pose.");
            telemetry.addData("Start X (inches)", "%.2f", follower.pose().x());
            telemetry.addData("Start Y (inches)", "%.2f", follower.pose().y());
            telemetry.addData("Start heading (degrees)", "%.1f",
                    Math.toDegrees(follower.pose().heading()));
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            follower.update();
            Scheduler.schedule(routine);
            while (opModeIsActive() && Scheduler.isScheduled(routine)) {
                follower.update();
                Scheduler.execute();
                telemetry.addData("Path", "%d / 6", currentPath);
                telemetry.addData("X (inches)", "%.2f", follower.pose().x());
                telemetry.addData("Y (inches)", "%.2f", follower.pose().y());
                telemetry.addData("Heading (degrees)", "%.1f",
                        Math.toDegrees(follower.pose().heading()));
                telemetry.update();
                idle();
            }
        } finally {
            try {
                Scheduler.reset();
            } finally {
                follower.stop();
                follower.drivetrain.stop();
            }
        }
    }
}
