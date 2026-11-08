package org.firstinspires.ftc.teamcode.Autonomous.AutoOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.SimplePath;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "SimpleAuto")
public class SimpleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.create(hardwareMap);

        try {
            SimplePath simplePath = new SimplePath();
            Path[] paths = {
                    simplePath.path1(),
                    simplePath.path2(),
                    simplePath.path3(),
                    simplePath.path4(),
                    simplePath.path5(),
                    simplePath.penStroke()
            };
            follower.setPose(paths[0].get(0.0));

            telemetry.addLine("SimpleAuto ready. Place robot at the starting pose.");
            telemetry.addData("Start X (inches)", "%.2f", follower.pose().x());
            telemetry.addData("Start Y (inches)", "%.2f", follower.pose().y());
            telemetry.addData("Start heading (degrees)", "%.1f",
                    Math.toDegrees(follower.pose().heading()));
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            follower.update();
            for (int i = 0; i < paths.length && opModeIsActive(); i++) {
                follower.follow(paths[i]);
                while (opModeIsActive() && follower.following()) {
                    follower.update();
                    telemetry.addData("Path", "%d / %d", i + 1, paths.length);
                    telemetry.addData("X (inches)", "%.2f", follower.pose().x());
                    telemetry.addData("Y (inches)", "%.2f", follower.pose().y());
                    telemetry.addData("Heading (degrees)", "%.1f",
                            Math.toDegrees(follower.pose().heading()));
                    telemetry.update();
                    idle();
                }
            }
        } finally {
            follower.stop();
            follower.drivetrain.stop();
        }
    }
}
