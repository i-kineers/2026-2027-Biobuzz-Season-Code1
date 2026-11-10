
package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "BlueAutoPath", group = "Autonomous")
public class BlueSideSimpleAuto extends OpMode {
    private Follower follower; // you added this before
    private final PoseFactory poseFactory = PoseFactory.degrees(); // add this
    private final Pose startPose = poseFactory.of(33.8647, 8.1163, 90);
    private final Pose ShootPoint = poseFactory.of(57.5672, 9.866,  90);
    private final Pose point1Control1 = poseFactory.of(20.9925, 39.0948, 0);
    private final Pose park2 = poseFactory.of(8, 100, 90);
    //poses from before

    private Path StartTOshoot() {
        return curve(startPose,point1Control1, ShootPoint).linear(startPose, ShootPoint);
    }
    private Path ShootTOpark() {
        return line(ShootPoint, park2).linear(ShootPoint, park2);
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, StartTOshoot()),
                // Add mechanism commands here.
                follow(follower, ShootTOpark())
        );
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
