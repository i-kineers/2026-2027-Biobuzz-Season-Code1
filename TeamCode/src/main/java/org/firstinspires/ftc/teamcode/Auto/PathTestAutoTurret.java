
package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "PathTestAutoTurret", group = "Autonomous")
public class PathTestAutoTurret extends OpMode {
    private Follower follower; // you added this before
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(30.8415, 6.9406, 180);
    private final Pose point1 = poseFactory.of(9.6487, 7.96, 180);
    private final Pose point1Control1 = poseFactory.of(23.8242, 7.2997, 0);
    private final Pose point2Start = poseFactory.of(9.6487, 6.6184, 180);
    private final Pose point2 = poseFactory.of(46.4911, 129.6514, 90);
    private final Pose point2Control1 = poseFactory.of(23.8445, 101.6002, 0);
    private final Pose point3Start = poseFactory.of(46.4911, 129.6514, 90);
    private final Pose point3 = poseFactory.of(14.3121, 45.993, 180);
    private final Pose point3Control1 = poseFactory.of(23.2886, 88.3898, 0);
    private final Pose point4Start = poseFactory.of(14.3121, 45.993, 180);
    private final Pose point4 = poseFactory.of(30.4101, 7.2809, 180);
    private final Pose point5Start = poseFactory.of(30.4101, 7.2809, 180);
    private final Pose point5 = poseFactory.of(11.1433, 95.9682, 180);
    public Path path1() {
        return Paths.curve(start, point1Control1, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                waitMs(2000),
                follow(follower, path2()),
                waitMs(2000),
                follow(follower, path3()),
                waitMs(2000),
                follow(follower, path4()),
                waitMs(2000),
                follow(follower, path5())
        );
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
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
