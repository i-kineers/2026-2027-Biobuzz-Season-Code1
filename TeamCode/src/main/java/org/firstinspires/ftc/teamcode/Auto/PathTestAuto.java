
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
import org.firstinspires.ftc.teamcode.subsystems.AutoMasterLogic;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "PathTestAuto", group = "Autonomous")
public class PathTestAuto extends OpMode {
    private Follower follower; // you added this before
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(30.0018, 12.6511, 225);
    private final Pose point1 = poseFactory.of(11.1603, 12.8326, 180);
    private final Pose point1Control1 = poseFactory.of(21.4729, 16.0333, 0);
    private final Pose point2Start = poseFactory.of(11.1603, 12.8326, 180);
    private final Pose point2 = poseFactory.of(55.5605, 128.3078, 105);
    private final Pose point2Control1 = poseFactory.of(12.0878, 62.1314, 0);
    private final Pose point3Start = poseFactory.of(55.5605, 128.3078, 105);
    private final Pose point3 = poseFactory.of(14.3272, 40.43, 180);
    private final Pose point4Start = poseFactory.of(14.3272, 40.43, 180);
    private final Pose point4 = poseFactory.of(35.4742, 14.25, 225);
    private final Pose point5Start = poseFactory.of(35.4742, 14.25, 225);
    private final Pose point5 = poseFactory.of(15.4029, 96.3808, 180);
    public Path path1() {
        return Paths.curve(start, point1Control1, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
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
