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
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
@Autonomous(name = "movementAuto", group = "Autonomous")
public class MovementAuto extends OpMode {
    private Follower follower; // you added this before
    private final PoseFactory poseFactory = PoseFactory.degrees(); // add this
    private final Pose startPOSE = poseFactory.of(9.007712570779052, 82.68725060709151, 270);
    private final Pose point1Control1 = poseFactory.of(32.92, 97.21, 180);
    private final Pose park = poseFactory.of(13.24, 92.26, 270);
    private final Pose point1Control2 = poseFactory.of(33.76, 107.79, 180);
    private final Pose park1 = poseFactory.of(13.24, 125.86, 270);
    //poses from before
    private String selectedAuto = "No Auto Selected";

    private Path StartTOpark() {
        return curve(startPOSE,point1Control1, park).linear(startPOSE, park);
    }
    private Path StartTOpark1() {
        return curve(startPOSE,point1Control2, park1).linear(startPOSE, park1);
    }
    private Command autoRoutine1() {
        return sequential(
                follow(follower, StartTOpark())
        );
    }
    private Command autoRoutine2() {
        return sequential(
                follow(follower, StartTOpark1())
        );
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPOSE);
        follower.update();

        schedule(autoRoutine1());
        selectedAuto = "Auto Routine 1, this one goes to right side of loading zone";

        telemetry.addData("Selected Auto", selectedAuto);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if(gamepad1.xWasPressed()){
            schedule(autoRoutine2());
            selectedAuto = "Auto Routine 2, this one goes to left side of loading zone";
        }
        telemetry.addData("Selected Auto", selectedAuto);
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Selected Auto", selectedAuto);
        telemetry.update();
    }
}
