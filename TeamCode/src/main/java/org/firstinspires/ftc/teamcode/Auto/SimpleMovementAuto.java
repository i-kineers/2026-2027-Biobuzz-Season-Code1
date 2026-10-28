//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.pedro.Constants;
//import static com.pedropathing.api.Paths.*;
//import com.pedropathing.paths.Path;
//import com.pedropathing.ivy.Scheduler;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.pedro.PedroCommands.follow;
//import com.pedropathing.ivy.Command;
//
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//
//@Autonomous(name = "AutoPath", group = "Autonomous")
//public class SimpleMovementAuto extends OpMode {
//    private Follower follower; // you added this before
//    private final PoseFactory p = PoseFactory.degrees(); // add this
//    private final Pose startPose = p.of(8, 80, 90);
//    private final Pose Park1= p.of(8, 100, 90);
//    private final Pose park2 = p.of(8, 95, 90);
//    //poses from before
//
//    private Path StartTOpark() {
//        return line(startPose, Park1).linear(startPose, Park1);
//    }
//    private Path ParkTOpark() {
//        return line(Park1, park2).linear(Park1, park2);
//    }
//    private Command autoRoutine() {
//        return sequential(
//                follow(follower, StartTOpark()),
//                // Add mechanism commands here.
//                follow(follower, ParkTOpark())
//        );
//    }
//    @Override
//    public void init() {
//        Scheduler.reset();
//        follower = Constants.create(hardwareMap);
//        follower.setPose(startPose);
//        follower.update();
//    }
//
//    @Override
//    public void start() {
//        schedule(autoRoutine());
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        Scheduler.execute();
//
//        telemetry.addData("X", follower.pose().x());
//        telemetry.addData("Y", follower.pose().y());
//        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
//        telemetry.addData("Follower Mode", follower.mode());
//        telemetry.update();
//    }
//}