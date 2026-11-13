
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
import org.firstinspires.ftc.teamcode.subsystems.AutoMasterLogic;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "MainAutonomous", group = "Autonomous")
public class MainAutonomous extends OpMode {
    private Follower follower; // you added this before
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private AutoMasterLogic Automaster;
    private final PoseFactory poseFactory = PoseFactory.degrees(); // add this
    private final Pose startshootPOSE = poseFactory.of(61.5768539587185, 8.6201844484233, 270);
    private final Pose point1Control1 = poseFactory.of(23.18, 18.27, 180);
    private final Pose park2 = poseFactory.of(13.24, 92.26, 270);
    //poses from before

    private Path ShootTOpark() {
        return curve(startshootPOSE,point1Control1, park2).linear(startshootPOSE, park2);
    }
    private Command autoRoutine() {
        return sequential(
                shooter.autoOPMODEShoot(2000),
                Automaster.autoOPMODEIntakeShooter(7000, 8000),
                waitMs(500),
                follow(follower, ShootTOpark())
        );
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startshootPOSE);
        follower.update();
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        Automaster = new AutoMasterLogic(hardwareMap, true);
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
