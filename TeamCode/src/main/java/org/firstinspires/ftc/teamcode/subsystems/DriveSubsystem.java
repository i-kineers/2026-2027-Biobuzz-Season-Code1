package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSubsystem {

    private final Follower follower;
    private final AimingSubsystem aiming;
    private double driverHeadingOffset = 0.0;

    // Keep the existing ramp-up behavior for forward, lateral, and turn input.
    private final InputRamper yRamper = new InputRamper();
    private final InputRamper xRamper = new InputRamper();
    private final InputRamper rxRamper = new InputRamper();

    public DriveSubsystem(HardwareMap hardwareMap, boolean isBlue, double startPoseX, double startPoseY, double heading) {
        VisionSubsystem vision = new VisionSubsystem(hardwareMap);
        follower = Constants.create(hardwareMap);
        aiming = new AimingSubsystem(vision, Constants.foresightConfig.headingFeedback.get(), isBlue);
        follower.setPose(new Pose(startPoseX, startPoseY, Math.toRadians(heading)));
    }

    public void drive(Gamepad gamepad) {
        if (gamepad.backWasPressed()) {
            resetHeading();
        }
        if (gamepad.rightBumperWasPressed()) {
            aiming.toggle();
            rxRamper.reset();
        }

        Pose robot = follower.pose();
        aiming.update(robot);

        // Use the joystick conventions from Pedro's TeleOp guide.
        double forward = yRamper.rampInput(gamepad.left_stick_y);
        double lateral = xRamper.rampInput(gamepad.left_stick_x);
        double turn = aiming.isEnabled()
                ? aiming.calculateTurnPower(robot, follower.velocity().omega)
                : rxRamper.rampInput(-gamepad.right_stick_x);

        DrivePowers powers = ManualDrive.fieldCentric(-forward, -lateral, turn, robot.heading() - driverHeadingOffset);

        // Apply translation and the selected turn input without entering position hold.
        follower.manual(powers);
        follower.update();
    }

    public void stop() {
        follower.stop();
        follower.drivetrain.stop();
        yRamper.reset();
        xRamper.reset();
        rxRamper.reset();
        aiming.stop();
    }

    /** Restores the configured reset pose and clears the aiming correction. */
    public void resetHeading() {
        follower.setPose(new Pose(6.59, 133.88, Math.toRadians(270)));
        aiming.resetTracking();
    }

    public Pose getPose() {
        return follower.pose();
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(Angle.normalize(follower.pose().heading()));
    }

    public boolean isHolding() {
        return follower.holding();
    }

    public void addTelemetry(Telemetry telemetry) {
        Pose robot = follower.pose();
        telemetry.addData("Drive", isHolding() ? "Holding position" : "Field-centric");
        telemetry.addLine("GP1 Back/Share: reset driving reference");

        telemetry.addLine("===== LOCALIZATION =====");
        telemetry.addData("Robot X (inches)", "%.1f", robot.x());
        telemetry.addData("Robot Y (inches)", "%.1f", robot.y());
        telemetry.addData("Heading (degrees)", "%.1f", getHeadingDegrees());

        aiming.addTelemetry(telemetry, robot, follower.velocity().omega);
    }

    private static class InputRamper {

        private final double MIN_MULTIPLIER = 0.4;
        private final double MAX_MULTIPLIER = 1.0;

        private final double increment = 0.2;

        private final double timeIncrementInMs = 100;

        private final ElapsedTime rampTimer = new ElapsedTime();

        private double multiplier = MIN_MULTIPLIER;


        public InputRamper() {
            rampTimer.reset();
        }


        public double rampInput(double input) {

            if (Math.abs(input) > 0.05) {

                if (rampTimer.milliseconds() > timeIncrementInMs) {

                    multiplier += increment;

                    rampTimer.reset();
                }

            } else {

                multiplier = MIN_MULTIPLIER;

                rampTimer.reset();

                return 0.0;
            }


            multiplier = Math.max(
                    MIN_MULTIPLIER,
                    Math.min(multiplier, MAX_MULTIPLIER)
            );

            return input * multiplier;
        }


        public void reset() {

            multiplier = MIN_MULTIPLIER;

            rampTimer.reset();
        }
    }
}
