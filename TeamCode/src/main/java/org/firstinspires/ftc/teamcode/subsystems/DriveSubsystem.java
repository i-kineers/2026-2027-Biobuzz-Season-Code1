package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSubsystem {

    // Localization heading follows the intake (robot front). The shooter faces the rear.
    // Intake heading 90 degrees => shooter heading 270 degrees, toward the hive at start.
    private static final double SHOOTER_OFFSET = Math.toRadians(180.0);
    private final Controller headingController;
    private final Follower follower;
    private double driverHeadingOffset = 0.0;
    private boolean autoAimEnabled = false;

    // Keep the existing ramp-up behavior for forward, lateral, and turn input.
    private final InputRamper yRamper = new InputRamper();
    private final InputRamper xRamper = new InputRamper();
    private final InputRamper rxRamper = new InputRamper();

    // Must use the same field coordinate frame as the localization pose.
    private static Pose rightGoalPose;
    private static Pose leftGoalPose;

    public DriveSubsystem(HardwareMap hardwareMap, boolean isBlue, double startPoseX, double startPoseY, double heading) {

        if (isBlue) {
            rightGoalPose = new Pose(84.17, 81.76);
            leftGoalPose = new Pose(84.17, 54.76);
        } else {
            rightGoalPose = new Pose(58.14, 60.56);
            leftGoalPose = new Pose(58.02, 81.77);
        }

        follower = Constants.create(hardwareMap);
        headingController = Constants.foresightConfig.headingFeedback.get();
        follower.setPose(new Pose(startPoseX, startPoseY, Math.toRadians(heading)));
    }

    public void drive(Gamepad gamepad) {
        if (gamepad.backWasPressed()) {
            resetHeading();
        }
        if (gamepad.rightBumperWasPressed()) {
            autoAimEnabled = !autoAimEnabled;
            headingController.reset();
            rxRamper.reset();
        }

        // Use the joystick conventions from Pedro's TeleOp guide.
        double forward = yRamper.rampInput(gamepad.left_stick_y);
        double lateral = xRamper.rampInput(gamepad.left_stick_x);
        double turn = autoAimEnabled ? autoAim() : rxRamper.rampInput(-gamepad.right_stick_x);

        DrivePowers powers = ManualDrive.fieldCentric(-forward, -lateral, turn, follower.pose().heading() - driverHeadingOffset);

        // Apply translation and the selected turn input without entering position hold.
        follower.manual(powers);
        follower.update();
    }

    /** Returns turn power for aiming; the caller must apply it as the drive turn input. */
    public double autoAim() {
        Pose robot = follower.pose();

        Pose currentGoalPose = robot.y() < 70.5 ? leftGoalPose : rightGoalPose;

        double dx = currentGoalPose.x() - robot.x();
        double dy = currentGoalPose.y() - robot.y();
        if (Math.hypot(dx, dy) < 1e-6) {
            headingController.reset();
            return 0.0;
        }

        double angleToGoal = Math.atan2(dy, dx);
        double targetHeading = Angle.normalize(angleToGoal + SHOOTER_OFFSET);

        double currentHeading = robot.heading();
        double error = Angle.normalizeSigned(targetHeading - currentHeading);

        // Match Foresight's signed heading braking displacement (omega is radians/sec).
        Vector2D brakeCoefficients = Constants.foresightConfig.headingBrakeCoefficients.get();
        double angularVelocity = follower.velocity().omega;
        double brakingDisplacement = brakeCoefficients.x() * angularVelocity + brakeCoefficients.y() * angularVelocity * Math.abs(angularVelocity);
        double predictedHeading = currentHeading + brakingDisplacement;
        double brakingError = Angle.normalizeSigned(targetHeading - predictedHeading);

        // Stop correcting only when both the current and predicted stopping headings are close.
        // Keep braking when momentum would carry the shooter past the goal.
        if (Math.abs(error) < Math.toRadians(1.5)
                && Math.abs(brakingError) < Math.toRadians(1.5)) {
            headingController.reset();
            return 0.0;
        }

        // Pedro 3 takes the target and the signed error as separate arguments.
        double turnPower = headingController.calculate(targetHeading, brakingError);

        // Ramp down near target for precision
        double maxTurn = Math.abs(error) < Math.toRadians(10) ? 0.2 : 0.4;
        return Math.max(-maxTurn, Math.min(turnPower, maxTurn));
    }

    public void stop() {
        follower.stop();
        // stop() selects IDLE; stop the motors immediately without waiting for another update.
        follower.drivetrain.stop();
        yRamper.reset();
        xRamper.reset();
        rxRamper.reset();
        headingController.reset();
        autoAimEnabled = false;
    }

    /** Resets the driver's field-centric reference without changing the localization pose. */
    public void resetHeading() {
        driverHeadingOffset = follower.pose().heading();
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
