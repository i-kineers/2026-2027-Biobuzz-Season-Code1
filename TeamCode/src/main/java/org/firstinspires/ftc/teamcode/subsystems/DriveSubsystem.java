package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSubsystem {

    private final Follower follower;
    private double driverHeadingOffset = 0.0;

    // Keep the existing ramp-up behavior for forward, lateral, and turn input.
    private final InputRamper yRamper = new InputRamper();
    private final InputRamper xRamper = new InputRamper();
    private final InputRamper rxRamper = new InputRamper();

    public DriveSubsystem(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(56.0, 36.0, Math.toRadians(270.0)));
    }

    public void drive(Gamepad gamepad) {
        if (gamepad.backWasPressed()) {
            resetHeading();
        }

        // Use the joystick conventions from Pedro's TeleOp guide.
        double forward = yRamper.rampInput(gamepad.left_stick_y);
        double lateral = xRamper.rampInput(gamepad.left_stick_x);
        double turn = rxRamper.rampInput(-gamepad.right_stick_x);

        DrivePowers powers = ManualDrive.fieldCentric(
                forward, lateral, turn, follower.pose().heading() - driverHeadingOffset);

        // The ramper already filters stick noise, so any remaining input exits holding.
        // Retain Pedro's default 0.2 inches/second threshold for entering a hold.
        ManualDrive.driveOrHold(follower, powers, 0.0, 0.2);
        follower.update();
    }

    public void stop() {
        follower.stop();
        // stop() selects IDLE; stop the motors immediately without waiting for another update.
        follower.drivetrain.stop();
        yRamper.reset();
        xRamper.reset();
        rxRamper.reset();
    }

    /** Resets the driver's field-centric reference without changing the localization pose. */
    public void resetHeading() {
        driverHeadingOffset = follower.pose().heading();
    }

    public Pose getPose() {
        return follower.pose();
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(follower.pose().heading());
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
