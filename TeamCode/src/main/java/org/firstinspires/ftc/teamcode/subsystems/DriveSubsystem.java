package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {

    // Set these to match the physical Control Hub mounting on your robot.
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private final IMU imu;
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;

    // Separate rampers for each type of movement
    private final InputRamper yRamper;
    private final InputRamper xRamper;
    private final InputRamper rxRamper;

    public DriveSubsystem(HardwareMap hardwareMap) {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "flmotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blmotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frmotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "brmotor");

        imu = hardwareMap.get(IMU.class, "imu");
        if (!imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION)))) {
            throw new IllegalStateException("Drive IMU failed to initialize");
        }
        resetHeading();


        // Brake when power = 0
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Same motor direction setup as your old subsystem
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        yRamper = new InputRamper();
        xRamper = new InputRamper();
        rxRamper = new InputRamper();
    }


    public void drive(Gamepad gamepad) {

        if (gamepad.optionsWasPressed()) {
            resetHeading();
        }

        // Negative because pushing the FTC stick forward gives a negative Y value
        double forward = gamepad.left_stick_y;

        double strafe = -gamepad.left_stick_x;

        double turn = -gamepad.right_stick_x;


        // Ramp the driver's inputs
        double fieldY = yRamper.rampInput(forward);
        double fieldX = xRamper.rampInput(strafe);
        double rx = rxRamper.rampInput(turn);

        // Rotate the field-relative translation into the robot's coordinate frame.
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x = fieldX * Math.cos(heading) + fieldY * Math.sin(heading);
        double y = -fieldX * Math.sin(heading) + fieldY * Math.cos(heading);

        // Standard mecanum wheel calculations
        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;


        // Normalize if any motor would exceed power 1.0
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        frontLeftPower /= denominator;
        backLeftPower /= denominator;
        frontRightPower /= denominator;
        backRightPower /= denominator;


        setMotorPowers(
                frontLeftPower,
                backLeftPower,
                frontRightPower,
                backRightPower
        );
    }


    private void setMotorPowers(double frontLeft, double backLeft, double frontRight, double backRight) {
        frontLeftMotor.setPower(frontLeft);
        backLeftMotor.setPower(backLeft);
        frontRightMotor.setPower(frontRight);
        backRightMotor.setPower(backRight);
    }


    public void stop() {
        setMotorPowers(0, 0, 0, 0);

        yRamper.reset();
        xRamper.reset();
        rxRamper.reset();
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    private static class InputRamper {

        private final double MIN_MULTIPLIER = 0.4;
        private final double MAX_MULTIPLIER = 1.0;

        private final double increment = 0.4;

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
