package org.firstinspires.ftc.teamcode.meet0.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class driveSubsystem {

    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;

    // Separate rampers for each type of movement
    private final InputRamper yRamper;
    private final InputRamper xRamper;
    private final InputRamper rxRamper;

    public driveSubsystem(HardwareMap hardwareMap) {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "flmotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blmotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frmotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "brmotor");


        // Brake when power = 0
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Same motor direction setup as your old subsystem
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        yRamper = new InputRamper();
        xRamper = new InputRamper();
        rxRamper = new InputRamper();
    }


    public void drive(Gamepad gamepad) {

        // Negative because pushing the FTC stick forward gives a negative Y value
        double forward = -gamepad.left_stick_y;

        double strafe = gamepad.left_stick_x;

        double turn = gamepad.right_stick_x;


        // Ramp the driver's inputs
        double y = yRamper.rampInput(forward);
        double x = xRamper.rampInput(strafe);
        double rx = rxRamper.rampInput(turn);


        // Standard mecanum wheel calculations
        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;


        // Normalize if any motor would exceed power 1.0
        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rx),
                1.0
        );

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