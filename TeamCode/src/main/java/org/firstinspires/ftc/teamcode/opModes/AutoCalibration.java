package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Calibrates the production single-motor shooter at two applied voltages. */
@TeleOp(name = "Flywheel kF Calibration", group = "Calibration")
public class AutoCalibration extends LinearOpMode {

    private static final double VOLTAGE_REFERENCE = 12.0;
    private static final double TEST_SECONDS = 12.0;
    private static final double SAMPLE_SECONDS = 2.0;

    private DcMotorEx flywheel;
    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() {
        // Match the hardware, direction, and encoder mode used by ShooterSubsystem.
        Servo gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(0.0);
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheel.setPower(0.0);

        try {
            flywheel.setDirection(DcMotor.Direction.FORWARD);
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

            telemetry.addLine("--- Flywheel Calibration Tool ---");
            telemetry.addLine("Ensure the wheel is clear and use a fresh battery.");
            telemetry.addLine("Press START to begin sequence.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            CalibrationResult low = performTest(4.0, "LOW SPEED");
            if (low == null) return;

            telemetry.addLine("Cooling down for 3 seconds...");
            telemetry.update();
            sleep(3000);
            if (!opModeIsActive()) return;

            CalibrationResult high = performTest(9.0, "HIGH SPEED");
            if (high == null) return;
            if (high.tps <= low.tps) {
                calibrationError("High-speed TPS must exceed low-speed TPS. Check the encoder and battery.");
                return;
            }

            while (opModeIsActive()) {
                telemetry.addLine("--- CALIBRATION RESULTS ---");
                telemetry.addData("kF_LOW", "%.6f", low.kF);
                telemetry.addData("LOW_TARGET_TPS", "%.1f", low.tps);
                telemetry.addData("kF_HIGH", "%.6f", high.kF);
                telemetry.addData("HIGH_TARGET_TPS", "%.1f", high.tps);
                telemetry.addLine("Copy all four calibration values into ShooterSubsystem.");
                telemetry.addLine("Press STOP to exit.");
                telemetry.update();
                idle();
            }
        } finally {
            flywheel.setPower(0.0);
            gate.setPosition(0.0);
        }
    }

    /** Measures the last two seconds after spin-up; an aborted test produces no result. */
    private CalibrationResult performTest(double targetVolts, String label) {
        ElapsedTime timer = new ElapsedTime();
        double tpsSum = 0.0;
        int samples = 0;

        try {
            while (opModeIsActive() && timer.seconds() < TEST_SECONDS) {
                double voltage = batteryVoltageSensor.getVoltage();
                if (Double.isNaN(voltage) || Double.isInfinite(voltage) || voltage < targetVolts) {
                    calibrationError("Battery voltage is invalid or too low for " + label + ".");
                    return null;
                }

                // Reject an unreachable voltage above rather than calibrating clipped power.
                flywheel.setPower(targetVolts / voltage);
                double tps = flywheel.getVelocity();
                if (Double.isNaN(tps) || Double.isInfinite(tps)) {
                    calibrationError("Encoder velocity is invalid.");
                    return null;
                }
                if (timer.seconds() >= TEST_SECONDS - SAMPLE_SECONDS) {
                    tpsSum += tps;
                    samples++;
                }

                telemetry.addData("Phase", label);
                telemetry.addData("Target Volts", targetVolts);
                telemetry.addData("Battery Volts", voltage);
                telemetry.addData("Current TPS", tps);
                telemetry.update();
                idle();
            }

            if (!opModeIsActive()) return null;
            double averageTPS = samples == 0 ? 0.0 : tpsSum / samples;
            if (averageTPS <= 0.0) {
                calibrationError("No positive encoder speed measured. Check the encoder and motor direction.");
                return null;
            }

            return new CalibrationResult(averageTPS,
                    (targetVolts / VOLTAGE_REFERENCE) / averageTPS);
        } finally {
            flywheel.setPower(0.0);
        }
    }

    private void calibrationError(String message) {
        telemetry.addData("Calibration failed", message);
        telemetry.update();
    }

    private static class CalibrationResult {
        final double tps;
        final double kF;

        CalibrationResult(double tps, double kF) {
            this.tps = tps;
            this.kF = kF;
        }
    }
}
