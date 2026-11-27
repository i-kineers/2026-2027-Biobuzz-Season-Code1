package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

    @TeleOp(name = "MelonboticsEncoderTest", group = "Calibration")
    public class MelonboticsEncoderTest extends OpMode {

        private DcMotorEx flywheel;
        private DcMotorEx shooterEncoder;

        private VoltageSensor batteryVoltageSensor;

        // Verify SDK ticks per revolution before trusting RPM.
        private static final double ENCODER_TICKS_PER_REV = 4112.0;

        @Override
        public void init() {

            // Shooter motor
            flywheel = hardwareMap.get(
                    DcMotorEx.class, "flywheel"
            );

            // Independent Melonbotics quadrature encoder
            shooterEncoder = hardwareMap.get(
                    DcMotorEx.class, "shooterEncoder"
            );

            batteryVoltageSensor =
                    hardwareMap.voltageSensor.iterator().next();

            // Configure shooter motor
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.FLOAT
            );

            // Configure external encoder
            shooterEncoder.setPower(0);
            shooterEncoder.setMode(
                    DcMotor.RunMode.STOP_AND_RESET_ENCODER
            );
            shooterEncoder.setMode(
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER
            );

            telemetry.addLine("Melonbotics Encoder Initialized");
            telemetry.addData("Battery Voltage",
                    batteryVoltageSensor.getVoltage());
            telemetry.update();
        }

        @Override
        public void loop() {

            // Manually control flywheel with left stick
            double motorPower = -gamepad1.left_stick_y;

            flywheel.setPower(motorPower);

            // Read independent encoder
            int encoderPosition =
                    shooterEncoder.getCurrentPosition();

            double encoderVelocity =
                    shooterEncoder.getVelocity();

            // Convert to RPM (verify ticks/revolution first)
            double rpm =
                    (encoderVelocity / ENCODER_TICKS_PER_REV) * 60.0;

            // Telemetry
            telemetry.addLine("=== FLYWHEEL TEST ===");

            telemetry.addData("Motor Power", motorPower);

            telemetry.addLine("--- MELONBOTICS ENCODER ---");
            telemetry.addData("Raw Encoder TPS",
                    encoderVelocity);

            telemetry.addData("Calculated RPM",
                    rpm);

            telemetry.addData("Encoder Position",
                    encoderPosition);

            telemetry.addData("Encoder Position",
                    encoderPosition);

            telemetry.addData("Encoder Velocity (TPS)",
                    encoderVelocity);

            telemetry.addData("Flywheel RPM (provisional)",
                    rpm);

            telemetry.addData("Battery Voltage",
                    batteryVoltageSensor.getVoltage());
           
            telemetry.addData("Raw Encoder TPS",
                    shooterEncoder.getVelocity());

            telemetry.update();
        }

        @Override
        public void stop() {
            flywheel.setPower(0);
        }
    }
