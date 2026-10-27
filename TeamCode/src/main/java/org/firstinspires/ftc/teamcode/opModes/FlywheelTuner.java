package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Flywheel Tuner", group = "Calibration")
public class FlywheelTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        // Let PID control the motor throughout tuning; keep the calibrated F values.
        shooter.DANGER_THRESHOLD = 0.0;

        try {
            telemetry.addLine("GP1: D-pad Up/Down = P | Right/Left = I | Y/X = D");
            telemetry.addLine("Start/Options = off/on | RT = shoot | Same PID for idle and shooting");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.dpadUpWasPressed()) {
                    shooter.kP += 0.0001;
                }
                if (gamepad1.dpadDownWasPressed()) {
                    shooter.kP = Math.max(0.0, shooter.kP - 0.0001);
                }
                if (gamepad1.dpadRightWasPressed()) {
                    shooter.kI += 0.00001;
                }
                if (gamepad1.dpadLeftWasPressed()) {
                    shooter.kI = Math.max(0.0, shooter.kI - 0.00001);
                }
                if (gamepad1.yWasPressed()) {
                    shooter.kD += 0.000001;
                }
                if (gamepad1.xWasPressed()) {
                    shooter.kD = Math.max(0.0, shooter.kD - 0.000001);
                }

                shooter.update(gamepad1);

                telemetry.addData("Shooter state", shooter.getShooterState());
                telemetry.addData("P (D-pad Up/Down)", "%.7f", shooter.kP);
                telemetry.addData("I (D-pad Right/Left)", "%.7f", shooter.kI);
                telemetry.addData("D (Y/X)", "%.7f", shooter.kD);
                telemetry.addData("Idle speed TPS", "%.0f", shooter.IDLE_TARGET_TPS);
                telemetry.addData("Shooting speed TPS", "%.0f", shooter.SHOOTING_TARGET_TPS);
                telemetry.addData("Target TPS", "%.0f", shooter.getTargetTPS());
                telemetry.addData("Actual TPS", "%.0f", shooter.getVelocity());
                telemetry.addLine("One PID set for both speeds. Record P/I/D and copy them into ShooterSubsystem.");
                telemetry.update();
                idle();
            }
        } finally {
            shooter.stop();
        }
    }
}
