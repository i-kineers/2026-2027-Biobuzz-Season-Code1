package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Flywheel Tuner", group = "Calibration")
public class FlywheelTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        Telemetry tunerTelemetry = panels.getWrapper();
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        // Let PID control the motor throughout tuning; keep the calibrated F values.
        shooter.DANGER_THRESHOLD = 0.0;

        double changeTPS = 20;

        try {
            tunerTelemetry.addLine("GP1: D-pad Up/Down = P | Right/Left = I | Y/X = D");
            tunerTelemetry.addLine("Start/Options = off/on | RT = shoot | Same PID for idle and shooting");
            tunerTelemetry.addLine("GP1 LB/RB = shooting target -/+20 TPS (minimum 20 TPS)");
            panels.update(telemetry);
            waitForStart();

            while (opModeIsActive()) {
                intake.runIntake(gamepad1, 1);


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
                if (gamepad1.rightBumperWasPressed()) {
                    shooter.setShootingTargetTPS(shooter.SHOOTING_TARGET_TPS + changeTPS);
                }
                if (gamepad1.leftBumperWasPressed()) {
                    shooter.setShootingTargetTPS(Math.max(changeTPS, shooter.SHOOTING_TARGET_TPS - changeTPS));
                }

                shooter.update(gamepad1);

                tunerTelemetry.addData("Shooter state", shooter.getShooterState());
                tunerTelemetry.addData("P (D-pad Up/Down)", "%.7f", shooter.kP);
                tunerTelemetry.addData("I (D-pad Right/Left)", "%.7f", shooter.kI);
                tunerTelemetry.addData("D (Y/X)", "%.7f", shooter.kD);
                tunerTelemetry.addData("Idle speed TPS", "%.0f", shooter.IDLE_TARGET_TPS);
                tunerTelemetry.addData("Shooting speed TPS", "%.0f", shooter.SHOOTING_TARGET_TPS);
                tunerTelemetry.addData("Target TPS", "%.0f", shooter.getTargetTPS());
                tunerTelemetry.addData("Actual TPS", "%.0f", shooter.getVelocity());
                tunerTelemetry.addLine("One PID set for both speeds. Record P/I/D and copy them into ShooterSubsystem.");
                panels.update(telemetry);
                idle();
            }
        } finally {
            shooter.stop();
        }
    }
}
