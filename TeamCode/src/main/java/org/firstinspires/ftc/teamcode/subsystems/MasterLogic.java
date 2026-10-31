package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MasterLogic {

    private final IntakeSubsystem intake;
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final boolean isBlue;

    public MasterLogic(HardwareMap hardwareMap, boolean isBlue) {
        this.isBlue = isBlue;

        intake = new IntakeSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
    }

    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        drive.drive(gamepad1);
        intake.runIntake(gamepad1, 1.0);
        shooter.update(gamepad1, gamepad2);
        displayTelemetry(telemetry);
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive", drive.isHolding() ? "Holding position" : "Field-centric");
        telemetry.addData("Alliance (display only)", isBlue ? "BLUE" : "RED");
        telemetry.addData("Robot X (inches)", "%.1f", drive.getPose().x());
        telemetry.addData("Robot Y (inches)", "%.1f", drive.getPose().y());
        telemetry.addData("Heading (degrees)", "%.1f", drive.getHeadingDegrees());
        telemetry.addData("Shooter state", shooter.getShooterState());
        telemetry.addData("Shooter target (TPS)", "%.0f", shooter.getTargetTPS());
        telemetry.addData("Shooter actual (TPS)", "%.0f", shooter.getVelocity());
        telemetry.addData("Shooter power", "%.2f", shooter.getPower());
        telemetry.addLine("GP1 Back/Share: reset driving reference | LT: intake | LB: outtake");
        telemetry.addLine("GP1 RT: shoot | Start/Options: toggle shooter off/on");
    }

    public void stop() {
        shooter.stop();
        drive.stop();
        intake.stop();
    }
}
