package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MasterLogic {

    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private final IntakeSubsystem intake;
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;

    public MasterLogic(HardwareMap hardwareMap, boolean isBlue, double startPoseX, double startPoseY, double heading) {
        intake = new IntakeSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, isBlue, startPoseX, startPoseY, heading);
        shooter = new ShooterSubsystem(hardwareMap);
    }

    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        drive.drive(gamepad1);
        intake.runIntake(gamepad1, 1);
        shooter.update(gamepad1, gamepad2);
        displayTelemetry(telemetry);
    }

    public void displayTelemetry(Telemetry driverStationTelemetry) {
        Telemetry telemetry = panels.getWrapper();
        drive.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        panels.update(driverStationTelemetry);
    }

    public void stop() {
        shooter.stop();
        drive.stop();
        intake.stop();
    }
}
