package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MasterLogic {

    private final IntakeSubsystem intake;
    private final DriveSubsystem drive;
    private final FlywheelSystem flywheel;

    public MasterLogic(HardwareMap hardwareMap, boolean isBlue) {

        intake = new IntakeSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);
        flywheel = new FlywheelSystem(hardwareMap);
    }

    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        drive.drive(gamepad1);
        intake.runIntake(gamepad1, 1.0);
        flywheel.update(gamepad1);
        telemetry.addData("Drive", "Field-centric");
        telemetry.addData("Heading (degrees)", "%.1f", drive.getHeadingDegrees());
        telemetry.addLine("Options/Start: reset heading | LT: intake | LB: outtake");
    }

    public void stop() {
        drive.stop();
        intake.stop();
    }
}
