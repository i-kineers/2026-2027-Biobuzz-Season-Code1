package org.firstinspires.ftc.teamcode.meet0.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MasterLogic {

    private final IntakeSubsystem intake;
    private final DriveSubsystem drive;

    public MasterLogic(HardwareMap hardwareMap, boolean isBlue) {

        intake = new IntakeSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);
    }

    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        drive.drive(gamepad1);
        intake.runIntake(gamepad1, 1.0);
    }
}
