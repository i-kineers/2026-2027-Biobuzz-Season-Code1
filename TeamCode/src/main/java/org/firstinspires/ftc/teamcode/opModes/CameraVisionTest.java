package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;


public class CameraVisionTest extends OpMode {
    private VisionSubsystem vision;
    @Override
    public void init() {
        vision = new VisionSubsystem(hardwareMap, VisionSubsystem.Alliance.BLUE);
        telemetry.addLine("Vision Test Initialized");
        telemetry.addLine("Waiting for START...");
        telemetry.update();
    }

    @Override
    public void loop() {
        vision.update();

        // Display both Hive cells.
        vision.telemetry(telemetry);

        telemetry.update();

    }
}
