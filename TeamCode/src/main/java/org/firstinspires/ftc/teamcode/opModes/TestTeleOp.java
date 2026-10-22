package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.MasterLogic;

@TeleOp(name = "Test Teleop")
public class TestTeleOp extends OpMode {

    private boolean isBlue = true;
    private MasterLogic master;

    @Override
    public void init() {
        telemetry.addLine("Main TeleOp Initialized.");
        telemetry.addLine("Wait for side selection in init_loop...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.yWasPressed()) {
            isBlue = !isBlue;
        }

        // --- Telemetry Display ---
        telemetry.addLine("=== PRE-MATCH SELECTION ===");
        telemetry.addData("Selected Alliance", isBlue ? "BLUE" : "RED");
        telemetry.addLine("Press (Y) on Gamepad 1 to Toggle");
        telemetry.addLine("----------------------------------");
        telemetry.addLine("Press START when ready.");
        telemetry.addLine("Point robot toward field forward before START to zero heading.");
        telemetry.update();
    }

    @Override
    public void start() {
        master = new MasterLogic(hardwareMap, isBlue);
    }

    @Override
    public void loop() {
        master.mainLogic(gamepad1, gamepad2, telemetry);
    }

    @Override
    public void stop() {
        if (master != null) {
            master.stop();
        }
    }

}
