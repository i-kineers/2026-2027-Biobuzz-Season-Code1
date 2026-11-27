package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "AprilTag Limelight Test")
public class AprilTagLimelightTest extends OpMode {

    private Limelight3A limelight;
    private boolean pipelineSwitched;

    @Override
    public void init() {
        limelight = hardwareMap.get(
                Limelight3A.class, "limelight"
        );

        telemetry.setMsTransmissionInterval(50);

        limelight.setPollRateHz(100);
        pipelineSwitched = limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void init_loop() {
        loop();
    }

    @Override
    public void loop() {

        telemetry.addData("Connected",
                limelight.isConnected());

        telemetry.addData("Polling",
                limelight.isRunning());

        telemetry.addData("Pipeline Switch",
                pipelineSwitched);

        telemetry.addData("Time Since Update",
                limelight.getTimeSinceLastUpdate());

        LLResult result = limelight.getLatestResult();

        if (result == null) {
            telemetry.addLine("Result is NULL");
            telemetry.update();
            return;
        }

        telemetry.addData("Pipeline Index",
                result.getPipelineIndex());

        telemetry.addData("Pipeline Type",
                result.getPipelineType());

        telemetry.addData("Valid Result",
                result.isValid());

        telemetry.addData("Staleness (ms)",
                result.getStaleness());

        telemetry.addData("Timestamp",
                result.getTimestamp());

        if (!result.isValid()) {
            telemetry.addLine("No valid target detected");
            telemetry.update();
            return;
        }

        List<FiducialResult> tags =
                result.getFiducialResults();

        telemetry.addData("AprilTag Count", tags.size());

        for (FiducialResult tag : tags) {

            int id = tag.getFiducialId();

            telemetry.addData("Tag " + id + " TX",
                    tag.getTargetXDegrees());

            telemetry.addData("Tag " + id + " TY",
                    tag.getTargetYDegrees());
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}