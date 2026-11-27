package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.List;

public class VisionSubsystem {

    private final Limelight3A limelight;
    private LLResult result;
    List<LLResultTypes.FiducialResult> tags;

    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

    }

    public void update() {
        result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            tags = null;
            return;
        }

        tags = result.getFiducialResults();
    }

    // Audience side is the left side if you are on the blue alliance
    public double[] getTagAngles(boolean isAudienceSide, boolean isBlue) {
        double[] angles = new double[] { Double.NaN, Double.NaN };
        if (tags == null) {
            return angles;
        }

        int firstId;
        int secondId;

        if (isBlue) {
            firstId = isAudienceSide ? 39 : 43;
            secondId = isAudienceSide ? 40 : 44;
        } else {
            firstId = isAudienceSide ? 35 : 31;
            secondId = isAudienceSide ? 36 : 32;
        }

        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            if (id == firstId) {
                angles[0] = tag.getTargetXDegrees();
            } else if (id == secondId) {
                angles[1] = tag.getTargetXDegrees();
            }
        }

        return angles;
    }

    public void getLimelightDebuggingTelemetry(Telemetry telemetry) {
        telemetry.addData("Connected", limelight.isConnected());
        telemetry.addData("Polling", limelight.isRunning());

        if (result == null) {
            telemetry.addLine("Result is NULL");
            return;
        }

        telemetry.addData("Pipeline Index", result.getPipelineIndex());
        telemetry.addData("Valid Result", result.isValid());
        telemetry.addData("Staleness (ms)", result.getStaleness());
    }

}