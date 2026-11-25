
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.EnumMap;
import java.util.List;

public class VisionSubsystem {

    public enum Alliance {
        RED,
        BLUE
    }

    public enum HiveSide {
        LEFT,
        RIGHT
    }

    public enum CellState {
        UP,
        DOWN,
        UNKNOWN
    }

    private static final int APRILTAG_PIPELINE = 1;

    private static final double UP_MAX_ROLL = 80.0;
    private static final double DOWN_MIN_ROLL = 100.0;

    private static final double MIN_TAG_AREA = 0.01;


    private final Limelight3A limelight;
    private final Alliance alliance;


    private HiveSide targetCell = HiveSide.RIGHT;

    private final EnumMap<HiveSide, CellState> cellStates =
            new EnumMap<>(HiveSide.class);

    private final EnumMap<HiveSide, Double> cellRolls =
            new EnumMap<>(HiveSide.class);

    private final EnumMap<HiveSide, Integer> cellTagIds =
            new EnumMap<>(HiveSide.class);

    private boolean targetVisible = false;


    public VisionSubsystem(HardwareMap hardwareMap, Alliance alliance) {

        this.alliance = alliance;

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        for (HiveSide side : HiveSide.values()) {
            cellStates.put(side, CellState.UNKNOWN);
            cellRolls.put(side, Double.NaN);
            cellTagIds.put(side, -1);
        }

        limelight.start();
    }

    // ---------------- UPDATE ----------------

    public void update() {

        // Clear previous frame's detections.
        for (HiveSide side : HiveSide.values()) {
            cellStates.put(side, CellState.UNKNOWN);
            cellRolls.put(side, Double.NaN);
            cellTagIds.put(side, -1);
        }

        targetVisible = false;

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        if (fiducials == null) {
            return;
        }

        // Track the strongest detection for each cell.
        EnumMap<HiveSide, Double> bestAreas =
                new EnumMap<>(HiveSide.class);

        for (HiveSide side : HiveSide.values()) {
            bestAreas.put(side, MIN_TAG_AREA);
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            int id = fiducial.getFiducialId();

            HiveSide side = getHiveSideFromId(id);

            // Ignore tags that do not belong to our alliance.
            if (side == null) {
                continue;
            }

            double area = fiducial.getTargetArea();

            if (area <= bestAreas.get(side)) {
                continue;
            }

            Pose3D targetPose =
                    fiducial.getTargetPoseCameraSpace();

            if (targetPose == null) {
                continue;
            }

            double roll = targetPose
                    .getOrientation()
                    .getRoll(AngleUnit.DEGREES);

            roll = normalizeDegrees(roll);

            CellState state = classifyRoll(roll);

            bestAreas.put(side, area);

            cellStates.put(side, state);
            cellRolls.put(side, roll);
            cellTagIds.put(side, id);
        }

        targetVisible =
                cellStates.get(targetCell) != CellState.UNKNOWN;
    }

    // ---------------- TAG ID MAPPING ----------------

    private HiveSide getHiveSideFromId(int id) {

        if (alliance == Alliance.RED) {

            // Red LEFT = scoring cell.
            if (id >= 30 && id <= 33) {
                return HiveSide.LEFT;
            }

            // Red RIGHT = audience cell.
            if (id >= 34 && id <= 37) {
                return HiveSide.RIGHT;
            }

        } else {

            // Blue LEFT = audience cell.
            if (id >= 38 && id <= 41) {
                return HiveSide.LEFT;
            }

            // Blue RIGHT = scoring cell.
            if (id >= 42 && id <= 45) {
                return HiveSide.RIGHT;
            }
        }

        return null;
    }

    private CellState classifyRoll(double roll) {

        double absoluteRoll = Math.abs(roll);

        if (absoluteRoll <= UP_MAX_ROLL) {
            return CellState.UP;
        }

        if (absoluteRoll >= DOWN_MIN_ROLL) {
            return CellState.DOWN;
        }

        // Near the transition, do not guess.
        return CellState.UNKNOWN;
    }

    private double normalizeDegrees(double angle) {

        while (angle > 180) {
            angle -= 360;
        }

        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }

    public void setTargetCell(HiveSide side) {
        targetCell = side;
        targetVisible =
                cellStates.get(side) != CellState.UNKNOWN;
    }

    public HiveSide getTargetCell() {
        return targetCell;
    }


    public CellState getCellState(HiveSide side) {
        return cellStates.get(side);
    }

    public boolean isCellUp(HiveSide side) {
        return getCellState(side) == CellState.UP;
    }

    public boolean isCellDown(HiveSide side) {
        return getCellState(side) == CellState.DOWN;
    }

    public boolean isTargetCellUp() {
        return isCellUp(targetCell);
    }

    public boolean isTargetCellVisible() {
        return targetVisible;
    }

    public double getCellRoll(HiveSide side) {
        return cellRolls.get(side);
    }

    public int getCellTagId(HiveSide side) {
        return cellTagIds.get(side);
    }


    public void telemetry(Telemetry telemetry) {

        telemetry.addLine("----- HIVE VISION -----");

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Target Cell", targetCell);

        telemetry.addData(
                "LEFT State",
                getCellState(HiveSide.LEFT)
        );

        telemetry.addData(
                "LEFT Roll",
                "%.1f",
                getCellRoll(HiveSide.LEFT)
        );

        telemetry.addData(
                "LEFT Tag ID",
                getCellTagId(HiveSide.LEFT)
        );

        telemetry.addData(
                "RIGHT State",
                getCellState(HiveSide.RIGHT)
        );

        telemetry.addData(
                "RIGHT Roll",
                "%.1f",
                getCellRoll(HiveSide.RIGHT)
        );

        telemetry.addData(
                "RIGHT Tag ID",
                getCellTagId(HiveSide.RIGHT)
        );

        telemetry.addData(
                "Target Visible",
                targetVisible
        );

        telemetry.addData(
                "Target UP",
                isTargetCellUp()
        );
    }

    public void stop() {
        limelight.stop();
    }
}
