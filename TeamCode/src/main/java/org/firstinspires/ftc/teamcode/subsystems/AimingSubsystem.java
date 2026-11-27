package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Angle;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class AimingSubsystem {

    // Localization heading follows the intake. The shooter faces the rear.
    private static final double SHOOTER_OFFSET = Math.toRadians(180.0);
    private static final double VISION_WEIGHT = 0.7;
    private static final double VISION_SMOOTHING = 0.2;
    private static final double AIM_STOP_THRESHOLD = Math.toRadians(1.5);
    private static final double AIM_RESTART_THRESHOLD = Math.toRadians(2.5);

    // Goal coordinates use the same field frame as the localization pose.
    private final Pose oppGoalPose = new Pose(58.14, 60.56);
    private final Pose audienceGoalPose = new Pose(58.02, 81.77);
    private final VisionSubsystem visionSubsystem;
    private final Controller headingController;
    private final boolean isBlue;

    private boolean autoAimEnabled = false;
    private double cameraTargetHeading = Double.NaN;
    private double visionCorrection = 0.0;
    private double combinedTargetHeading = Double.NaN;
    private Boolean previousAudienceSide = null;
    private boolean aimLocked = false;

    public AimingSubsystem(VisionSubsystem visionSubsystem, Controller headingController, boolean isBlue) {
        this.visionSubsystem = visionSubsystem;
        this.headingController = headingController;
        this.isBlue = isBlue;
    }

    public boolean isEnabled() {
        return autoAimEnabled;
    }

    public void toggle() {
        autoAimEnabled = !autoAimEnabled;
        resetTracking();
    }

    public void stop() {
        autoAimEnabled = false;
        resetTracking();
    }

    public void resetTracking() {
        headingController.reset();
        visionCorrection = 0.0;
        aimLocked = false;
    }

    public void update(Pose robot) {
        boolean currentAudienceSide = isAudienceSide(robot);
        if (previousAudienceSide != null && previousAudienceSide != currentAudienceSide) {
            resetTracking();
        }
        previousAudienceSide = currentAudienceSide;

        visionSubsystem.update();
        cameraTargetHeading = calculateMidpointAngle(robot, currentAudienceSide);
    }

    /** Returns turn power for aiming; the caller must apply it as the drive turn input. */
    public double calculateTurnPower(Pose robot, double angularVelocity) {
        Pose currentGoalPose = getSelectedGoalPose(robot);

        double dx = currentGoalPose.x() - robot.x();
        double dy = currentGoalPose.y() - robot.y();
        if (Math.hypot(dx, dy) < 1e-6) {
            headingController.reset();
            return 0.0;
        }

        combinedTargetHeading = combineTargetHeadings(getOdometryTargetHeading(robot, currentGoalPose));
        double targetHeading = combinedTargetHeading;

        double currentHeading = robot.heading();
        double error = Angle.normalizeSigned(targetHeading - currentHeading);

        double brakingDisplacement = getBrakingDisplacement(angularVelocity);
        double predictedHeading = currentHeading + brakingDisplacement;
        double brakingError = Angle.normalizeSigned(targetHeading - predictedHeading);

        // Stop correcting only when both the current and predicted stopping headings are close.
        // Keep braking when momentum would carry the shooter past the goal.
        if (aimLocked) {
            if (Math.abs(error) > AIM_RESTART_THRESHOLD || Math.abs(brakingError) > AIM_RESTART_THRESHOLD) {
                aimLocked = false;
            } else {
                return 0.0;
            }
        }

        if (Math.abs(error) < AIM_STOP_THRESHOLD && Math.abs(brakingError) < AIM_STOP_THRESHOLD) {

            aimLocked = true;
            headingController.reset();
            return 0.0;
        }

        // Pedro 3 takes the target and the signed error as separate arguments.
        double turnPower = headingController.calculate(targetHeading, brakingError);

        // Ramp down near target for precision
        double maxTurn = Math.abs(error) < Math.toRadians(10) ? 0.2 : 0.4;
        return Math.max(-maxTurn, Math.min(turnPower, maxTurn));
    }

    private double getOdometryTargetHeading(Pose robot, Pose goal) {
        double bearing = Math.atan2(goal.y() - robot.y(), goal.x() - robot.x());
        return Angle.normalize(bearing + SHOOTER_OFFSET);
    }

    private double combineTargetHeadings(double odometryHeading) {
        if (Double.isFinite(cameraTargetHeading)) {

            // Convert Limelight heading to radians
            double visionHeading = Math.toRadians(
                    cameraTargetHeading
            );

            // Calculate difference between the two systems
            double correction = Angle.normalizeSigned(
                    visionHeading - odometryHeading
            );

            // Smooth the vision correction
            visionCorrection = Angle.normalizeSigned(visionCorrection + VISION_SMOOTHING * Angle.normalizeSigned(correction - visionCorrection));

        } else {

            // Gradually return to odometry-only aiming
            visionCorrection *= 0.95;
        }

        // Combine odometry and vision
        return Angle.normalize(odometryHeading + VISION_WEIGHT * visionCorrection);
    }

    private double calculateMidpointAngle(Pose robot, boolean isAudienceSide) {
        double robotHeading = Math.toDegrees(robot.heading());
        robotHeading = (robotHeading % 360 + 360) % 360;

        // Shooter faces opposite the robot's forward direction
        double shooterHeading = (robotHeading + 180) % 360;

        double[] angles = visionSubsystem.getTagAngles(isAudienceSide, isBlue);

        // Make sure both tags are detected
        if (angles == null || angles.length < 2 || !Double.isFinite(angles[0]) || !Double.isFinite(angles[1])) {
            return Double.NaN;
        }

        // Convert Limelight angles into reference angles
        double theta1 = Math.toRadians(shooterHeading - angles[0]);
        double theta2 = Math.toRadians(shooterHeading - angles[1]);

        double sin1 = Math.sin(theta1);
        double sin2 = Math.sin(theta2);

        if (Math.abs(sin1) < 1e-6
                || Math.abs(sin2) < 1e-6
                || sin1 * sin2 < 0) {
            return Double.NaN;
        }

        // Calculate cotangents
        double cot1 = 1.0 / Math.tan(theta1);
        double cot2 = 1.0 / Math.tan(theta2);

        // Calculate geometric midpoint
        double midpoint = Math.atan2(2.0, cot1 + cot2);

        // Correct midpoint for the opposite viewing side
        if (sin1 < 0) {
            midpoint += Math.PI;
        }

        // Return desired robot heading in degrees
        return (Math.toDegrees(midpoint) + 180) % 360;
    }

    private boolean isAudienceSide(Pose robot) {
        return robot.y() < 71;
    }

    private Pose getSelectedGoalPose(Pose robot) {
        return isAudienceSide(robot) ? audienceGoalPose : oppGoalPose;
    }

    private double getBrakingDisplacement(double angularVelocity) {
        // Match Foresight's signed heading braking displacement (omega is radians/sec).
        Vector2D brakeCoefficients = Constants.foresightConfig.headingBrakeCoefficients.get();
        return brakeCoefficients.x() * angularVelocity
                + brakeCoefficients.y() * angularVelocity * Math.abs(angularVelocity);
    }

    public void addTelemetry(Telemetry telemetry, Pose robot, double angularVelocity) {
        Pose goal = getSelectedGoalPose(robot);
        boolean isAudienceSide = isAudienceSide(robot);

        double headingError = Angle.normalizeSigned(
                combinedTargetHeading - robot.heading()
        );

        double brakingDisplacement = getBrakingDisplacement(angularVelocity);
        double predictedHeading = robot.heading() + brakingDisplacement;

        double brakingError = Angle.normalizeSigned(
                combinedTargetHeading - predictedHeading
        );

        telemetry.addLine("===== AUTO AIM =====");

        telemetry.addData("Auto aim enabled", autoAimEnabled);
        telemetry.addData("Aim Locked", aimLocked);
        telemetry.addData("Aim alliance", isBlue ? "BLUE" : "RED");
        telemetry.addData("Field Side", isAudienceSide ? "Audience" : "Non-Audience");

        telemetry.addData("Current shooter heading (deg)",
                Math.toDegrees(Angle.normalize(robot.heading() + SHOOTER_OFFSET)));

        telemetry.addLine("===== TARGETING =====");

        telemetry.addData("Aim goal X, Y", "%.2f, %.2f", goal.x(), goal.y());
        telemetry.addData("Aim target intake heading (deg)", Math.toDegrees(getOdometryTargetHeading(robot, goal)));
        telemetry.addData("Vision Target", "%.2f", cameraTargetHeading);
        telemetry.addData("Combined Target", "%.2f", Math.toDegrees(combinedTargetHeading));

        telemetry.addData("Vision Available",
                Double.isFinite(cameraTargetHeading));

        double correctionDegrees = Math.toDegrees(visionCorrection);
        telemetry.addData("Smoothed Vision Correction", "%.2f", correctionDegrees);

        telemetry.addData("Applied Vision Correction", "%.2f",
                VISION_WEIGHT * correctionDegrees);

        telemetry.addLine("===== ROTATIONAL CONTROL =====");

        telemetry.addData("Heading Error", "%.2f deg",
                Math.toDegrees(headingError));

        telemetry.addData("Braking Error", "%.2f deg",
                Math.toDegrees(brakingError));

        telemetry.addData("Angular Velocity", "%.2f deg/s",
                Math.toDegrees(angularVelocity));

        telemetry.addData("Braking Displacement", "%.2f deg",
                Math.toDegrees(brakingDisplacement));

        telemetry.addData("Predicted Heading", "%.2f deg",
                Math.toDegrees(Angle.normalize(predictedHeading)));

        visionSubsystem.addTelemetry(telemetry, isAudienceSide, isBlue);
    }
}
