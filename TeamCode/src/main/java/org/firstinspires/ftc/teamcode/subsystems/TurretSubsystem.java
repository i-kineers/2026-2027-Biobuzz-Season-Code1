package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {

    private final Servo turret;
    private final AimingSubsystem aimingSubsystem;

    // Servo configuration
    private static final double SERVO_RANGE = 270.0;
    private static final double CENTER_POSITION = 0.5;

    private static final double MIN_POSITION = 0.05;
    private static final double MAX_POSITION = 0.95;

    // Servo rotation / turret rotation
    private static final double GEAR_RATIO = 1.0;

    // Change to -1.0 if the servo rotates backward
    private static final double DIRECTION = 1.0;

    // Mechanical turret limits
    private static final double MIN_TURRET_ANGLE = -60.0;
    private static final double MAX_TURRET_ANGLE = 60.0;

    private double targetAngle = 0.0;
    private double commandedAngle = 0.0;
    private double servoPosition = CENTER_POSITION;

    private boolean targetReachable = false;

    public TurretSubsystem(HardwareMap hardwareMap, AimingSubsystem aimingSubsystem) {
        this.aimingSubsystem = aimingSubsystem;

        turret = hardwareMap.get(Servo.class, "turret");
    }

    public void autoAim(Pose robot) {

        // Get combined Limelight and odometry heading
        double targetHeading = aimingSubsystem.getCombinedTargetHeading();

        // Check whether targeting data is valid
        if (!Double.isFinite(targetHeading) || !Double.isFinite(robot.heading())) {
            targetReachable = false;
            return;
        }

        // Convert field heading into turret-relative angle
        double turretAngle = Angle.normalizeSigned(targetHeading - robot.heading());

        setTurretAngle(Math.toDegrees(turretAngle));
    }

    public void setTurretAngle(double angleDegrees) {

        if (!Double.isFinite(angleDegrees)) {
            targetReachable = false;
            return;
        }

        targetAngle = angleDegrees;

        // Check mechanical reachability
        targetReachable = angleDegrees >= MIN_TURRET_ANGLE && angleDegrees <= MAX_TURRET_ANGLE;

        // Apply mechanical rotation limits
        double limitedAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, angleDegrees));

        // Account for gear ratio and direction
        double servoAngle = limitedAngle * GEAR_RATIO * DIRECTION;

        // Convert degrees into servo position
        double requestedPosition = CENTER_POSITION + servoAngle / SERVO_RANGE;

        // Check servo reachability
        targetReachable = targetReachable && requestedPosition >= MIN_POSITION && requestedPosition <= MAX_POSITION;

        // Apply servo position limits
        servoPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, requestedPosition));

        // Calculate the final commanded turret angle
        commandedAngle = (servoPosition - CENTER_POSITION) * SERVO_RANGE / (GEAR_RATIO * DIRECTION);

        // Move the turret
        turret.setPosition(servoPosition);
    }

    public void center() {
        setTurretAngle(0.0);
    }

    public boolean isTargetReachable() {
        return targetReachable;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getCommandedAngle() {
        return commandedAngle;
    }

    public double getServoPosition() {
        return servoPosition;
    }

    public void addTelemetry(Telemetry telemetry) {

        telemetry.addLine("===== TURRET =====");

        telemetry.addData(
                "Target Turret Angle",
                "%.2f deg", targetAngle
        );

        telemetry.addData(
                "Commanded Turret Angle",
                "%.2f deg", commandedAngle
        );

        telemetry.addData(
                "Servo Position",
                "%.3f", servoPosition
        );

        telemetry.addData(
                "Target Reachable",
                targetReachable
        );

        telemetry.addData(
                "Vision Available",
                aimingSubsystem.isVisionAvailable()
        );

        telemetry.addData(
                "Combined Target Heading",
                "%.2f deg",
                Math.toDegrees(
                        aimingSubsystem.getCombinedTargetHeading()
                )
        );
    }
}