package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {

    private final Servo turret;

    private static final double SERVO_RANGE = 270.0;
    private static final double CENTER_POSITION = 0.5;

    private static final double MIN_POSITION = 0.05;
    private static final double MAX_POSITION = 0.95;

    private static final double GEAR_RATIO = 1.0;
    private static final double SHOOTER_OFFSET = Math.PI;

    private double targetAngle = 0.0;
    private double servoPosition = CENTER_POSITION;
    private boolean targetReachable = false;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(Servo.class, "turret");
    }

    public void setTurretAngle(double angleDegrees) {

        targetAngle = angleDegrees;

        double servoAngle = angleDegrees * GEAR_RATIO;

        double requestedPosition = CENTER_POSITION + servoAngle / SERVO_RANGE;

        targetReachable = requestedPosition >= MIN_POSITION && requestedPosition <= MAX_POSITION;

        servoPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, requestedPosition));

        turret.setPosition(servoPosition);
    }

    public void autoAim(Pose robot, Pose goal) {

        double dx = goal.x() - robot.x();
        double dy = goal.y() - robot.y();

        if (Math.hypot(dx, dy) < 1e-6) {
            targetReachable = false;
            return;
        }

        double targetHeading = Math.atan2(dy, dx);

        double turretAngle = Angle.normalizeSigned(targetHeading - robot.heading() - SHOOTER_OFFSET);

        setTurretAngle(Math.toDegrees(turretAngle));
    }

    public void center() {
        setTurretAngle(0.0);
    }

    public boolean isTargetReachable() {
        return targetReachable;
    }

    public void getTelemetry(Telemetry telemetry) {

        telemetry.addLine("===== TURRET =====");

        telemetry.addData("Target Turret Angle",
                "%.2f", targetAngle);

        telemetry.addData("Commanded Servo Position",
                "%.3f", servoPosition);

        telemetry.addData("Target Reachable",
                targetReachable);
    }
}