package org.firstinspires.ftc.teamcode.meet0.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;
import java.util.List;
public class OuttakeSubsystem {
    private final DcMotorEx flywheel;
    private final VoltageSensor batteryVoltage;
    public double kP = 0.0035;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kF_LOW = 0.000490; // NEEDS TUNING (REFER TO OLD PROGRAM LAST  YEAR)
    public double kF_HIGH = 0.000426; // NEEDS TUNING (REFER TO OLD PROGRAM LAST  YEAR)
    public double LOW_TARGET_TPS = 700;
    public double HIGH_TARGET_TPS = 1780;
    public double DANGER_THRESHOLD = 0.93;
    public double RECOVERY_SLEW = 1.0;
    private double normalTPS = 1100;
    private double targetTPS = normalTPS;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastPower = 0;
    private boolean isRecovering = false;
    private double lastRecoveryTime = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final List<Double> recoveryLog = new ArrayList<>();
    public OuttakeSubsystem(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        pidTimer.reset();
    }
    // Call once per OpMode loop to maintain the target velocity.
    public void update() {
        if (targetTPS <= 0) {
            setFlywheelPower(0);
            return;
        }
        double finalPower = calculateCompensatedPower();
        monitorRecovery();
        setFlywheelPower(finalPower);
    }
    private double calculatePIDF(double target, double current) {
        double slope = (kF_HIGH - kF_LOW) / (HIGH_TARGET_TPS - LOW_TARGET_TPS);
        double kF = kF_LOW + (target - LOW_TARGET_TPS) * slope;
        double feedforward = target * kF;
        double error = target - current;
        if (current < target * DANGER_THRESHOLD) {
            return 1.0; // Bang-bang recovery
        }
        double dt = Math.max(pidTimer.seconds(), 0.001);
        if (Math.abs(error) < 150) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }
        double derivative = (error - lastError) / dt;
        lastError = error;
        pidTimer.reset();
        return feedforward + (kP * error) + (kI * integralSum) + (kD * derivative);
    }
    private double applySlew(double targetPower) {
        double delta = targetPower - lastPower;
        if (Math.abs(delta) > RECOVERY_SLEW) {
            targetPower = lastPower + Math.signum(delta) * RECOVERY_SLEW;
        }
        lastPower = targetPower;
        return targetPower;
    }
    private double calculateCompensatedPower() {
        double currentTPS = getVelocity();
        double battery = batteryVoltage.getVoltage();
        double rawPower = calculatePIDF(targetTPS, currentTPS);
        double compensated = rawPower * (12.0 / battery);
        return applySlew(compensated);
    }
    private void monitorRecovery() {
        double ratio = getVelocity() / targetTPS;
        if (!isRecovering && ratio < DANGER_THRESHOLD) {
            isRecovering = true;
            recoveryTimer.reset();
        }
        if (isRecovering && ratio >= 0.98) {
            isRecovering = false;
            lastRecoveryTime = recoveryTimer.milliseconds();
            recoveryLog.add(lastRecoveryTime);
        }
    }
    public void setFlywheelPower(double power) {
        power = Range.clip(power, 0, 1);
        flywheel.setPower(power);
    }
    public void setTargetTPS(double tps) { targetTPS = Math.max(0, tps); }
    public double getTargetTPS() {return targetTPS;}
    public void setNormalTPS(double tps) { this.normalTPS = tps; }
    public double getLastRecoveryTime() {
        return lastRecoveryTime;
    }
    public double getVelocity() {
        return flywheel.getVelocity();
    }
}