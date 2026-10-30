package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** Single-motor flywheel control. Call update(gamepad) once per TeleOp loop. */
public class ShooterSubsystem {

    // Starting values from last year's shooter; tune for this motor and gearing.
    public double kP = 0.0035;
    public double kI = 0.0;
    public double kD = 0.0;

    public double kF_LOW = 0.000437;
    public double kF_HIGH = 0.000422;
    public double LOW_TARGET_TPS = 760;
    public double HIGH_TARGET_TPS = 1460;

    // Operating speeds, independent of the feedforward calibration points above.
    // Idle is a separately tunable preset; changing the shooting target leaves it unchanged.
    public double SHOOTING_TARGET_TPS = 1700;
    public double IDLE_TARGET_TPS = SHOOTING_TARGET_TPS * 0.8;

    public double DANGER_THRESHOLD = 0.93;
    // Maximum power change per update(), matching last year's units.
    // 1.0 permits an immediate change across the full power range.
    public double RECOVERY_SLEW = 1.0;

    private final DcMotorEx flywheel;
    private final Servo gate;
    private final VoltageSensor batteryVoltage;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();

    private double targetTPS = 0.0;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastPower = 0.0;
    private boolean hasLastError = false;
    private boolean wasBangBang = false;
    private boolean isRecovering = false;
    private double lastRecoveryTime = 0.0;

    public enum ShooterState {OFF, SHOOTING, INTAKING}
    private ShooterState shooterState = ShooterState.INTAKING;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(0.0);

        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        flywheel.setPower(0.0);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Read encoder velocity while this class controls motor power itself.
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        resetController();
    }

    private void handleShotStates(Gamepad gamepad) {
        // OFF stays latched until Start is pressed again.
        if (gamepad.startWasPressed()) {
            shooterState = shooterState == ShooterState.OFF
                    ? ShooterState.INTAKING : ShooterState.OFF;
        }
        if (shooterState != ShooterState.OFF) {
            shooterState = gamepad.right_trigger > 0.3
                    ? ShooterState.SHOOTING : ShooterState.INTAKING;
        }
    }

    private void handleShotLogic() {
        switch (shooterState)  {
            case OFF:
                stop();
                break;
            case INTAKING:
                gate.setPosition(0.6);
                setTargetTPS(IDLE_TARGET_TPS);
                break;
            case SHOOTING:
                setTargetTPS(SHOOTING_TARGET_TPS);
                gate.setPosition(targetTPS > 0.0 ? 1.0 : 0.0);
                break;
        }
    }

    /** Selects idle/shooting/off, applies the gate state, and updates motor power. */
    public void update(Gamepad gamepad) {
        handleShotStates(gamepad);
        handleShotLogic();
        update();
    }

    /** Runs velocity control for a target supplied through setTargetTPS(). */
    public void update() {
        if (targetTPS <= 0.0) {
            stop();
            return;
        }

        double currentTPS = getVelocity();
        double dt = Math.max(pidTimer.seconds(), 0.001);
        pidTimer.reset();

        double rawPower = calculatePIDF(currentTPS, dt);
        double voltage = batteryVoltage.getVoltage();
        if (Double.isNaN(voltage) || Double.isInfinite(voltage) || voltage <= 0.0) {
            voltage = 12.0;
        }

        // Limit before slewing so lastPower always represents a real motor command.
        double requestedPower = Range.clip(rawPower * (12.0 / voltage), 0.0, 1.0);
        lastPower = applySlew(requestedPower);
        flywheel.setPower(lastPower);
        monitorRecovery(currentTPS);
    }

    private double calculatePIDF(double currentTPS, double dt) {
        double error = targetTPS - currentTPS;
        boolean bangBang = currentTPS < targetTPS * DANGER_THRESHOLD;

        // Avoid a derivative kick on startup or when leaving full-power recovery.
        double derivative = hasLastError && !wasBangBang ? (error - lastError) / dt : 0.0;
        lastError = error;
        hasLastError = true;
        wasBangBang = bangBang;

        if (bangBang) {
            integralSum = 0.0;
            return 1.0;
        }

        if (Math.abs(error) < 150.0) {
            integralSum += error * dt;
        } else {
            integralSum = 0.0;
        }

        double targetRange = HIGH_TARGET_TPS - LOW_TARGET_TPS;
        double slope = targetRange == 0.0 ? 0.0 : (kF_HIGH - kF_LOW) / targetRange;
        double kF = kF_LOW + (targetTPS - LOW_TARGET_TPS) * slope;
        return targetTPS * kF + kP * error + kI * integralSum + kD * derivative;
    }

    private double applySlew(double requestedPower) {
        double maxChange = Range.clip(RECOVERY_SLEW, 0.0, 1.0);
        double change = Range.clip(requestedPower - lastPower, -maxChange, maxChange);
        return Range.clip(lastPower + change, 0.0, 1.0);
    }

    private void monitorRecovery(double currentTPS) {
        double ratio = currentTPS / targetTPS;
        if (!isRecovering && ratio < DANGER_THRESHOLD) {
            isRecovering = true;
            recoveryTimer.reset();
        }
        if (isRecovering && ratio >= 0.98) {
            isRecovering = false;
            lastRecoveryTime = recoveryTimer.milliseconds();
        }
    }

    /**
     * Supplies the shooting preset, including targets from a future dynamic speed system.
     * Call before update(gamepad). Units are encoder ticks per second, not RPM.
     */
    public void setShootingTargetTPS(double tps) {
        if (Double.isNaN(tps) || Double.isInfinite(tps) || tps <= 0.0) {
            throw new IllegalArgumentException("Shooting target TPS must be finite and positive");
        }
        SHOOTING_TARGET_TPS = tps;
    }

    /**
     * Direct target for update() without a gamepad. Zero stops the motor immediately.
     * update(gamepad) selects its own target from the idle/shooting presets.
     */
    public void setTargetTPS(double tps) {
        if (Double.isNaN(tps) || Double.isInfinite(tps)) {
            throw new IllegalArgumentException("Shooter target TPS must be finite");
        }
        double newTarget = Math.max(0.0, tps);
        if (newTarget <= 0.0) {
            stop();
        } else if (newTarget != targetTPS) {
            targetTPS = newTarget;
            resetController();
        }
    }

    /** Stops immediately, bypassing the slew limit. */
    public void stop() {
        shooterState = ShooterState.OFF;
        targetTPS = 0.0;
        lastPower = 0.0;
        gate.setPosition(0.6);
        flywheel.setPower(0.0);
        resetController();
    }

    private void resetController() {
        integralSum = 0.0;
        lastError = 0.0;
        hasLastError = false;
        wasBangBang = false;
        isRecovering = false;
        lastRecoveryTime = 0.0;
        pidTimer.reset();
        recoveryTimer.reset();
    }

    public double getTargetTPS() {
        return targetTPS;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public double getPower() {
        return lastPower;
    }

    public boolean isRecovering() {
        return isRecovering;
    }

    public double getLastRecoveryTime() {
        return lastRecoveryTime;
    }
}
