package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class IntakeSubsystem {

    private static final double GATE_POSITION_STEP = 0.05;

    private final DcMotor frontIntake;

    // Keep tuned presets between loops; reset them when this subsystem is recreated.
    private double firstServoPosition = 0.0;
    private double secondServoPosition = 1.0;

    public enum IntakeState {IDLE, INTAKE, OUTTAKE}
    private IntakeState currentIntakeState = IntakeState.IDLE;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        frontIntake = hardwareMap.get(DcMotor.class, "intake");
        frontIntake.setDirection(DcMotor.Direction.FORWARD);
        frontIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runIntake(Gamepad gamepad, double power) {

        if (gamepad.left_trigger > 0.1) {
            currentIntakeState = IntakeState.INTAKE;

        } else if (gamepad.left_bumper) {
            currentIntakeState = IntakeState.OUTTAKE;

        } else {
            currentIntakeState = IntakeState.IDLE;
        }

        intakeFSM(power);
    }

    private void intakeFSM(double power) {
        switch (currentIntakeState) {

            case INTAKE:
                setFrontIntake(-power);
                break;

            case OUTTAKE:
                setFrontIntake(power);
                break;

            case IDLE:
                setFrontIntake(0);
                break;
        }
    }

    public void setFrontIntake(double power) {
        frontIntake.setPower(power);
    }

    public void stop() {
        currentIntakeState = IntakeState.IDLE;
        setFrontIntake(0);
    }
}
