package org.firstinspires.ftc.teamcode.meet0.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private final DcMotor frontIntake;

    public enum IntakeState {IDLE, INTAKE, OUTTAKE}
    private IntakeState currentIntakeState = IntakeState.IDLE;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        frontIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        frontIntake.setDirection(DcMotor.Direction.FORWARD);
    }

    public void runIntake(Gamepad gamepad, double power) {

        if (gamepad.left_trigger > 0.1) {
            currentIntakeState = IntakeState.INTAKE;

        } else if (gamepad.left_bumper) {
            currentIntakeState = IntakeState.OUTTAKE;

        } else {
            currentIntakeState = IntakeState.IDLE;
        }

        intakeFSM(gamepad, power);
    }

    private void intakeFSM(Gamepad gamepad, double power) {
        switch (currentIntakeState) {

            case INTAKE:
                setFrontIntake(power);
                break;

            case OUTTAKE:
                setFrontIntake(-power);
                break;

            case IDLE:
                setFrontIntake(0);
                break;
        }
    }

    public void setFrontIntake(double power) {
        frontIntake.setPower(power);
    }
}
