package org.firstinspires.ftc.teamcode.meet0.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class MasterLogic {

    private final intakeSubsystem intake;

    public MasterLogic(HardwareMap hardwareMap) {

        intake = new intakeSubsystem(hardwareMap);
    }
}
