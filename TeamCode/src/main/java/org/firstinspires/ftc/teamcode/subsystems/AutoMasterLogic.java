package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.ivy.groups.Groups.parallel;

public class AutoMasterLogic {
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private boolean isBlue;
    private final ElapsedTime autoIntakeTimer = new ElapsedTime();
    private final ElapsedTime autoShootTimer = new ElapsedTime();
    public AutoMasterLogic(HardwareMap hardwareMap, boolean isBlue) {
        this.isBlue = isBlue;

        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
    }
    public Command autoOPMODEIntakeShooter(long intakeTime, long shooterTime) {
        return parallel(
                intake.autoOPMODEintake(intakeTime),
                shooter.autoOPMODEShoot(shooterTime)
        );
    }
}
