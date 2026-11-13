package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.31942513079169044);
                Controller secondaryTranslationalForward = Controller.proportional(0.1180190506308399);
                Controller primaryTranslationalLateral = Controller.proportional(0.4619468075617948);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17067700194805263);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012441399785442597));
                c.brake.set(Controller.proportionalFeedforward(0.010575189817626207));

                c.headingFeedback.set(Controller.proportional(4.5440325018254395));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05825411214575116, 0.004269382114285345));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11618420726220845, 0.06364128725831307));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012921663297347746, 0.002170453364747562));

                c.maxAchievableForwardVelocity.set(78.41762847250587);
                c.maxAchievableStrafeVelocity.set(62.9552953860848);
                c.naturalForwardDeceleration.set(25.726206558830675);
                c.naturalStrafeDeceleration.set(58.73267178058792);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.3825279896653546);
        c.yPodOffset.set(-6.865445835383858);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("flmotor");
        c.frontRightName.set("frmotor");
        c.backLeftName.set("blmotor");
        c.backRightName.set("brmotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}