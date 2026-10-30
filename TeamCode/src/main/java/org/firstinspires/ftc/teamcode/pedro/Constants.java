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
                Controller primaryTranslationalForward = Controller.proportional(0.3123219120620702);
                Controller secondaryTranslationalForward = Controller.proportional(0.11539460111174532);
                Controller primaryTranslationalLateral = Controller.proportional(0.42128272889267404);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15565271144400608);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012133390023216915));
                c.brake.set(Controller.proportionalFeedforward(0.010313381519734377));

                c.headingFeedback.set(Controller.proportional(4.602528573741608));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05546831629062969, 0.004055169002463617));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10945331143012903, 0.042086722118304984));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011822068443809108, 0.0021684708566502035));

                c.maxAchievableForwardVelocity.set(78.45690598814966);
                c.maxAchievableStrafeVelocity.set(62.625390357225534);
                c.naturalForwardDeceleration.set(27.657448671122708);
                c.naturalStrafeDeceleration.set(57.36878060592682);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.9300256563922553);
        c.yPodOffset.set(-4.197308397668553);
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