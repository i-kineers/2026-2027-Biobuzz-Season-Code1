package org.firstinspires.ftc.teamcode.meet0.path;


import com.pedropathing.math.Pose;


import com.pedropathing.api.PoseFactory;
public class SimplePath {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(8, 80, 90);
    private final Pose point1 = poseFactory.of(8, 95, 90);

}
