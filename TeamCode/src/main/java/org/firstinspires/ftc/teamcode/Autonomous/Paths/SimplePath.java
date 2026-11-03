package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class SimplePath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(86.1166, 133.226, 90);
    private final Pose point1 = poseFactory.of(108.0793, 133.0276, 0);
    private final Pose point2Start = poseFactory.of(108.0793, 133.0276, 0);
    private final Pose point2 = poseFactory.of(137.5693, 134.949, 10);
    private final Pose point3Start = poseFactory.of(137.5693, 134.949, 10);
    private final Pose point3 = poseFactory.of(101.1725, 116.6089, 60);
    private final Pose point4Start = poseFactory.of(101.1725, 116.6089, 60);
    private final Pose point4 = poseFactory.of(134, 72.7645, 90);
    private final Pose point5 = poseFactory.of(133.5311, 54.745, 88.5095);
    private final Pose penStroke = poseFactory.of(131.2946, 21.5575, 90);
    private final Pose penStrokeThrough1 = poseFactory.of(109.1984, 65.2983, 0);
    private final Pose penStrokeThrough2 = poseFactory.of(106.8946, 40.6554, 0);
    private final Pose penStrokeThrough3 = poseFactory.of(111.9556, 21.0341, 0);

    public Path path1() {
        return Paths.line(start, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point4, point5).reverseTangent();
    }

    public Path penStroke() {
        return Paths.through(point5, penStrokeThrough1, penStrokeThrough2, penStrokeThrough3, penStroke).constant(penStroke);
    }
}
