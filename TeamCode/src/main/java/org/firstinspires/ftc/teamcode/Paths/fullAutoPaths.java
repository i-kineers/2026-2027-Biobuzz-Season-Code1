package org.firstinspires.ftc.teamcode.Paths;


import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;


import com.pedropathing.api.PoseFactory;
import com.pedropathing.paths.Path;

public class fullAutoPaths {

    // THis is assuming that we have flower intaker and turret
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.5383, 5.9329, 180);
    private final Pose point1 = poseFactory.of(7.1294, 5.2748, 180);
    private final Pose point1Control1 = poseFactory.of(36.9245, 9.3152, 0);
    private final Pose point2Start = poseFactory.of(7.1294, 5.2748, 180);
    private final Pose point2 = poseFactory.of(34.2993, 18.5539, 180);
    private final Pose point2Control1 = poseFactory.of(40.0923, 10.1298, 0);
    private final Pose point3Start = poseFactory.of(34.2993, 18.5539, 180);
    private final Pose point3 = poseFactory.of(12.7255, 46.6014, 180);
    private final Pose point4Start = poseFactory.of(12.7255, 46.6014, 180);
    private final Pose point4 = poseFactory.of(47.1629, 112.8562, 180);
    private final Pose point4Control1 = poseFactory.of(14.943, 108.9901, 0);
    private final Pose point5Start = poseFactory.of(47.1629, 112.8562, 180);
    private final Pose point5 = poseFactory.of(47.2561, 129.0602, 90);
    private final Pose point5Control1 = poseFactory.of(49.6175, 122.8991, 0);
    private final Pose point6Start = poseFactory.of(47.2561, 129.0602, 90);
    private final Pose point6 = poseFactory.of(37.6726, 114.447, 90);
    private final Pose point7Start = poseFactory.of(37.6726, 114.447, 90);
    private final Pose point7 = poseFactory.of(10.6172, 91.8441, 90);

    public Path path1() {
        return Paths.curve(start, point1Control1, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }
    // THis is assuming we only have flower intaker
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    private final Pose start = poseFactory.of(56.5383, 5.9329, 270);
//    private final Pose point1 = poseFactory.of(7.1294, 5.2748, 180);
//    private final Pose point1Control1 = poseFactory.of(36.9245, 9.3152, 0);
//    private final Pose point2Start = poseFactory.of(7.1294, 5.2748, 180);
//    private final Pose point2 = poseFactory.of(34.2993, 18.5539, 225);
//    private final Pose point2Control1 = poseFactory.of(40.0923, 10.1298, 0);
//    private final Pose point3Start = poseFactory.of(34.2993, 18.5539, 225);
//    private final Pose point3 = poseFactory.of(12.7255, 46.6014, 180);
//    private final Pose point4Start = poseFactory.of(12.7255, 46.6014, 180);
//    private final Pose point4 = poseFactory.of(47.1629, 112.8562, 120);
//    private final Pose point4Control1 = poseFactory.of(14.943, 108.9901, 0);
//    private final Pose point5Start = poseFactory.of(47.1629, 112.8562, 120);
//    private final Pose point5 = poseFactory.of(47.2561, 129.0602, 90);
//    private final Pose point5Control1 = poseFactory.of(49.6175, 122.8991, 0);
//    private final Pose point6Start = poseFactory.of(47.2561, 129.0602, 90);
//    private final Pose point6 = poseFactory.of(37.6726, 114.447, 135);
//    private final Pose point7Start = poseFactory.of(37.6726, 114.447, 135);
//    private final Pose point7 = poseFactory.of(10.6172, 91.8441, 90);
//
//    public Path path1() {
//        return Paths.curve(start, point1Control1, point1).linear(start, point1);
//    }
//
//    public Path path2() {
//        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
//    }
//
//    public Path path3() {
//        return Paths.line(point3Start, point3).linear(point3Start, point3);
//    }
//
//    public Path path4() {
//        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
//    }
//
//    public Path path5() {
//        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
//    }
//
//    public Path path6() {
//        return Paths.line(point6Start, point6).linear(point6Start, point6);
//    }
//
//    public Path path7() {
//        return Paths.line(point7Start, point7).linear(point7Start, point7);
//    }
//}
}