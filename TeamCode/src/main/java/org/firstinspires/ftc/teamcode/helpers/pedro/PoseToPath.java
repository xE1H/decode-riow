package org.firstinspires.ftc.teamcode.helpers.pedro;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;

public class PoseToPath {
    public static PathBuilder bezierPath(Pose... poses) {
        if (poses.length == 2) {
            return new PathBuilder().addPath(new BezierLine(poses[0], poses[1]));
        }

        BezierCurve curve = new BezierCurve(poses);
        return new PathBuilder().addPath(curve);
    }
}
