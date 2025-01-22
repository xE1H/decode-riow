package org.firstinspires.ftc.teamcode.auto.pedroCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

public class TranslatePosition extends CommandBase {
    double translateX;
    double translateY;

    static Follower follower;

    public TranslatePosition(double translateX, double translateY) {
        this.translateX = translateX;
        this.translateY = translateY;
    }

    public static void setFollower(Follower newFollower) {
        follower = newFollower;
    }

    @Override
    public void initialize() {
        Pose lastPoint = follower.getPose();
        Pose currentPose = follower.getPose();
        double heading = follower.getTotalHeading();

        currentPose.setX(currentPose.getX() + translateX);
        currentPose.setY(currentPose.getY() + translateY);
        Path path = new Path(
                new BezierLine(
                        new Point(lastPoint),
                        new Point(currentPose)
                )
        );
        path.setConstantHeadingInterpolation(heading);

        follower.followPath(path);
    }
}
