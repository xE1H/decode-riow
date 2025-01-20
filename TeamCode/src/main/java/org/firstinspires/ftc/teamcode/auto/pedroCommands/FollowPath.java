package org.firstinspires.ftc.teamcode.auto.pedroCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

@Config
public class FollowPath extends CommandBase {
    private static Follower follower;
    private PathChain pathChain = null;
    private static Point lastPoint;
    public static double translationalErrorConstraint = 5;

    public FollowPath(int constantHeading, Point point) {
        pathChain = follower.pathBuilder().addPath(
                        new BezierLine(
                                lastPoint,
                                point
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(constantHeading))
                .setPathEndTranslationalConstraint(translationalErrorConstraint)
                .build();
        lastPoint = point;
    }

    public FollowPath(int constantHeading, Point... points) {
        pathChain = follower.pathBuilder().addPath(new BezierCurve(
                        prependPoint(lastPoint, points)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(constantHeading))
                .setPathEndTranslationalConstraint(translationalErrorConstraint)

                .build();
        lastPoint = points[points.length - 1];
    }

    public FollowPath(boolean reverseTangentialDirection, Point... points) {
        pathChain = follower.pathBuilder().addPath(new BezierCurve(
                        prependPoint(lastPoint, points)
                ))
                .setTangentHeadingInterpolation()
                .setPathEndTranslationalConstraint(translationalErrorConstraint)
                .setReversed(reverseTangentialDirection)
                .build();
        lastPoint = points[points.length - 1];
    }

    public FollowPath(int startHeading, int endHeading, Point point) {
        pathChain = follower.pathBuilder().addPath(new BezierLine(lastPoint, point))
                .setLinearHeadingInterpolation(Math.toRadians(startHeading), Math.toRadians(endHeading))
                .setPathEndHeadingConstraint(Math.toRadians(1))
                .setPathEndTranslationalConstraint(translationalErrorConstraint)
                .build();
        lastPoint = point;
    }

    public FollowPath(int startHeading, int endHeading) {
        pathChain = follower.pathBuilder().addPath(new BezierLine(lastPoint, lastPoint))
                .setLinearHeadingInterpolation(Math.toRadians(startHeading), Math.toRadians(endHeading))
                .setPathEndHeadingConstraint(Math.toRadians(1))
                .setPathEndTranslationalConstraint(translationalErrorConstraint)
                .build();
    }

    private Point[] prependPoint(Point point, Point... otherPoints) {
        Point[] updatedArray = new Point[otherPoints.length + 1];
        updatedArray[0] = point;
        System.arraycopy(otherPoints, 0, updatedArray, 1, otherPoints.length);
        return updatedArray;
    }

    public static void setStartingPoint(Point point) {
        lastPoint = point;
    }

    public static void setFollower(Follower newFollower) {
        follower = newFollower;
    }

    @Override
    public void initialize() {
        follower.followPath(pathChain);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}