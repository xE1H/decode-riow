package org.firstinspires.ftc.teamcode.auto.pedroCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

public class FollowPath extends CommandBase {
    private static Follower follower;
    private PathChain pathChain = null;
    private static Point lastPoint;

    public FollowPath(double constantHeading, Point point){
        pathChain = follower.pathBuilder().addPath(
                new BezierLine(
                        lastPoint,
                        point
                )
        ).setConstantHeadingInterpolation(Math.toRadians(constantHeading)).build();
        lastPoint = point;
    }

    public FollowPath(double constantHeading, Point... points) {
        pathChain = follower.pathBuilder().addPath(new BezierCurve(
                prependPoint(lastPoint, points)
        )).setConstantHeadingInterpolation(Math.toRadians(constantHeading)).build();
        lastPoint = points[points.length - 1];
    }

    public FollowPath(boolean reverseTangentialDirection, Point point){
        pathChain = follower.pathBuilder().addPath(new BezierLine(lastPoint, point))
                .setTangentHeadingInterpolation().setReversed(reverseTangentialDirection).build();
        lastPoint = point;
    }

    public FollowPath(boolean reverseTangentialDirection, Point... points){
        pathChain = follower.pathBuilder().addPath(new BezierCurve(
                prependPoint(lastPoint, points)
        )).setTangentHeadingInterpolation().setReversed(reverseTangentialDirection).build();
        lastPoint = points[points.length - 1];
    }

    public FollowPath(double startHeading, double endHeading, Point point){
        pathChain = follower.pathBuilder().addPath(new BezierLine(lastPoint, point))
                .setLinearHeadingInterpolation(Math.toRadians(startHeading), Math.toRadians(endHeading)).build();
        lastPoint = point;
    }

    public FollowPath(double startHeading, double endHeading){
        pathChain = follower.pathBuilder().addPath(new BezierLine(lastPoint, lastPoint))
                .setLinearHeadingInterpolation(Math.toRadians(startHeading), Math.toRadians(endHeading)).build();
    }

    private Point[] prependPoint(Point point, Point... otherPoints){
        Point[] updatedArray = new Point[otherPoints.length + 1];
        updatedArray[0] = point;
        System.arraycopy(otherPoints, 0, updatedArray, 1, otherPoints.length);
        return updatedArray;
    }

    public static void setStartingPoint(Point point){
        lastPoint = point;
    }

    public static void setFollower(Follower newFollower){
        follower = newFollower;
    }

    @Override
    public void initialize() {
        follower.followPath(pathChain);
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public void end(boolean isInterrupted){
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}