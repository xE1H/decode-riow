package org.firstinspires.ftc.teamcode.auto.pedroCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static double translationalErrorConstraint = 0.1;
    public static double headingErrorConstraint = Math.PI / (360 * 2);
    private int PATH_TIMEOUT = 500;
    private ElapsedTime currentTime = new ElapsedTime();
    private boolean TIMER_HAS_BEEN_RESET = false;

    private int pathLength = 1;


    public FollowPath(int constantHeading, Point point) {
        pathChain = follower.pathBuilder().addPath(
                        new BezierLine(
                                lastPoint,
                                point
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(constantHeading))
                .setPathEndTranslationalConstraint(translationalErrorConstraint)
                .setPathEndHeadingConstraint(headingErrorConstraint)
                .build();
        lastPoint = point;
    }

    public FollowPath(int constantHeading, Point... points) {
        pathChain = follower.pathBuilder().addPath(new BezierCurve(
                        prependPoint(lastPoint, points)
                ))
                .setConstantHeadingInterpolation(constantHeading)
                .setPathEndHeadingConstraint(Math.toRadians(1))
                .setPathEndTranslationalConstraint(1)
                .build();
        lastPoint = points[points.length - 1];
    }

    public FollowPath(int startHeading, int endHeading, Point... points) {
        pathLength = points.length;
        pathChain = follower.pathBuilder().addPath(new BezierCurve(
                        prependPoint(lastPoint, points)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(startHeading), Math.toRadians(endHeading))
                .setPathEndHeadingConstraint(Math.toRadians(1))
                .setPathEndTranslationalConstraint(1)

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

    public FollowPath(int startHeading, int endHeading, Point point, double tValue, boolean dontmatter) {
        pathChain = follower.pathBuilder().addPath(new BezierLine(lastPoint, point))
                .setLinearHeadingInterpolation(Math.toRadians(startHeading), Math.toRadians(endHeading))
                .setPathEndHeadingConstraint(Math.toRadians(1))
                .setPathEndTranslationalConstraint(translationalErrorConstraint)
                .setPathEndTValueConstraint(tValue)
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

    public FollowPath withTimeout(int timeoutMs) {
        PATH_TIMEOUT = timeoutMs;
        return this;
    }

    @Override
    public void execute(){
        if(!TIMER_HAS_BEEN_RESET){
            TIMER_HAS_BEEN_RESET = true;
            currentTime.reset();
        }
    }

    @Override
    public void initialize() {
        follower.followPath(pathChain, true);
    }

    @Override
    public boolean isFinished() {
        // This is what the *official* library now uses to determine if the path is finished
        // Not even the isbusy thing anymore, so this might work better.
        // todo Constants are defined inline for now, fix
        boolean timeoutCondition = TIMER_HAS_BEEN_RESET && currentTime.milliseconds() > PATH_TIMEOUT;
        boolean pathFinishedCondition = Math.abs(follower.headingError) < Math.PI / 360 && follower.getCurrentTValue() >= 0.995;
        System.out.printf("TIMOUT FINISHED: " + currentTime.milliseconds());
        if (follower.getCurrentPathNumber() == pathLength - 1 && ( timeoutCondition || pathFinishedCondition)) {
            return true;
        }
        return false;
    }
}