package org.firstinspires.ftc.teamcode.auto.commands.factory;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.auto.commands.GrabBucketSample;
import org.firstinspires.ftc.teamcode.auto.commands.ScoreHighBucketSample;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

@Config
@Photon
public class NetCommandFactory extends CommandFactory {
    public static int toScoreX = 25;
    public static int toScoreY = 121;

    public static double toSample1X = 32.5;
    public static double toSample1Y = 118;

    public static double toSample2X = 32.5;
    public static double toSample2Y = 128;

    public static int toSample3X = 33;
    public static int toSample3Y = 134;

    private final Point startingPoint;
    private final Point toScore;
    private final Point toSample1;
    private final Point toSample2;
    private final Point toSample3;
    private final Point toNetArea;

    public static int toScoreHeading = -50;

    public NetCommandFactory(){
        startingPoint = new Point(10, 111.5);
        toScore = new Point(toScoreX, toScoreY);
        toSample1 = new Point(toSample1X, toSample1Y);
        toSample2 = new Point(toSample2X, toSample2Y);
        toSample3 = new Point(toSample3X, toSample3Y);
        toNetArea = new Point(14, 130);
    }

    @Override
    public Point getStartingPoint() {
        return startingPoint;
    }

    @Override
    public Class<? extends VLRSubsystem<?>>[] getRequiredSubsystems() {
        return new Class[]{ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class};
    }


    @Override
    public SequentialCommandGroup getCommands(){
        return new SequentialCommandGroup(
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                new FollowPath(0, new Point(startingPoint.getX() + 10, startingPoint.getY())),
                new FollowPath(0, toScoreHeading, toScore),
                new ScoreHighBucketSample(),

                new FollowPath(toScoreHeading, 0, new Point(toScoreX, toSample1Y)),
                new FollowPath(0, toSample1),
                new GrabBucketSample(),

                new FollowPath(0, toScoreHeading, toScore),
                new ScoreHighBucketSample(),

                new FollowPath(toScoreHeading, 0, new Point(toScoreX, toSample2Y)),
                new FollowPath(0, toSample2),
                new GrabBucketSample(),

                new FollowPath(0, toScoreHeading, toScore),
                new ScoreHighBucketSample(),

//                new FollowPath(toScoreHeading, 0, toSample3),
//                new GrabBucketSample(),
//
//                new FollowPath(0, toScoreHeading, toScore),
//                new ScoreHighBucketSample(),

                new FollowPath(toScoreHeading, toNetArea)
        );
    }
}
