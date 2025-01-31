package org.firstinspires.ftc.teamcode.auto.commands.factory;

import static org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration.TargetPosition.UP;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.auto.commands.GrabBucketSample;
import org.firstinspires.ftc.teamcode.auto.commands.ScoreHighBucketSample;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.ScoreSample;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;

@Config
@Photon
public class NetCommandFactory extends CommandFactory {
    public static int toScoreX = 27;
    public static int toScoreY = 117;

    public static double toSample1X = 32.5;
    public static double toSample1Y = 118;

    public static double toSample2X = 32.5;
    public static double toSample2Y = 128;

    public static double toSample3PrepareX = 49;
    public static double toSample3PrepareY = 110;

    public static double toSample3X = 49;
    public static double toSample3Y = 123;

    private final Point startingPoint;
    private final Point toScore;
    private final Point toSample1;
    private final Point toSample2;
    //    private final Point toSample3;
    private final Point toNetArea;

    public static int toScoreHeading = -50;

    public NetCommandFactory() {
        startingPoint = new Point(10, 111.5);
        toScore = new Point(toScoreX, toScoreY);
        toSample1 = new Point(toSample1X, toSample1Y);
        toSample2 = new Point(toSample2X, toSample2Y);
//        toSample3 = new Point(toSample3X, toSample3Y);
        toNetArea = new Point(14, 130);
    }

    @Override
    public Point getStartingPoint() {
        return startingPoint;
    }

    @Override
    public Class<? extends VLRSubsystem<?>>[] getRequiredSubsystems() {
        return new Class[]{ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class, HangSubsystem.class};
    }


    @Override
    public SequentialCommandGroup getCommands() {
        return new SequentialCommandGroup(
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
//                new FollowPath(0, toScoreHeading, new Point(20, 116)),
                new FollowPath(0, toScoreHeading, toScore),
                new ScoreHighBucketSample(),

                new FollowPath(toScoreHeading, 0, new Point(toScoreX, toSample1Y)),
                new ParallelCommandGroup(
                        new FollowPath(0, toSample1),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new GrabBucketSample()
                        )
                ),

//                new FollowPath(0, toScoreHeading, new Point(30, 120)),
                new ParallelCommandGroup(
                        new FollowPath(0, toScoreHeading, toScore),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ScoreSample(117),
                                new WaitCommand(100)
                        )
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new FollowPath(toScoreHeading, 0, new Point(toScoreX, toSample2Y))
                        ),
                        new RetractArm()
                ),
                new ParallelCommandGroup(
                        new FollowPath(0, toSample2),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new GrabBucketSample()
                        )
                ),

//                new FollowPath(0, toScoreHeading, new Point(30, 120)),
                new FollowPath(0, toScoreHeading, toScore),
                new ScoreHighBucketSample(),


                new FollowPath(toScoreHeading, 90, new Point(43.8, 93), new Point(toSample3PrepareX, toSample3PrepareY)),
                new ParallelCommandGroup(
                        new FollowPath(90, new Point(toSample3X, toSample3Y)),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new GrabBucketSample(true)
                        )
                ),


//                new FollowPath(90, toScoreHeading, new Point(30, 120)),
                new FollowPath(90, toScoreHeading, new Point(43.8, 93), new Point(30, 120)),
                new FollowPath(toScoreHeading, toScore),
                new ScoreHighBucketSample(),
//                new FollowPath(toScoreHeading, 0, toSample3),
//                new GrabBucketSample(),
//
//                new FollowPath(0, toScoreHeading, toScore),
//                new ScoreHighBucketSample(),
                new InstantCommand() {
                    @Override
                    public void run() {
                        VLRSubsystem.getInstance(HangSubsystem.class).setTargetPosition(UP);
                    }
                },
                new FollowPath(toScoreHeading, 90, new Point(72, 140), new Point(64, 89))
                //new FollowPath(toScoreHeading, toNetArea)
        );
    }
}
