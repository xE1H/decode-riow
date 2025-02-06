package org.firstinspires.ftc.teamcode.auto.commands.factory;

import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.HORIZONTAL_EXTENSION_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_EXTENSION_IN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.TICKS_PER_IN;
import static org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration.TargetPosition.UP;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.commands.MoveRelative;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.auto.commands.GrabBucketSample;
import org.firstinspires.ftc.teamcode.auto.commands.ScoreHighBucketSample;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArmAuto;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.IntakeSample;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.ScoreSample;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.BestSampleDeterminer;
import org.firstinspires.ftc.teamcode.subsystems.vision.OrientationDeterminerPostProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.vision.commands.ProcessFrame;

import java.util.List;
import java.util.function.Supplier;

@Config
@Photon
public class NetCommandFactory extends CommandFactory {
    public static int toScoreX = 27;
    public static int toScoreY = 119;

    public static double toSample1X = 33.5;
    public static double toSample1Y = 118.7;

    public static double toSample2X = 33.5;
    public static double toSample2Y = 128.7;

    public static double toSample3PrepareX = 35;
    public static double toSample3PrepareY = 120;

    public static int toSample3Heading = 60;

    public static double toSample3X = 40;
    public static double toSample3Y = 125.7;

    private final Point startingPoint;
    private final Point toScore;
    private final Point toSample1;
    private final Point toSample2;
    //    private final Point toSample3;
    private final Point toNetArea;

    public static int toScoreHeading = -50;

    private final Alliance alliance;

    private final ElapsedTime time;

    private SequentialCommandGroup samplePickup = new SequentialCommandGroup();

    final OrientationDeterminerPostProcessor.SampleOrientation[] bestSampleOrientation = {null};

    public NetCommandFactory(Alliance alliance, ElapsedTime time) {
        this.alliance = alliance;
        this.time = time;

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
        return new Class[]{ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class, Vision.class};
    }


    @Override
    public SequentialCommandGroup getCommands() {
        // funny things happening here because the variable is accessed from an inner class
        // so it has to be final - intellisense is suggesting this

        return new SequentialCommandGroup(
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
//                new FollowPath(0, toScoreHeading, new Point(20, 116)),
                new ParallelCommandGroup(
                        new FollowPath(0, toScoreHeading, new Point(toScoreX - 2, toScoreY - 2), 0.999, false),
                        new SequentialCommandGroup(
                                new WaitCommand(450),
                                new ScoreSample(117),
                                new WaitCommand(100)
                        )
                ),
                new ParallelCommandGroup(
                        new RetractArmAuto(),
                        new FollowPath(toScoreHeading, 0, new Point(toScoreX, toSample1Y)).withTimeout(0)
                ),
                new ParallelCommandGroup(
                        new FollowPath(0, toSample1).withTimeout(0),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new GrabBucketSample()
                        )
                ),

//                new FollowPath(0, toScoreHeading, new Point(30, 120)),
                new ParallelCommandGroup(
                        new FollowPath(0, toScoreHeading, toScore, 0.999, false),
                        new SequentialCommandGroup(
                                new WaitCommand(650),
                                new ScoreSample(117),
                                new WaitCommand(100)
                        )
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(900),
                                new FollowPath(toScoreHeading, 0, new Point(toScoreX, toSample2Y))
                        ),
                        new RetractArmAuto()
                ),
                new ParallelCommandGroup(
                        new FollowPath(0, toSample2),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new GrabBucketSample()
                        )
                ),

//                new FollowPath(0, toScoreHeading, new Point(30, 120)),
                new ParallelCommandGroup(
                        new FollowPath(0, toScoreHeading, toScore, 0.999, false),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ScoreSample(117),
                                new WaitCommand(100)
                        )
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(700),
                                new FollowPath(toScoreHeading, toSample3Heading, new Point(toSample3PrepareX, toSample3PrepareY))
                        ),
                        new RetractArmAuto()
                ),
                new InstantCommand() {
                    @Override
                    public void run() {
                        VLRSubsystem.getInstance(ClawSubsystem.class).setHorizontalRotation(0.8);
                    }
                },
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new FollowPath(toSample3Heading, new Point(toSample3X, toSample3Y)),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new IntakeSample(0.2),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(100),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new WaitCommand(120),
                                new SetClawTwist(0.8),
                                new WaitCommand(300),
                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new WaitCommand(200),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetCurrentArmState(ArmState.State.IN_ROBOT)
                        )
                ),
//                  new FollowPath(90, toScoreHeading, new Point(30, 120)),
                new ParallelCommandGroup(
                        new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                        new FollowPath(40, toScoreHeading, toScore, 0.999, false).withTimeout(300),
                        new SequentialCommandGroup(
                                new WaitCommand(650),
                                new ScoreSample(117),
                                new WaitCommand(100)
                        )
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                new FollowPath(toScoreHeading, -90, new Point(72, 140), new Point(64, 95))
                        ).withTimeout(2000),
                        new RetractArm().withTimeout(1500),
                        new WaitCommand(3500)
                ),
                new ParallelCommandGroup(
                        new PrintCommand("eikit nahui"),
                        new ProcessFrame(),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new InstantCommand() {
                                    @Override
                                    public void run() {
                                        //VLRSubsystem.getInstance(ArmSlideSubsystem.class).setTargetPosition(0.35);
                                        VLRSubsystem.getInstance(ClawSubsystem.class).setTargetState(ClawConfiguration.GripperState.OPEN);
                                        VLRSubsystem.getInstance(ClawSubsystem.class).setTargetAngle(ClawConfiguration.VerticalRotation.DEPOSIT);
                                        VLRSubsystem.getInstance(ClawSubsystem.class).setHorizontalRotation(ClawConfiguration.HorizontalRotation.NORMAL);
                                    }
                                }
                        )
                ),
                new InstantCommand() {
                    @Override
                    public void run() {
                        System.out.println("vatafak epel meps");
                        List<OrientationDeterminerPostProcessor.SampleOrientation> samples = VLRSubsystem.getInstance(Vision.class).getSampleOrientations();
                        System.out.println("vatafak gogol meps");
                        bestSampleOrientation[0] = BestSampleDeterminer.determineBestSample(samples, alliance);
                        // log for debug
                        System.out.println("Going for sample: " + bestSampleOrientation[0].color + " in X: " + bestSampleOrientation[0].relativeX + " Y: " + bestSampleOrientation[0].relativeY);
                    }
                },
                new InstantCommand() {
                    @Override
                    public void run() {
                        generateSubmersibleSampleCommand();
                        System.out.println(bestSampleOrientation[0] == null);
                    }
                },
                new ConditionalCommand(
                        getSubmersibleSample(),
                        dontDoShit(),
                        () -> bestSampleOrientation[0] != null && 30 - time.seconds() > 3 // don't go if theres no time, better to park
                )
        );
    }

    private void generateSubmersibleSampleCommand() {
        System.out.println("generating sub grab cmd");
        samplePickup.addCommands(
                // sample relative X is positive to the right; Y is positive to the front
                new SetSlideExtension((TICKS_PER_IN * (bestSampleOrientation[0].relativeY + 2)) / MAX_POSITION),
                new ParallelCommandGroup(
                        new MoveRelative(-bestSampleOrientation[0].relativeX + 0.6, 0),
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition)
                ).withTimeout(600),
                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                new WaitCommand(150),
                new SetClawTwist(bestSampleOrientation[0].isVerticallyOriented ? ClawConfiguration.HorizontalRotation.NORMAL : ClawConfiguration.HorizontalRotation.FLIPPED),
                new WaitCommand(250),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new WaitCommand(150),
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                new CustomConditionalCommand(
                        new ParallelRaceGroup(
                                new FollowPath(-90, toScoreHeading, new Point(72, 140), new Point(toScoreX - 2, toScoreY - 2)),
                                new SequentialCommandGroup(
                                        new SetCurrentArmState(ArmState.State.IN_ROBOT),
                                        new WaitCommand(1600),
                                        new ScoreSample(117),

                                        new WaitCommand(100),
                                        new SetClawState(ClawConfiguration.GripperState.OPEN),
                                        new PrintCommand("fein fein fein"),
                                        new RetractArm(),
                                        new PrintCommand("fein2")
                                )
                        ),
                        () -> 30 - time.seconds() > 3
                )
        );
    }

    private SequentialCommandGroup getSubmersibleSample() {
        System.out.println("getting cmd");
        return samplePickup;
    }

    /**
     * Park if there's no time left
     */
    private SequentialCommandGroup dontDoShit() {
        return new SequentialCommandGroup(
                // todo
                new PrintCommand("gavno gavno gavno")
        );
    }
}
