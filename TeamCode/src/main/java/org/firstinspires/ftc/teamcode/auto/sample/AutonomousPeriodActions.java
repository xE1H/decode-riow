//package org.firstinspires.ftc.teamcode.commands.sample;
//
//import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.START_POSE;
//import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.rad;
//import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;
//
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//import com.pedropathing.commands.FollowPath;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.Point;
//
//import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
//import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArmAuto;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.ScoreSample;
//import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
//import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
//import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
//import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
//
//public class AutonomousPeriodActions extends SequentialCommandGroup {
//    public static int toScoreX = 26;
//    public static int toScoreY = 115;
//
//    public static double toSample1X = 32.3;
//    public static double toSample1Y = 118.7;
//
//    public static double toSample2X = 32.3;
//    public static double toSample2Y = 129;
//
//    public static double toSample3PrepareX = 35;
//    public static double toSample3PrepareY = 120;
//
//    public static int toSample3Heading = 60;
//
//    public static double toSample3X = 40;
//    public static double toSample3Y = 125.7;
//
//    private final Point startingPoint;
//    private final Point toScore;
//    private final Point toSample1;
//    private final Point toSample2;
//    //    private final Point toSample3;
//    private final Point toNetArea;
//
//    public static int toScoreHeading = -45;
//
//    public AutonomousPeriodActions(Follower f) {
//        startingPoint = new Point(10, 111.5);
//        toScore = new Point(toScoreX, toScoreY);
//        toSample1 = new Point(toSample1X, toSample1Y);
//        toSample2 = new Point(toSample2X, toSample2Y);
////        toSample3 = new Point(toSample3X, toSample3Y);
//        toNetArea = new Point(14, 130);
//
//        addCommands(
//                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
////                new FollowPath(0, toScoreHeading, new Point(20, 116)),
//                new ParallelCommandGroup(
//                        //new SetTargetLed(NeoPixelConfiguration.Colour.YELLOW),
//                        new FollowPath(f, bezierPath(START_POSE, new Pose(toScoreX, toScoreY, rad(toScoreHeading)))
//                                .setLinearHeadingInterpolation(0, rad(toScoreHeading)).build())),
//                new SequentialCommandGroup(
//                        new WaitCommand(450),
//                        //new SetLiftUpLed(NeoPixelConfiguration.Colour.YELLOW),
//                        new ScoreSample(117),
//                        new WaitCommand(100)
//                ),
//                new ParallelCommandGroup(
//                        new SequentialCommandGroup(
//                                //new SetLiftDownLed(NeoPixelConfiguration.Colour.YELLOW),
//                                //new SetColour(NeoPixelConfiguration.Colour.CYAN),
//                                new RetractArmAuto()
//                                //new SetTargetLed(NeoPixelConfiguration.Colour.YELLOW)
//                        ),
//                        new FollowPath(f, bezierPath(new Pose(toScoreX, toScoreY, rad(toScoreHeading)), new Pose(toSample1X, toSample1Y, 0)).setLinearHeadingInterpolation(rad(toScoreHeading), 0).build()).withTimeout(0)
//                ),
//                new ParallelCommandGroup(
//                        //new FollowPath(f, bezierPath(new Pose(toScoreX, toSample1Y, 0), new Pose(toSample1X, toSample1Y, 0)).build()).withTimeout(0),
//                        new SequentialCommandGroup(
//                                new WaitCommand(1),
//                                //new SetColour(NeoPixelConfiguration.Colour.YELLOW),
//                                new GrabBucketSample()
//                        )
//                ),
//                //
//                //
////                new FollowPath(0, toScoreHeading, new Point(30, 120)),
//                new ParallelCommandGroup(
//                        new FollowPath(f, bezierPath(new Pose(toSample1X, toSample1Y, 0), new Pose(toScoreX, toScoreY, rad(toScoreHeading)))
//                                .setLinearHeadingInterpolation(0, rad(toScoreHeading)).build()),
//                        new SequentialCommandGroup(
//                                new WaitCommand(650),
//                                new SetSlideExtension(0),
//                                //new SetLiftUpLed(NeoPixelConfiguration.Colour.YELLOW),
//                                new ScoreSample(117),
//                                new WaitCommand(100)
//                        )
//                ),
//                new ParallelCommandGroup(
//                        new SequentialCommandGroup(
//                                new WaitCommand(900),
//                                new FollowPath(f, bezierPath(new Pose(toScoreX, toScoreY, rad(toScoreHeading)), new Pose(toScoreX, toSample2Y, 0))
//                                        .setLinearHeadingInterpolation(rad(toScoreHeading), 0).build())
//                        ),
//                        new SequentialCommandGroup(
//                                //new SetLiftDownLed(NeoPixelConfiguration.Colour.YELLOW),
//                                //new SetColour(NeoPixelConfiguration.Colour.CYAN),
//                                new RetractArmAuto()
//                                //new SetTargetLed(NeoPixelConfiguration.Colour.YELLOW)
//                        )
//                ),
//                new ParallelCommandGroup(
//                        //new SetColour(NeoPixelConfiguration.Colour.YELLOW),
//                        new FollowPath(f, bezierPath(new Pose(toScoreX, toSample2Y, 0), new Pose(toSample2X, toSample2Y, 0)).build()),
//                        new SequentialCommandGroup(
//                                new WaitCommand(300),
//                                new GrabBucketSample()
//                        )
//                ),
//
////                new FollowPath(0, toScoreHeading, new Point(30, 120)),
//                new ParallelCommandGroup(
//                        new FollowPath(f, bezierPath(new Pose(toSample2X, toSample2Y, 0), new Pose(toScoreX, toScoreY, rad(toScoreHeading)))
//                                .setLinearHeadingInterpolation(0, rad(toScoreHeading)).build()),
//                        new SequentialCommandGroup(
//                                new WaitCommand(500),
//                                new SetSlideExtension(0),
//                                //new SetLiftUpLed(NeoPixelConfiguration.Colour.YELLOW),
//                                new ScoreSample(117),
//                                new WaitCommand(100)
//                        )
//                ),
//                new ParallelCommandGroup(
//                        new SequentialCommandGroup(
//                                new WaitCommand(700),
//                                new FollowPath(f, bezierPath(new Pose(toScoreX, toScoreY, rad(toScoreHeading)),
//                                        new Pose(toSample3X, toSample3Y, rad(toSample3Heading)))
//                                        .setLinearHeadingInterpolation(rad(toScoreHeading), rad(toSample3Heading)).build())
//                        ),
//                        new SequentialCommandGroup(
//                                //new SetLiftDownLed(NeoPixelConfiguration.Colour.YELLOW),
//                                //new SetColour(NeoPixelConfiguration.Colour.CYAN),
//                                new RetractArmAuto()
//                                //new SetTargetLed(NeoPixelConfiguration.Colour.YELLOW)
//                        )
//                ),
//                new InstantCommand() {
//                    @Override
//                    public void run() {
//                        VLRSubsystem.getInstance(ClawSubsystem.class).setHorizontalRotation(0.8);
//                    }
//                },
//                new WaitCommand(100),
//                new ParallelCommandGroup(
//                        //new SetColour(NeoPixelConfiguration.Colour.YELLOW),
////                        new FollowPath(f, bezierPath(new Pose(toSample3PrepareX, toSample3PrepareY, rad(toSample3Heading)),
////                                new Pose(toSample3X, toSample3Y, rad(toSample3Heading))).build()),
//                        new SequentialCommandGroup(
//                                new WaitCommand(300),
//                                new SetSlideExtension(0.1),
//                                new SetRotatorAngle(5),
////                                new IntakeSample(0.40),
//                                new SetClawState(ClawConfiguration.GripperState.OPEN),
//                                new WaitCommand(100),
//                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
//                                new WaitCommand(120),
////                                new SetClawTwist(0.8),
//                                new SetRotatorAngle(0),
//                                new SetSlideExtension(0.26),
//
//                                new WaitCommand(300),
//                                new WaitUntilCommand(() -> VLRSubsystem.getRotator().reachedTargetPosition()),
//                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
//
//                                //new SetColour(NeoPixelConfiguration.Colour.CYAN),
//
//                                new WaitCommand(200),
//                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
//                                new SetSlideExtension(0),
//                                new WaitCommand(100)
//                        )
//                ),
////                  new FollowPath(90, toScoreHeading, new Point(30, 120)),
//                new ParallelCommandGroup(
//                        new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
//                        new FollowPath(f, bezierPath(new Pose(toSample3X, toSample3Y, rad(40)), new Pose(toScoreX + 2, toScoreY - 2, rad(toScoreHeading)))
//                                .setLinearHeadingInterpolation(rad(40), rad(toScoreHeading)).build()).withTimeout(300),
//                        new SequentialCommandGroup(
//                                new WaitCommand(650),
//                                //new SetLiftUpLed(NeoPixelConfiguration.Colour.YELLOW),
//                                new ScoreSample(117),
//                                new WaitCommand(100)
//                        )
//                )
//        );
//    }
//}