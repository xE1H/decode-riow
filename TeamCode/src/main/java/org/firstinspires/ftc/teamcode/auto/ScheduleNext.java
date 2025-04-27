//package org.firstinspires.ftc.teamcode.commands;
//
//import static org.firstinspires.ftc.teamcode.Points.BUCKET_HIGH_SCORE_POSE;
//import static org.firstinspires.ftc.teamcode.Points.SUB_GRAB_POSE;
//import static org.firstinspires.ftc.teamcode.Points.SUB_PRE_BEZIER_POSE;
//import static org.firstinspires.ftc.teamcode.Points.SUB_PRE_PREGRAB_POSE;
//import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;
//
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.pedropathing.commands.FollowPath;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//
//import org.firstinspires.ftc.teamcode.StrategyController;
//import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
//import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
//import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
//
//public class ScheduleNext extends InstantCommand {
//    StrategyController controller;
//    Follower f;
//    Alliance alliance;
//
//    public static int XGrabOffset = 0;
//    public static boolean XGrabOffsetMaxTriggered = false;
//
//    public ScheduleNext(StrategyController controller, Follower f, Alliance alliance) {
//        this.controller = controller;
//        this.f = f;
//        this.alliance = alliance;
//    }
//
//    @Override
//    public void initialize() {
//        CommandScheduler cs = CommandScheduler.getInstance();
//        controller.reportAsCompleted();
//        System.out.println("SCHEDULE NEXT: REPORTED AS COMPLETED");
//        System.out.println("SCHEDULE NEXT: CURRENT STATE: " + controller.getCurrentState());
//
//        switch (controller.getCurrentState()) {
//            case INIT:
//                // shouldn't even be here really.
//                cs.schedule(new ScheduleNext(controller, f, alliance));
//                break;
//            case SPIKE_SCORE:
//                cs.schedule(
//                        new SequentialCommandGroup(
//                                new AutonomousPeriodActions(f),
//                                new ScheduleNext(controller, f, alliance))
//                );
//                // todo run auto preload + 3 spike marks.
//                break;
//            case SUB_GRAB:
//                System.out.println("Submersible grab");
//                cs.schedule(new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                new RetractArm(),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(600),
//                                        new org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand() {
//                                            @Override
//                                            public void run() {
//                                                VLRSubsystem.getInstance(Limelight.class).enable();
//                                            }
//                                        },
//                                        new FollowPath(f, bezierPath(BUCKET_HIGH_SCORE_POSE, SUB_PRE_BEZIER_POSE,
//                                                new Pose(SUB_PRE_PREGRAB_POSE.getX() + XGrabOffset, SUB_PRE_PREGRAB_POSE.getY(), SUB_PRE_PREGRAB_POSE.getHeading()),
//                                                new Pose(SUB_GRAB_POSE.getX() + XGrabOffset, SUB_GRAB_POSE.getY(), SUB_GRAB_POSE.getHeading()))
//                                                .setLinearHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading(), SUB_GRAB_POSE.getHeading())
//                                                .build()).withTimeout(2500)
//                                )
//                        ),
//                        new WaitCommand(600),
//                        new SubmersibleGrab(f, alliance),
//                        new ScheduleNext(controller, f, alliance)
//                ));
//                break;
//            case LOW_SCORE:
//                // todo modify highbasketscore command to score into the low basket.
//                // now it just runs the high_score function.
//
//            case HIGH_SCORE:
//                cs.schedule(new SequentialCommandGroup(
//                        new HighBasketScore(f),
//                        new ScheduleNext(controller, f, alliance)
//                ));
//                break;
//            case HANG:
//                // todo automatic hang
//                break;
//            case AUTO_TELEOP_TRANSITION_WAIT:
//                cs.schedule(new SequentialCommandGroup(
//                        new RetractArm(),
//                        new WaitCommand(100),
//                        new ScheduleNext(controller, f, alliance)
//                ));
//                break;
//            case END:
//                break;
//        }
//    }
//}
