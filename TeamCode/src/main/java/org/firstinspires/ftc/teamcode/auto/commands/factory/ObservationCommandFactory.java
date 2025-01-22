package org.firstinspires.ftc.teamcode.auto.commands.factory;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen.PrepareSpecimenHigh;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen.PrepareSpecimenIntake;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen.ScoreSpecimenHigh;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.outoftheboxrobotics.photoncore.Photon;

@Config
@Photon
public class ObservationCommandFactory extends CommandFactory {
    private boolean isBlueTeam;

    public static Point startingPoint;
    public static Point toPrepareSpecimenIntake1 = new Point(36, 28);
    public static Point toPrepareSpecimenIntake2 = new Point(24, 28);
    public static Point toSpecimenIntake;

//    private Point rotate;
//    private Point toAllSamplesControl1;
//    private Point toAllSamplesControl2;
//    private Point toAllSamples;
//    private Point toSample1Horizontal;
//    private Point sample1ToObservation;
//    private Point toSample1Vertical;
//    private Point toSample2Horizontal;
//    private Point sample2ToObservation;
//    private Point toSample2Vertical;
//    private Point toSample3Horizontal;
//    private Point sample3ToObservation;
//    private Point toScoreControl1;
//    private Point toSpecimenIntake;
//    private Point toSpecimenIntakeControl1;


    public static double toScoreX = 38;
    public static double toScoreY = 59.5;

    public static double toParkX = 18;
    public static double toParkY = 18;

    public ObservationCommandFactory(boolean isBlueTeam) {
        initializePointsForBlueTeam();
        this.isBlueTeam = isBlueTeam;
//        if (!isBlueTeam) {
//            Point[] allPoints = {startingPoint, toSpecimenScore, rotate, toAllSamplesControl1, toAllSamplesControl2, toAllSamples, toSample1Horizontal, sample1ToObservation, toSample1Vertical, toSample2Horizontal, sample2ToObservation, toSample2Vertical, toSample3Horizontal, sample3ToObservation};
//            mirrorPointsToRedTeam(
//                    allPoints
//            );
//        }
    }


    public void initializePointsForBlueTeam() {
        startingPoint = new Point(9.6, 59.5);
        toSpecimenIntake = new Point(22, 28);

//        rotate = new Point(28.05, 53);
//        toAllSamplesControl1 = new Point(29, 42.5);
//        toAllSamplesControl2 = new Point(38.5, 29);
//        toAllSamples = new Point(59.5, 29);
//        toSample1Horizontal = new Point(59.5, 18);
//        sample1ToObservation = new Point(21, 19);
//        toSample1Vertical = new Point(59.5, 19);
//        toSample2Horizontal = new Point(59.5, 8.5);
//        sample2ToObservation = new Point(21, 9.5);
//        toSample2Vertical = new Point(59.5, 9.5);
//        toSample3Horizontal = new Point(59.5, 1.5);
//        sample3ToObservation = new Point(21, 1.5);
//        toScoreControl1 = new Point(20, 47);
//        toSpecimenIntake = new Point(14, 37.5);
//        toSpecimenIntakeControl1 = new Point(37, 42.6);

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
    public SequentialCommandGroup getCommands() {
        return new SequentialCommandGroup(
                // Score preload
                new PrepareSpecimenHigh(toScoreX, toScoreY),
                new ScoreSpecimenHigh(),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                // Intake specimen 1
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new FollowPath(0, 180, toPrepareSpecimenIntake1)
                        ),
                        new SequentialCommandGroup(
                                new RetractArm(),
                                new WaitCommand(800),
                                new PrepareSpecimenIntake()
                        )
                ),

                new FollowPath(180, toPrepareSpecimenIntake2),
                new IntakeSpecimen(toSpecimenIntake),

                // Score specimen 1
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new ParallelCommandGroup(
                        new FollowPath(180, new Point(30, toScoreY)),
                        new SetSlideExtension(0)
                ),
                new PrepareSpecimenHigh(toScoreX, toScoreY - 3),
                new ScoreSpecimenHigh(),

                // Intake specimen 2
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new FollowPath(0, 180, toPrepareSpecimenIntake1)
                        ),
                        new SequentialCommandGroup(
                                new RetractArm(),
                                new WaitCommand(800),
                                new PrepareSpecimenIntake()
                        )
                ),
                new FollowPath(180, toPrepareSpecimenIntake2),
                new IntakeSpecimen(toSpecimenIntake),

                // Score specimen 2
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new ParallelCommandGroup(
                        new FollowPath(180, new Point(30, toScoreY)),
                        new SetSlideExtension(0)
                ),
                new PrepareSpecimenHigh(toScoreX, toScoreY - 6),
                new ScoreSpecimenHigh(),
                new RetractArm(),
                new FollowPath(0, new Point(toParkX, toParkY))
        );
    }

}
