package org.firstinspires.ftc.teamcode.auto.commands.factory;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen.PrepareSpecimenHigh;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen.ScoreSpecimenHigh;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.roboctopi.cuttlefish.queue.PointTask;

public class ObservationCommandFactory extends CommandFactory {
    private boolean isBlueTeam;

    private Point startingPoint;
    private Point toSpecimenScore;
    /**
     * Not sure if this point is necessary
     */
    private Point rotate;
    private Point toAllSamplesControl1;
    private Point toAllSamplesControl2;
    private Point toAllSamples;
    private Point toSample1Horizontal;
    private Point sample1ToObservation;
    private Point toSample1Vertical;
    private Point toSample2Horizontal;
    private Point sample2ToObservation;
    private Point toSample2Vertical;
    private Point toSample3Horizontal;
    private Point sample3ToObservation;
    private Point toScoreControl1;
    private Point toSpecimenIntake;
    private Point toSpecimenIntakeControl1;


    public ObservationCommandFactory(boolean isBlueTeam) {
        initializePointsForBlueTeam();
        this.isBlueTeam = isBlueTeam;
        if (!isBlueTeam) {
            Point[] allPoints = {startingPoint, toSpecimenScore, rotate, toAllSamplesControl1, toAllSamplesControl2, toAllSamples, toSample1Horizontal, sample1ToObservation, toSample1Vertical, toSample2Horizontal, sample2ToObservation, toSample2Vertical, toSample3Horizontal, sample3ToObservation};
            mirrorPointsToRedTeam(
                    allPoints
            );
        }
    }

    @Override
    public void initializePointsForBlueTeam() {
        startingPoint = new Point(9.6, 60);
        toSpecimenScore = new Point(37, 60);
        rotate = new Point(28.05, 60);
        toAllSamplesControl1 = new Point(29, 42.5);
        toAllSamplesControl2 = new Point(38.5, 29);
        toAllSamples = new Point(57.5, 29);
        toSample1Horizontal = new Point(57.5, 19);
        sample1ToObservation = new Point(21, 19);
        toSample1Vertical = new Point(57.5, 19);
        toSample2Horizontal = new Point(57.5, 9.5);
        sample2ToObservation = new Point(21, 9.5);
        toSample2Vertical = new Point(57.5, 9.5);
        toSample3Horizontal = new Point(57.5, 3.5);
        sample3ToObservation = new Point(21, 3.5);
        toScoreControl1 = new Point(20, 47);
        toSpecimenIntake = new Point(14, 37.5);
        toSpecimenIntakeControl1 = new Point(37, 42.6);

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

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new PrintCommand("Specimen: Reaching Score"),
                                new FollowPath(0, toSpecimenScore),
                                new PrintCommand("Specimen: Score Reached")
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                new PrintCommand("Specimen: Preparing"),
                                new PrepareSpecimenHigh(),
                                new PrintCommand("Specimen: Prepared")
                        )
                ),

                new WaitCommand(100),
                new PrintCommand("Specimen: Scoring"),
                new ScoreSpecimenHigh(),
                new PrintCommand("Specimen: Scored"),
                new RetractArm(),
                new PrintCommand("Specimen: Arm Retracted"),
                new FollowPath(0, -90, rotate),
                new WaitCommand(500),
                new FollowPath(!isBlueTeam, toAllSamplesControl1, toAllSamplesControl2, toAllSamples),
                new FollowPath(0, toSample1Horizontal),
                new FollowPath(0, sample1ToObservation),
                new FollowPath(0, toSample1Vertical),
                new FollowPath(0, toSample2Horizontal),
                new FollowPath(0, sample2ToObservation),
                new FollowPath(0, toSample2Vertical),
                new FollowPath(0, toSample3Horizontal),
                new FollowPath(0, sample3ToObservation),
                new IntakeSpecimen(),
                new ParallelCommandGroup(
                        new RetractArm(),
                        new FollowPath(0, toScoreControl1, toSpecimenScore)
                ),
                new ScoreSpecimenHigh(),
                new ParallelCommandGroup(
                        new RetractArm(),
                        new FollowPath(0, toSpecimenIntakeControl1, toSpecimenIntake)
                ),
                new IntakeSpecimen(),
                new ParallelCommandGroup(
                        new RetractArm(),
                        new FollowPath(0, toScoreControl1, toSpecimenScore)
                ),
                new ScoreSpecimenHigh(),
                new ParallelCommandGroup(
                        new RetractArm(),
                        new FollowPath(0, toSpecimenIntakeControl1, toSpecimenIntake)
                ),
                new IntakeSpecimen(),
                new ParallelCommandGroup(
                        new RetractArm(),
                        new FollowPath(0, toScoreControl1, toSpecimenScore)
                ),
                new ScoreSpecimenHigh()
        );

    }

}
