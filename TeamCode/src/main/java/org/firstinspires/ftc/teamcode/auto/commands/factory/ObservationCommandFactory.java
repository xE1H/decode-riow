package org.firstinspires.ftc.teamcode.auto.commands.factory;

import org.firstinspires.ftc.teamcode.auto.commands.GrabBucketSample;
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
    public static Point startingPoint = new Point(9.6, 59.5);
    public static Point toPrepareSpecimenIntake1 = new Point(38, 28);
    public static Point toPrepareSpecimenIntake2 = new Point(26, 28);
    public static Point toSpecimenIntake = new Point(22, 28);

    public static double toScoreX = 38;
    public static double toScoreY = 59.5;

    public static double toParkX = 18;
    public static double toParkY = 18;

    public static double toSampleX = 29.9;
    public static double toSampleY = 24.1;


    public static double toSamplePickupX = 12;
    public static double toSamplePickupY = 12;

    public static double ROTATOR_PICKUP = 160;
    public static double SLIDE_PICKUP = 0.1;

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
                new FollowPath(0, new Point(toScoreX, toScoreY)),
                new PrepareSpecimenHigh(),
                new ScoreSpecimenHigh(),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),


//                new FollowPath(0, 180, new Point(8, 28.9), new Point(99, 27), new Point(108.2, 10.2), new Point(26.3, 38), new Point(20.4, 23.8)),

                // Intake specimen 1
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(100),
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
                new FollowPath(0, new Point(toScoreX, toScoreY - 3)),
                new PrepareSpecimenHigh(),
                new ScoreSpecimenHigh(),

                // Prepare Sample
                new FollowPath(0, new Point(4, 37.6), new Point(toSampleX, toSampleY)),
                new ParallelCommandGroup(
                        new FollowPath(0, new Point(toSamplePickupX, toSamplePickupY)),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new GrabBucketSample()
                        )
                ),

                new SequentialCommandGroup(
                        new RetractArm(),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetRotatorAngle(ROTATOR_PICKUP),
                                new SetSlideExtension(SLIDE_PICKUP)
                        ),
                        new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                        new SetClawState(ClawConfiguration.GripperState.OPEN)
                ),

                // Intake specimen 2
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(100),
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
                new FollowPath(0, new Point(toScoreX, toScoreY - 6)),
                new PrepareSpecimenHigh(),
                new ScoreSpecimenHigh(),

                new RetractArm(),
                new FollowPath(0, new Point(toParkX, toParkY))
        );
    }

}
