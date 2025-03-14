package org.firstinspires.ftc.teamcode.commands.sample;

import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.TICKS_PER_IN;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.limelight.BestSampleDeterminer;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.commands.WaitUntilNextLimelightUpdate;

import java.util.List;

public class SubmersibleGrab extends SequentialCommandGroup {

    private SequentialCommandGroup submersibleGrabCommand = new SequentialCommandGroup();

    double angle = 90;

    public SubmersibleGrab(Follower f, Alliance alliance, RumbleControls rc) {
        addCommands(
                new PrintCommand("Sub grab command"),
                new ParallelCommandGroup(
                        new WaitUntilNextLimelightUpdate(),
                        new SequentialCommandGroup(
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DEPOSIT),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL)
                        )
                ),
                new InstantCommand() {
                    @Override
                    public void run() {
                        List<Limelight.Sample> samples = VLRSubsystem.getInstance(Limelight.class).getDetections();
                        Limelight.Sample sample = BestSampleDeterminer.determineBestSample(samples, alliance, f.getPose().getX());
                        if (sample == null) {
                            System.out.println("Found nothing ts pmo");
                            rc.doubleBlip();
                            //generateRetry(f, alliance);
                        } else {
                            angle = VLRSubsystem.getInstance(Limelight.class).getAngleEstimation(sample);
                            if (angle == -360) {
                                System.out.println("Found nothing ts pmo");
                                rc.doubleBlip();
                                return;
                            }
                            if (angle < 0) angle += 180;

                            System.out.println("Going for sample: " + sample.color + " in X: " + sample.x + " Y: " + sample.y);
                            System.out.println("rich millionaire: " + angle);
                            System.out.println((angle / -90) + 1);

                            generateSubmersibleGrabCommand(f, sample);
                        }
                    }
                },
                submersibleGrabCommand,
                new WaitCommand(200),
                new RetractArm()
        );

    }

//    private void generateRetry(Follower f, Alliance alliance) {
//        if (ScheduleNext.XGrabOffset >= 20) {
//            ScheduleNext.XGrabOffsetMaxTriggered = true;
//        } else if (ScheduleNext.XGrabOffsetMaxTriggered && ScheduleNext.XGrabOffset == 0) {
//            ScheduleNext.XGrabOffsetMaxTriggered = false;
//        }
//        ScheduleNext.XGrabOffset += 5 * (ScheduleNext.XGrabOffsetMaxTriggered ? -1 : 1); // Do not go to the same spot again if left it to go looking before
//        submersibleGrabCommand.addCommands(
//                new MoveRelative(f, 5 * (ScheduleNext.XGrabOffsetMaxTriggered ? -1 : 1), 0),
//                new WaitCommand(1000),
//                new SubmersibleGrab(f, alliance)
//        );
//    }

    private void generateSubmersibleGrabCommand(Follower f, Limelight.Sample sample) {
        // determine sample orientation
        // a sample is vertically oriented if its height is greater than its width
        submersibleGrabCommand.addCommands(
                new MoveRelative(f, -sample.x, 0),
                new SetSlideExtension((TICKS_PER_IN * (sample.y + 1.5)) / MAX_POSITION),
                new SetCurrentArmState(ArmState.State.SAMPLE_INTAKE),
                new ParallelCommandGroup(
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition)
                ).withTimeout(600),
                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                new WaitCommand(150),
                //new SetClawTwist(isVerticallyOriented ? ClawConfiguration.HorizontalRotation.NORMAL : ClawConfiguration.HorizontalRotation.FLIPPED),
                new SetClawTwist((angle / -90) + 1),
                new WaitCommand(250),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new WaitCommand(150)
        );
    }
}
