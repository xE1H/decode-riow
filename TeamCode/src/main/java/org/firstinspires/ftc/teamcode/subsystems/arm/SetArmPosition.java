package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem.getArm;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OFFSET_REFERENCE_PLANE;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

import java.util.logging.Level;

public class SetArmPosition extends SequentialCommandGroup{
    private MainArmSubsystem arm;

    public SetArmPosition(boolean interpolation){
        arm = getArm();

        //addRequirements(arm);
        arm.setInterpolation(interpolation);
    }

    public SetArmPosition() {this(false);}

    private ConditionalCommand safeSetTargetPoint(Point targetPoint, Command ifCameraIsInTheWay){
        return new ConditionalCommand(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SET ARM POS COMMAND", "SETTING TARGET ARM ANGLE TO " + targetPoint.angleDegrees() + " WITH MAGNITUDE " + targetPoint.magnitude()),
                                new InstantCommand(()-> arm.setTargetPoint(targetPoint)),
                                new WaitCommand(5),
                                new WaitUntilCommand(()-> arm.motionProfilePathsAtParametricEnd()),
                                new WaitCommand(500),

                                new ConditionalCommand(
                                        new LogCommand("SET ARM POS COMMAND", Level.WARNING, "TARGET REACHED SUCCESSFULLY"),
                                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "ARM TIMEOUT TRIGGERED"),
                                        ()-> arm.reachedTargetPosition()
                                )
                        ),

                        ifCameraIsInTheWay,
                        ()-> !arm.isCameraInTheWayToNewTarget(targetPoint)
                ),

                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "TARGET POINT IS INVALID, SKIPPING"),
                ()-> arm.isTargetPointValid(targetPoint.angleDegrees())
        );
    }

    private ConditionalCommand safeSetTargetPoint(Point point){
        return safeSetTargetPoint(point, moveToTargetWhileAvoidingCamera(point));
    }

    public ConditionalCommand magnitudeAndExtension(double magnitude, double angleDegrees){
        return safeSetTargetPoint(new Point(magnitude, angleDegrees));
    }

    public ConditionalCommand XY(double x_cm, double y_cm, OFFSET_REFERENCE_PLANE reference){
        Point targetPoint = arm.calculateTargetPointWithRealWordCoordinates(x_cm, y_cm, reference);
        return safeSetTargetPoint(targetPoint);
    }

    private SequentialCommandGroup moveToTargetWhileAvoidingCamera(Point target){
        return new SequentialCommandGroup(
                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "MAKING A CAMERA AVOIDANCE MANEUVER"),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new WaitCommand(100),

                safeSetTargetPoint(new Point(0, arm.angleDegrees()),
                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG ON RETRACTION")
                ),

                safeSetTargetPoint(new Point(0, target.angleDegrees()),
                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG WHILE ROTATING TO TARGET")
                ),

                safeSetTargetPoint(new Point(target.magnitude(), target.angleDegrees()),
                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG WHILE EXTENDING ARM TO TARGET")
                ),

                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "END OF CAMERA AVOIDANCE MANEUVER")
        );
    }

    public ConditionalCommand extension(double extension){
        return magnitudeAndExtension(extension, arm.getTargetAngleDegrees());
    }

    public ConditionalCommand angleDegrees(double angleDegrees){
        return magnitudeAndExtension(arm.getTargetExtension(), angleDegrees);
    }

    public ConditionalCommand X(double x, OFFSET_REFERENCE_PLANE reference){
        return XY(x, arm.getTargetY(), reference);
    }

    public ConditionalCommand Y(double y, OFFSET_REFERENCE_PLANE reference){
        return XY(arm.getTargetX(), y, reference);
    }

    public SequentialCommandGroup intakeSample(double extension, double twist){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new SetArmPosition().extension(extension).alongWith(
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.extension() > clamp(extension - 0.15, 0.2 ,1)),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                                new SetClawTwist(twist)
                                        )
                                ),
                                new WaitCommand(150),
                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new WaitCommand(100),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                new SetArmPosition().extension(0)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                )

//                new CustomConditionalCommand(
//                        new SequentialCommandGroup(
//
//                        ),
//                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE)
//                ),
        );
    }
}