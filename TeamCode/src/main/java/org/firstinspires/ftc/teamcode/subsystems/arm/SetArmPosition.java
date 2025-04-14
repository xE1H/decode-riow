package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem.getArm;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OFFSET_REFERENCE_PLANE;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

import java.util.function.BooleanSupplier;
import java.util.logging.Level;

public class SetArmPosition extends SequentialCommandGroup{
    private MainArmSubsystem arm;
    private static BooleanSupplier interruptCondition = ()-> false;

    public SetArmPosition(boolean interpolation){
        arm = getArm();

        addRequirements(arm);
        arm.setInterpolation(interpolation);
    }

    public SetArmPosition() {this(false);}

    private ConditionalCommand safeSetTargetPoint(Point targetPoint, Command ifCameraIsInTheWay){
        return new ConditionalCommand(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SET ARM POS COMMAND", "SETTING TARGET ARM ANGLE TO " + targetPoint.angleDegrees() + " WITH MAGNITUDE " + targetPoint.magnitude()),
                                new InstantCommand(()-> arm.setOperationMode(MainArmConfiguration.OPERATION_MODE.NORMAL)),
                                new InstantCommand(()-> arm.setTargetPoint(targetPoint)),
                                new WaitCommand(5),
                                new WaitUntilCommand(()-> arm.motionProfilePathsAtParametricEnd()),
                                new WaitCommand(500),

                                new ConditionalCommand(
                                        new LogCommand("SET ARM POS COMMAND", Level.WARNING, "TARGET REACHED SUCCESSFULLY")
                                            .andThen(new InstantCommand(()-> arm.setOperationMode(MainArmConfiguration.OPERATION_MODE.HOLD_POINT))),
                                            //^more aggressive pids when arm arrives at target position, but only if it correctly reaches target position

                                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "ARM TIMEOUT TRIGGERED, WAITING INDEFINITELY OR FOR MANUAL OVERRIDE")
                                                .andThen(new WaitUntilCommand(()-> arm.reachedTargetPosition()).interruptOn(interruptCondition)),
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
        Point targetPoint = arm.calculateTargetPointFromRealWordCoordinates(x_cm, y_cm, reference);
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

    public SequentialCommandGroup MANUALLY_INTERRUPT_CURRENT_COMMAND(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> interruptCondition = ()-> true),
                new WaitCommand(50),
                new InstantCommand(()-> interruptCondition = ()-> false)
        );
    }


    private InstantCommand setArmState(ArmState.State state){
        return new InstantCommand(()-> ArmState.set(state));
    }


    public SequentialCommandGroup intake(double extension, double angle, double twist){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("INTAKE SAMPLE COMMAND", "INTAKING SAMPLE FROM IN ROBOT STATE"),

                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new SetArmPosition().extension(extension).alongWith(
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.extension() > clamp(extension - 0.15, 0.2 ,1)),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                                new SetClawTwist(twist)
                                        )
                                ),
                                setArmState(ArmState.State.SAMPLE_INTAKE),

                                new WaitCommand(150),
                                retract()
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                retract(),
                                new LogCommand("INTAKE SAMPLE COMMAND", "INTAKING SAMPLE FROM SAMPLE SCORE OR SPECIMEN SCORE STATES"),
                                intake(extension, angle, twist)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_SCORE)
                )
        );
    }


    public SequentialCommandGroup intakeSample(double extension, double twist) {
        return intake(extension, ClawConfiguration.VerticalRotation.DOWN.pos, twist);
    }

    public SequentialCommandGroup intakeSpecimen(double extension) {
        return intake(extension, 0.8, ClawConfiguration.HorizontalRotation.NORMAL.pos);
    }


    public SequentialCommandGroup retract(){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                       new SequentialCommandGroup(
                               new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SAMPLE SCORE STATE"),

                               new SetClawState(ClawConfiguration.GripperState.OPEN),
                               new WaitCommand(150),
                               new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                               new WaitCommand(50),
                               new SetArmPosition().extension(0),
                               new SetClawState(ClawConfiguration.GripperState.CLOSED),
                               new SetArmPosition().angleDegrees(0),
                               setArmState(ArmState.State.IN_ROBOT)
                       ),
                       ()-> ArmState.isCurrentState(ArmState.State.SPECIMEN_SCORE)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SPECIMEN SCORE STATE"),

                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(80),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new WaitCommand(50),

                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(65),
                                        new WaitCommand(100).andThen(new SetArmPosition().extension(0)),
                                        new WaitCommand(250).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP))
                                ),

                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new SetArmPosition().angleDegrees(0),
                                setArmState(ArmState.State.IN_ROBOT)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.HANG_SECOND_STAGE, ArmState.State.HANG_THIRD_STAGE)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SPECIMEN OR SAMPLE INTAKE STATE"),

                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new WaitCommand(100),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                new SetArmPosition().extension(0),

                                setArmState(ArmState.State.IN_ROBOT)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_INTAKE, ArmState.State.SPECIMEN_INTAKE)
                )
        );
    }


    public SequentialCommandGroup scoreSample(){
        return new SequentialCommandGroup(

        );
    }


    public SequentialCommandGroup scoreSpecimen(){
        return new SequentialCommandGroup(

        );
    }


    public SequentialCommandGroup hangSecondStage(){
        return new SequentialCommandGroup(

        );
    }


    public SequentialCommandGroup hangThirdStage(){
        return new SequentialCommandGroup(

        );
    }
}