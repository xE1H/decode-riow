//package org.firstinspires.ftc.teamcode.subsystems.arm;
//
//import static com.arcrobotics.ftclib.util.MathUtils.clamp;
//import static org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem.getArm;
//
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.ConditionalCommand;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
//import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
//import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
//import org.firstinspires.ftc.teamcode.helpers.utils.Point;
//import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OFFSET_REFERENCE_PLANE;
//import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OPERATION_MODE;
//import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.SAMPLE_SCORE_HEIGHT;
//import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
//import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
//import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
//import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
//import org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration;
//import org.firstinspires.ftc.teamcode.subsystems.hang.commands.SetHangPosition;
//
//import java.util.function.BooleanSupplier;
//import java.util.logging.Level;
//
//public class SetArmPositionBACKUP extends SequentialCommandGroup{
//    private MainArmSubsystem arm;
//    private static BooleanSupplier interruptCondition = ()-> false;
//
//    {
//        // This needs to be here, since addRequirements needs to be called BEFORE the command is
//        // able to run. The running may happen instantly (on super() being called), or at any point
//        // in the future, so it's best to call addRequirements as soon as possible, in this case
//        // before the constructor ever runs.
//        addRequirements(getArm());
//    }
//
//    public SetArmPositionBACKUP(boolean interpolation){
//        arm = getArm();
//        arm.setInterpolation(interpolation);
//    }
//
//    public SetArmPositionBACKUP() {this(false);}
//
//    private ConditionalCommand safeSetTargetPoint(Point targetPoint, Command ifCameraIsInTheWay){
//        return new ConditionalCommand(
//                new ConditionalCommand(
//                        new SequentialCommandGroup(
//                                new LogCommand("SET ARM POS COMMAND", "SETTING TARGET ARM ANGLE TO " + targetPoint.angleDegrees() + " WITH MAGNITUDE " + targetPoint.magnitude()),
//                                new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.NORMAL)),
//                                new InstantCommand(()-> arm.setTargetPoint(targetPoint)),
//                                new WaitCommand(5),
//                                new WaitUntilCommand(()-> arm.motionProfilePathsAtParametricEnd()),
//                                new WaitCommand(800),
//
//                                new ConditionalCommand(
//                                        new LogCommand("SET ARM POS COMMAND", Level.WARNING, "TARGET REACHED SUCCESSFULLY")
//                                            .andThen(new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.HOLD_POINT))),
//                                            //^more aggressive pids when arm arrives at target position, but only if it correctly reaches target position
//
//                                        new SequentialCommandGroup(
//                                                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "ARM TIMEOUT TRIGGERED, WAITING INDEFINITELY FOR MANUAL OVERRIDE"),
//                                                new WaitUntilCommand(()-> arm.reachedTargetPosition()).interruptOn(interruptCondition),
//                                                new ConditionalCommand(
//                                                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "WAIT INTERRUPTED, HOPE THE DRIVER KNOWS WHAT HES DOING"),
//                                                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "ARM MAGICALLY REACHED TARGET POSITION WITH NO OVERRIDE"),
//                                                        interruptCondition
//                                                )
//                                        ),
//                                        ()-> arm.reachedTargetPosition()
//                                )
//                        ),
//
//                        ifCameraIsInTheWay,
//                        ()-> !arm.isCameraInTheWayToNewTarget(targetPoint)
//                ),
//
//                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "TARGET POINT IS INVALID, SKIPPING"),
//                ()-> arm.isTargetPointValid(targetPoint.angleDegrees())
//        );
//    }
//
//    private ConditionalCommand safeSetTargetPoint(Point point){
//        return safeSetTargetPoint(point, moveToTargetWhileAvoidingCamera(point));
//    }
//
//    public ConditionalCommand extensionAndAngleDegrees(double magnitude, double angleDegrees){
//        return safeSetTargetPoint(new Point(magnitude, angleDegrees));
//    }
//
//    public ConditionalCommand XY(double x_cm, double y_cm, OFFSET_REFERENCE_PLANE reference){
//        Point targetPoint = arm.calculateTargetPointFromRealWordCoordinates(x_cm, y_cm, reference);
//        return new ConditionalCommand(
//                new LogCommand("SET ARM POSITION.XY", Level.SEVERE, "ARM ALREADY MOVING, WAITING PREVIOUS COMMAND TO FINISH")
//                        .andThen(new WaitUntilCommand(()-> arm.motionProfilePathsAtParametricEnd()).interruptOn(interruptCondition)
//                ),
//                safeSetTargetPoint(targetPoint),
//                ()-> !arm.motionProfilePathsAtParametricEnd()
//        );
//    }
//
//    private SequentialCommandGroup moveToTargetWhileAvoidingCamera(Point target){
//        return new SequentialCommandGroup(
//                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "MAKING A CAMERA AVOIDANCE MANEUVER"),
//                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
//                new WaitCommand(100),
//
//                safeSetTargetPoint(new Point(0, arm.angleDegrees()),
//                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG ON RETRACTION")
//                ),
//
//                safeSetTargetPoint(new Point(0, target.angleDegrees()),
//                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG WHILE ROTATING TO TARGET")
//                ),
//
//                safeSetTargetPoint(new Point(target.magnitude(), target.angleDegrees()),
//                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG WHILE EXTENDING ARM TO TARGET")
//                ),
//
//                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "END OF CAMERA AVOIDANCE MANEUVER")
//        );
//    }
//
//    public ConditionalCommand extension(double extension){
//        return new ConditionalCommand(
//                new LogCommand("SET ARM POSITION.EXTENSION", Level.SEVERE, "SLIDES ALREADY MOVING, WAITING PREVIOUS COMMAND TO FINISH")
//                        .andThen(new WaitUntilCommand(()-> !arm.areSlidesMoving()).interruptOn(interruptCondition)
//                        ),
//                extensionAndAngleDegrees(extension, arm.getTargetAngleDegrees()),
//                ()-> arm.areSlidesMoving()
//        );
//    }
//
//    public ConditionalCommand extensionRelative(double delta){
//        return extension(arm.extension() + delta);
//    }
//
//    public ConditionalCommand angleDegrees(double angleDegrees){
//        return new ConditionalCommand(
//                new LogCommand("SET ARM POSITION.ANGLE_DEGREES", Level.SEVERE, "ROTATOR ALREADY MOVING, WAITING PREVIOUS COMMAND TO FINISH")
//                        .andThen(new WaitUntilCommand(()-> !arm.isRotatorMoving()).interruptOn(interruptCondition)
//                ),
//                extensionAndAngleDegrees(arm.getTargetExtension(), angleDegrees),
//                ()-> arm.isRotatorMoving()
//        );
//    }
//
//    public ConditionalCommand X(double x, OFFSET_REFERENCE_PLANE reference){
//        return XY(x, arm.getTargetY(), reference);
//    }
//
//    public ConditionalCommand Y(double y, OFFSET_REFERENCE_PLANE reference){
//        return XY(arm.getTargetX(), y, reference);
//    }
//
//    public SequentialCommandGroup MANUALLY_INTERRUPT_CURRENT_COMMAND(){
//        return new SequentialCommandGroup(
//                new InstantCommand(()-> interruptCondition = ()-> true),
//                new WaitCommand(20),
//                new InstantCommand(()-> interruptCondition = ()-> false)
//        );
//    }
//
//
//    private InstantCommand setArmState(ArmState.State state){
//        return new InstantCommand(()-> ArmState.set(state));
//    }
//
//
//    public SequentialCommandGroup intake(double extension, double angle, double twist){
//        return new SequentialCommandGroup(
//                new LogCommand("INTAKE SAMPLE COMMAND", "SKIBIDI HELLO?"),
//
//                new CustomConditionalCommand(
//                        new SequentialCommandGroup(
//                                new LogCommand("INTAKE SAMPLE COMMAND", "INTAKING SAMPLE FROM IN ROBOT STATE"),
//
//                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
//                                new SetClawState(ClawConfiguration.GripperState.OPEN),
//                                new SetArmPositionBACKUP().extension(extension).alongWith(
//                                        new SequentialCommandGroup(
//                                                new WaitUntilCommand(()-> arm.extension() > clamp(extension - 0.1, 0.15 ,1)),
//                                                new SetClawAngle(angle),
//                                                new SetClawTwist(twist)
//                                        )
//                                ),
//                                setArmState(ArmState.State.SAMPLE_INTAKE)
//                        ),
//                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
//                ),
//
//                new LogCommand("SKIBIDI TEST STATE", "CURRENT STATE BEFORE THIS STUPID THING IS: " + ArmState.get().toString()),
//
//                new CustomConditionalCommand(
//                        new SequentialCommandGroup(
//                                retract(),
//                                new LogCommand("INTAKE SAMPLE COMMAND", "INTAKING SAMPLE FROM SAMPLE SCORE OR SPECIMEN SCORE STATES"),
//                                intake(extension, angle, twist)
//                        ),
//                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_SCORE)
//                )
//        );
//    }
//
//
//    public SequentialCommandGroup intakeSample(double extension, double twist) {
//        return intake(extension, ClawConfiguration.VerticalRotation.DOWN.pos, twist);
//    }
//
//    public SequentialCommandGroup intakeSpecimen(double extension) {
//        return intake(extension, 0.8, ClawConfiguration.HorizontalRotation.NORMAL.pos);
//    }
//
//
//    public SequentialCommandGroup retract(){
//        return new SequentialCommandGroup(
//                new CustomConditionalCommand(
//                       new SequentialCommandGroup(
//                               new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SPECIMEN SCORE STATE"),
//
//                               new SetArmPositionBACKUP().extensionRelative(0.2).withTimeout(1000),
//                               new SetClawState(ClawConfiguration.GripperState.OPEN),
//                               new WaitCommand(150),
//                               new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
//                               new WaitCommand(50),
//                               new SetArmPositionBACKUP().extension(0),
//                               new SetClawState(ClawConfiguration.GripperState.CLOSED),
//                               new SetArmPositionBACKUP().angleDegrees(0),
//                               setArmState(ArmState.State.IN_ROBOT)
//                       ),
//                       ()-> ArmState.isCurrentState(ArmState.State.SPECIMEN_SCORE)
//                ),
//
//
//                new CustomConditionalCommand(
//                        new SequentialCommandGroup(
//                                new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SAMPLE SCORE, OR ANY HANG STATE"),
//
//                                new CustomConditionalCommand(
//                                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.HANG)),
//                                        ()-> ArmState.isCurrentState(ArmState.State.HANG_SECOND_STAGE, ArmState.State.HANG_THIRD_STAGE)
//                                ),
//
//                                new SetClawState(ClawConfiguration.GripperState.OPEN),
//                                new WaitCommand(80),
//                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
//                                new WaitCommand(50),
//
//                                new ParallelCommandGroup(
//                                        new SetArmPositionBACKUP().angleDegrees(65),
//                                        new WaitCommand(100).andThen(new SetArmPositionBACKUP().extension(0)),
//                                        new WaitCommand(250).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP))
//                                ),
//
//                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
//                                new SetArmPositionBACKUP().angleDegrees(0),
//                                setArmState(ArmState.State.IN_ROBOT)
//                        ),
//                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.HANG_SECOND_STAGE, ArmState.State.HANG_THIRD_STAGE)
//                ),
//
//
//                new CustomConditionalCommand(
//                        new SequentialCommandGroup(
//                                new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SPECIMEN OR SAMPLE INTAKE STATE"),
//
//                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
//                                new WaitCommand(100),
//                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
//                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
//                                new SetArmPositionBACKUP().extension(0),
//
//                                setArmState(ArmState.State.IN_ROBOT)
//                        ),
//                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_INTAKE, ArmState.State.SPECIMEN_INTAKE)
//                )
//        );
//    }
//
//
//    public SequentialCommandGroup scoreSample(SAMPLE_SCORE_HEIGHT sampleScoreHeight){
//        return new SequentialCommandGroup(
//                new CustomConditionalCommand(
//                        new SequentialCommandGroup(
//                                new LogCommand("SCORE SAMPLE", Level.SEVERE, "SCORING SAMPLE FROM IN ROBOT STATE"),
//
//                                new ParallelCommandGroup(
//                                        new SetArmPositionBACKUP().angleDegrees(110),
//                                        new WaitUntilCommand(()-> arm.angleDegrees() > 60).andThen(new SetArmPositionBACKUP().extension(sampleScoreHeight.extension)),
//                                        new SequentialCommandGroup(
//                                                new WaitUntilCommand(()-> arm.angleDegrees() > 75),
//                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
//                                                new WaitUntilCommand(()-> arm.extension() > sampleScoreHeight.extension - 0.3),
//                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DEPOSIT)
//                                        )
//                                ),
//                                setArmState(ArmState.State.SAMPLE_SCORE)
//                        ),
//                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
//                )
//        );
//    }
//
//
//    public SequentialCommandGroup scoreSpecimen(){
//        return new SequentialCommandGroup(
//                new CustomConditionalCommand(
//                        new SequentialCommandGroup(
//                                new LogCommand("SCORE SPECIMEN", Level.SEVERE, "SCORING SPECIMEN FROM IN ROBOT STATE"),
//                                new SetArmPositionBACKUP().XY(20, 55, OFFSET_REFERENCE_PLANE.FRONT).alongWith(
//                                        new WaitUntilCommand(()-> arm.extension() > 0.1).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN))
//                                ),
//
//                                setArmState(ArmState.State.SPECIMEN_SCORE)
//                        ),
//
//                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
//                )
//        );
//    }
//
//
//    public CustomConditionalCommand level_2_hang(BooleanSupplier gamepadCondition){
//        return new CustomConditionalCommand(
//                new SequentialCommandGroup(
//                        new LogCommand("SECOND STAGE HANG", Level.SEVERE, "STARTING LEVEL 2 HANG COMMAND"),
//
//                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
//                        new SetArmPositionBACKUP().angleDegrees(102.5).alongWith(
//                                new WaitUntilCommand(()-> arm.angleDegrees() >= 70).andThen(new SetArmPositionBACKUP().extension(0.314))
//                        ),
//
//                        new WaitUntilCommand(gamepadCondition),
//                        new SetArmPositionBACKUP().extension(0.1),
//                        new InstantCommand(()-> arm.updateCoefficients(OPERATION_MODE.HANG)),
//
//                        new SetArmPositionBACKUP().angleDegrees(42),
//                        new SetArmPositionBACKUP().extension(0.03),
//
//                        setArmState(ArmState.State.HANG_SECOND_STAGE)
//                ),
//                ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
//        );
//    }
//
//
//    public CustomConditionalCommand level_3_hang(BooleanSupplier gamepadCondition){
//        return new CustomConditionalCommand(
//                new SequentialCommandGroup(
//                        new LogCommand("THIRD STAGE HANG", Level.SEVERE, "STARTING LEVEL 3 HANG COMMAND"),
//
//                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
//
//                        new InstantCommand(()-> VLRSubsystem.getHang().setTargetAngleUP()),
//                        new SetArmPositionBACKUP().angleDegrees(101).alongWith(
//                                new WaitUntilCommand(()-> arm.angleDegrees() > 80).andThen(new SetArmPositionBACKUP().extension(0.314))
//                        ),
//
//                        new WaitUntilCommand(gamepadCondition),
//                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.HANG)),
//
//                        new ParallelCommandGroup(
//                                new SetArmPositionBACKUP().extension(0.15),
//                                new WaitCommand(200).andThen(new SetArmPositionBACKUP().angleDegrees(85)),
//                                new WaitCommand(500).andThen(new InstantCommand(()-> VLRSubsystem.getHang().setPower(0.2)))
//                        ).interruptOn(()-> VLRSubsystem.getHang().analogFeedbackThresholdReached()),
//
//                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(0)),
//                        new SetArmPositionBACKUP().extension(0.3),
//
//                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.NORMAL)),
//                        new SetArmPositionBACKUP().extension(0.888).alongWith(new SetArmPositionBACKUP().angleDegrees(80)),
//
//                        new SetArmPositionBACKUP().angleDegrees(102).alongWith(
//                                new WaitCommand(400).andThen(new SetArmPositionBACKUP().extension(0.83))
//                        ),
//
//                        new WaitUntilCommand(gamepadCondition),
//                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.HANG)),
//
//                        new SetArmPositionBACKUP().extension(0.06).alongWith(
//                                new SequentialCommandGroup(
//                                        new WaitUntilCommand(()-> arm.extension() < 0.3),
//                                        new SetHangPosition(HangConfiguration.TargetPosition.DOWN),
//                                        new SetArmPositionBACKUP().angleDegrees(35)
//                                )
//                        )
//                ),
//                ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
//        );
//    }
//}