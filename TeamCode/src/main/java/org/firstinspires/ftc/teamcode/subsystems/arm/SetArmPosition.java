package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem.getArm;
import org.firstinspires.ftc.teamcode.helpers.commands.ScheduleRuntimeCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OFFSET_REFERENCE_PLANE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OPERATION_MODE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.SAMPLE_SCORE_HEIGHT;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.Point;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.SetPattern;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

import java.util.function.BooleanSupplier;
import java.util.logging.Level;

public class SetArmPosition extends SequentialCommandGroup{
    private final MainArmSubsystem arm;
    private static BooleanSupplier interruptCondition = ()-> true;

    {
        // This needs to be here, since addRequirements needs to be called BEFORE the command is
        // able to run. The running may happen instantly (on super() being called), or at any point
        // in the future, so it's best to call addRequirements as soon as possible, in this case
        // before the constructor ever runs.
        addRequirements(getArm());
    }

    public SetArmPosition() {arm = getArm();}


    private Command safeSetTargetPoint(Point targetPoint, Command ifCameraIsInTheWay){
        return new ConditionalCommand(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SET ARM POS COMMAND", "SETTING TARGET ARM ANGLE TO " + targetPoint.angleDegrees() + " WITH MAGNITUDE " + targetPoint.magnitude()),
                                new CustomConditionalCommand(
                                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.NORMAL)),
                                        ()-> !arm.isCurrentOperationMode(OPERATION_MODE.HANG, OPERATION_MODE.NORMAL_SLOWER)
                                ),

                                new InstantCommand(()-> arm.setTargetPoint(targetPoint)),
                                new WaitCommand(5),
                                new WaitUntilCommand(arm::motionProfilePathsAtParametricEnd),

                                new CustomConditionalCommand(
                                        new InstantCommand(()-> arm.setOperationMode(MainArmConfiguration.OPERATION_MODE.HOLD_POINT)),
                                        ()-> !arm.isCurrentOperationMode(OPERATION_MODE.HANG)
                                ),

                                new ConditionalCommand(
                                        new LogCommand("SET ARM POS COMMAND", Level.WARNING, "TARGET REACHED SUCCESSFULLY"),
                                        new LogCommand("SET ARM POS COMMAND", Level.WARNING, "TARGET NOT REACHED"),
//                                        new SequentialCommandGroup(
//                                                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "ARM TIMEOUT TRIGGERED, WAITING INDEFINITELY FOR MANUAL OVERRIDE"),
//                                                new WaitUntilCommand(arm::reachedTargetPosition).interruptOn(interruptCondition),
//                                                new ConditionalCommand(
//                                                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "WAIT INTERRUPTED, HOPE THE DRIVER KNOWS WHAT HES DOING"),
//                                                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "ARM MAGICALLY REACHED TARGET POSITION WITH NO OVERRIDE"),
//                                                        interruptCondition
//                                                )
//                                        ),
                                        arm::reachedTargetPosition
                                )
                        ),

                        ifCameraIsInTheWay,
                        ()-> !arm.isCameraInTheWayToNewTarget(targetPoint)
                ),

                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "TARGET POINT IS INVALID, SKIPPING"),
                ()-> arm.isTargetPointValid(targetPoint.angleDegrees(), targetPoint.magnitude())
        );
    }


    private Command safeSetTargetPoint(Point point, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return safeSetTargetPoint(point, moveToTargetWhileAvoidingCamera(point, gamePieceType));
    }

    public Command extensionAndAngleDegrees(double magnitude, double angleDegrees, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return safeSetTargetPoint(new Point(magnitude, angleDegrees), gamePieceType);
    }

    public Command extensionAndAngleDegrees(double magnitude, double angleDegrees){
        return safeSetTargetPoint(new Point(magnitude, angleDegrees), MainArmConfiguration.GAME_PIECE_TYPE.SPECIMEN);
    }

    public Command XY(double x_cm, double y_cm, OFFSET_REFERENCE_PLANE reference, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return new ScheduleRuntimeCommand(
                ()-> new ConditionalCommand(
                        new LogCommand("SET ARM POSITION.XY", Level.SEVERE, "ARM ALREADY MOVING, WAITING PREVIOUS COMMAND TO FINISH")
                                .andThen(new WaitUntilCommand(arm::motionProfilePathsAtParametricEnd).interruptOn(interruptCondition)
                        ),
                        safeSetTargetPoint(arm.calculateTargetPointFromRealWordCoordinates(x_cm, y_cm, reference), gamePieceType),
                        ()-> !arm.motionProfilePathsAtParametricEnd()
                )
        );
    }


    private Command moveToTargetWhileAvoidingCamera(Point target, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return new SequentialCommandGroup(
                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "MAKING A CAMERA AVOIDANCE MANEUVER"),

                new ScheduleRuntimeCommand(
                        ()-> safeSetTargetPoint(new Point(0, arm.getTargetAngleDegrees()),
                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG ON RETRACTION"))
                ),

                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, ()-> ("FIRST SECTION PASSED, NOW GOING TO: " + target.angleDegrees() + "; 0")),

                new CustomConditionalCommand(
                        new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                        ()-> gamePieceType == MainArmConfiguration.GAME_PIECE_TYPE.SPECIMEN
                ),

                new ScheduleRuntimeCommand(
                        ()-> safeSetTargetPoint(new Point(0, target.angleDegrees()),
                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG WHILE ROTATING TO TARGET"))
                ),


                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, ()-> ("SECOND SECTION PASSED, NOW GOING TO: " + target.angleDegrees() + target.magnitude())),

                new ScheduleRuntimeCommand(
                        ()-> safeSetTargetPoint(new Point(target.magnitude(), target.angleDegrees()),
                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG WHILE EXTENDING ARM TO TARGET")
                )),

                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "END OF CAMERA AVOIDANCE MANEUVER")
        );
    }

    public Command extension(double extension){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        new LogCommand("SET ARM POSITION.EXTENSION", Level.SEVERE, "SLIDES ALREADY MOVING"),
                        ()-> (arm.areSlidesMoving() && arm.getTargetExtension() != extension)
                ),

                new ScheduleRuntimeCommand(()-> new SetArmPosition().extensionAndAngleDegrees(extension, arm.getTargetAngleDegrees()))

        );
    }

    public Command extensionRelative(double delta){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        new LogCommand("SET ARM POSITION.EXTENSION", Level.SEVERE, "SLIDES ALREADY MOVING, WAITING PREVIOUS COMMAND TO FINISH"),
                        ()-> (arm.areSlidesMoving() && arm.getTargetExtension() != arm.getTargetExtension() + delta)
                ),

                new ScheduleRuntimeCommand(()-> new SetArmPosition().extensionAndAngleDegrees(arm.getTargetExtension() + delta, arm.getTargetAngleDegrees()))
        );
    }


    public Command angleDegrees(double angleDegrees){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        new LogCommand("SET ARM POSITION.ANGLE_DEGREES", Level.SEVERE, "ROTATOR ALREADY MOVING, WAITING PREVIOUS COMMAND TO FINISH"),
                        ()-> (arm.isRotatorMoving() && arm.getTargetAngleDegrees() != angleDegrees)
                ),

                new ScheduleRuntimeCommand(()-> new SetArmPosition().extensionAndAngleDegrees(arm.getTargetExtension(), angleDegrees))
        );
    }

    public Command X(double x, OFFSET_REFERENCE_PLANE reference, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return new ScheduleRuntimeCommand(() -> {
            double currentY = arm.getTargetY();
            return new SetArmPosition().XY(x, currentY, reference, gamePieceType);
        });
    }

    public Command Y(double y, OFFSET_REFERENCE_PLANE reference, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return new ScheduleRuntimeCommand(() -> {
            double currentX = arm.getTargetX();
            return new SetArmPosition().XY(currentX, y, reference, gamePieceType);
        });
    }

    public Command MANUALLY_INTERRUPT_CURRENT_COMMAND(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> interruptCondition = ()-> true),
                new WaitCommand(20),
                new InstantCommand(()-> interruptCondition = ()-> false)
        );
    }

    public Command SET_INTERRUPT_CONDITION(BooleanSupplier state){
        return new InstantCommand(()-> interruptCondition = state);
    }

    public Command setArmState(ArmState.State state){
        return new InstantCommand(()-> ArmState.set(state));
    }


    public Command setArmOperationMode(OPERATION_MODE operationMode){
        return new InstantCommand(()-> arm.setOperationMode(operationMode));
    }


    private Command intake(double extension, double angle, double twist){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        retract(),
                        ()-> !ArmState.isCurrentState(ArmState.State.IN_ROBOT, ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_SCORE_FRONT, ArmState.State.SPECIMEN_SCORE_BACK)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("INTAKE SAMPLE COMMAND", "INTAKING SAMPLE FROM IN ROBOT STATE"),

                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(twist),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new SetArmPosition().extensionAndAngleDegrees(extension, 4).alongWith(
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentExtension() > clamp(extension - 0.25, 0.08 ,1)),
                                                new SetClawAngle(angle)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentExtension() > extension - 0.05),
                                                new SetArmPosition().angleDegrees(0)
                                        )
                                ),
                                new WaitCommand(20)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("INTAKE SAMPLE COMMAND", "INTAKING SAMPLE FROM IN SAMPLE SCORE STATE"),

                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(70),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new WaitCommand(100),

                                new ParallelCommandGroup(
                                        new SetArmPosition().extensionAndAngleDegrees(0, 52, MainArmConfiguration.GAME_PIECE_TYPE.SAMPLE),
                                        new WaitCommand(350).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP))
                                ),

                                new WaitUntilCommand(()-> arm.currentExtension() < 0.1),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(2).andThen(new SetClawTwist(twist)),
                                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),

                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentAngleDegrees() < 40),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),

                                                new ParallelCommandGroup(
                                                        new SetArmPosition().extension(extension),
                                                        new SequentialCommandGroup(
                                                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                                                new WaitUntilCommand(()-> arm.currentExtension() > clamp(extension - 0.25, 0.08 ,1)),
                                                                new SetClawAngle(angle),
                                                                new WaitCommand(200),
                                                                new SetArmPosition().angleDegrees(0),
                                                                new WaitCommand(50)
                                                        )
                                                )
                                        )
                                )
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_SCORE_BACK)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SPECIMEN SCORE STATE"),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(100),

                                new ParallelCommandGroup(
                                        new SetArmPosition().extension(0),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                new SetClawAngle(0.65),
                                                new WaitCommand(200),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentExtension() < 0.13),
                                                new SetArmPosition().angleDegrees(1.5)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentAngleDegrees() < 15),
                                                new ParallelCommandGroup(
                                                        new SetClawAngle(0.68),
                                                        new SetArmPosition().extension(extension),
                                                        new SetClawTwist(twist),
                                                        new SequentialCommandGroup(
                                                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                                                new WaitUntilCommand(()-> arm.currentExtension() > clamp(extension - 0.25, 0.08 ,1)),
                                                                new SetClawAngle(angle),
                                                                new WaitCommand(200),
                                                                new SetArmPosition().angleDegrees(0),
                                                                new WaitCommand(30)
                                                        )
                                                )
                                        )
                                )
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SPECIMEN_SCORE_FRONT)
                )
        );
    }

    public Command intakeSample(double extension) {
        return intake(extension, ClawConfiguration.VerticalRotation.DOWN.pos, ClawConfiguration.HorizontalRotation.NORMAL.pos)
                .andThen(setArmState(ArmState.State.SAMPLE_INTAKE));
    }

    public Command intakeSampleAuto(double extension, double twist) {
        return intake(extension, ClawConfiguration.VerticalRotation.DOWN.pos, twist)
                .andThen(setArmState(ArmState.State.SAMPLE_INTAKE));
    }

    public Command intakeSpecimen(double extension) {
        return intake(extension, 0.88, ClawConfiguration.HorizontalRotation.NORMAL.pos)
                .andThen(setArmState(ArmState.State.SPECIMEN_INTAKE));
    }

    public Command retract(){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                       new SequentialCommandGroup(
                               new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SPECIMEN SCORE STATE"),

                               new SetClawState(ClawConfiguration.GripperState.OPEN),
                               new WaitCommand(90),

                               new ParallelCommandGroup(
                                       new SequentialCommandGroup(
                                               new WaitCommand(50),
                                               new SetClawAngle(0.5),
                                               new WaitCommand(130),
                                               new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                               new SetClawAngle(ClawConfiguration.VerticalRotation.UP)
                                       ),
                                       new SequentialCommandGroup(
                                               new SetArmPosition().extension(0),
                                               new SetArmPosition().angleDegrees(0)
                                       )
                               ),
                               setArmState(ArmState.State.IN_ROBOT)
                       ),
                       ()-> ArmState.isCurrentState(ArmState.State.SPECIMEN_SCORE_FRONT)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SAMPLE SCORE STATE"),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(70),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new WaitCommand(60),

                                new ParallelCommandGroup(
                                        new SetArmPosition().extensionAndAngleDegrees(0, 52, MainArmConfiguration.GAME_PIECE_TYPE.SAMPLE),
                                        new WaitCommand(250).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP))
                                ),

                                new WaitUntilCommand(()-> arm.currentExtension() < 0.2),
                                new WaitUntilCommand(()-> arm.currentExtension() < 0.1).withTimeout(1000),
                                new SetArmPosition().angleDegrees(0),
                                setArmState(ArmState.State.IN_ROBOT)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_SCORE_BACK)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM ANY HANG STATE"),

                                new CustomConditionalCommand(
                                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.HANG)),
                                        ()-> ArmState.isCurrentState(ArmState.State.HANG_SECOND_STAGE, ArmState.State.HANG_THIRD_STAGE)
                                ),

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
                        ()-> ArmState.isCurrentState(ArmState.State.HANG_SECOND_STAGE, ArmState.State.HANG_THIRD_STAGE)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SPECIMEN OR SAMPLE INTAKE STATE"),

                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new WaitCommand(240),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                new SetArmPosition().extension(0),

                                setArmState(ArmState.State.IN_ROBOT)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_INTAKE, ArmState.State.SPECIMEN_INTAKE)
                )
        );
    }


    public Command scoreSample(SAMPLE_SCORE_HEIGHT sampleScoreHeight){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        retract(),
                        ()-> !ArmState.isCurrentState(ArmState.State.IN_ROBOT, ArmState.State.SAMPLE_INTAKE)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SCORE SAMPLE", Level.SEVERE, "SCORING SAMPLE FROM IN ROBOT STATE"),

                                new ParallelCommandGroup(
                                        new WaitCommand(300).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN)),
                                        new SetArmPosition().angleDegrees(101),
                                        new WaitUntilCommand(()-> arm.currentAngleDegrees() > 17).andThen(new SetArmPosition().extension(sampleScoreHeight.extension)),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentExtension() > sampleScoreHeight.extension - 0.27),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DEPOSIT)
                                        )
                                ),
                                setArmState(ArmState.State.SAMPLE_SCORE)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SCORE SAMPLE", Level.SEVERE, "SCORING SAMPLE FROM IN ROBOT STATE"),

                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new WaitCommand(140),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),

                                new ParallelCommandGroup(
                                        new SetArmPosition().extension(0),

                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentExtension() < 0.2),
                                                new ParallelCommandGroup(
                                                        new WaitCommand(300).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN)),
                                                        new SetArmPosition().angleDegrees(101),
                                                        new WaitUntilCommand(()-> arm.currentAngleDegrees() > 17).andThen(new SetArmPosition().extension(sampleScoreHeight.extension)),
                                                        new SequentialCommandGroup(
                                                                new WaitUntilCommand(()-> arm.currentExtension() > sampleScoreHeight.extension - 0.27),
                                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DEPOSIT)
                                                        )
                                                )
                                        )
                                ),
                                setArmState(ArmState.State.SAMPLE_SCORE)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_INTAKE)
                )
        );
    }


    public Command scoreSpecimenFront(){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        retract(),
                        ()-> !ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SCORE SPECIMEN FRONT", Level.SEVERE, "SCORING SPECIMEN FRONT FROM IN ROBOT STATE"),

                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(63),

                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentAngleDegrees() > 20),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                                new ParallelCommandGroup(
                                                        new SetArmPosition().extension(0.37),
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(100)
                                                        )
                                                )
                                        )
                                ),
                                setArmState(ArmState.State.SPECIMEN_SCORE_FRONT)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                )
        );
    }


    public Command scoreSpecimenBack(){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        retract(),
                        ()-> !ArmState.isCurrentState(ArmState.State.IN_ROBOT, ArmState.State.SPECIMEN_INTAKE)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SCORE SPECIMEN BACK", Level.SEVERE, "SCORING SPECIMEN BACK FROM IN ROBOT STATE"),
                                new SetArmPosition().extensionAndAngleDegrees(0.28, 102, MainArmConfiguration.GAME_PIECE_TYPE.SAMPLE).alongWith(
                                        new WaitUntilCommand(()-> arm.currentAngleDegrees() > 80).andThen(new SetClawAngle(0.24))
                                ),
                                setArmState(ArmState.State.SPECIMEN_SCORE_BACK)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SCORE SPECIMEN BACK", Level.SEVERE, "SCORING SPECIMEN BACK FROM IN SPECIMEN INTAKE STATE"),

                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new WaitCommand(120),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),


                                new ParallelCommandGroup(
                                        new SetArmPosition().extension(0),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentExtension() < 0.2),
                                                new ParallelCommandGroup(
                                                        new SetArmPosition().angleDegrees(102),
                                                        new SequentialCommandGroup(
                                                                new WaitUntilCommand(()-> (arm.currentAngleDegrees() > 18 && !arm.areSlidesMoving())),
                                                                new SetArmPosition().extension(0.3)
                                                        ),
                                                        new WaitUntilCommand(()-> arm.currentAngleDegrees() > 80).andThen(new SetClawAngle(0.265))
                                                )
                                        )
                                ),
                                setArmState(ArmState.State.SPECIMEN_SCORE_BACK)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SPECIMEN_INTAKE)
                )
        );
    }


    public Command level2Hang(BooleanSupplier gamepadCondition){
        return new CustomConditionalCommand(
                new SequentialCommandGroup(
                        new LogCommand("SECOND STAGE HANG", Level.SEVERE, "STARTING LEVEL 2 HANG COMMAND"),

                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                        new SetArmPosition().angleDegrees(94).alongWith(
                                new WaitUntilCommand(()-> arm.currentAngleDegrees() > 45).andThen(new SetArmPosition().extension(0.314))),

                        new WaitUntilCommand(gamepadCondition),
                        new SetArmPosition().setArmOperationMode(OPERATION_MODE.HANG),
                        new ParallelCommandGroup(
                                new SetArmPosition().extension(0.13),
                                new WaitCommand(100).andThen(new SetArmPosition().angleDegrees(48))
//                                new SequentialCommandGroup(
//                                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(0.8)),
//                                        new WaitCommand(450),
//                                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(0.3))
//                                ),
                        ),

                        new WaitCommand(250),
                        new SetArmPosition().extension(0.0),


                        setArmState(ArmState.State.HANG_SECOND_STAGE)
                ),
                ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
        );
    }


    public Command level_3_hang(BooleanSupplier gamepadCondition){
        return new CustomConditionalCommand(
                new SequentialCommandGroup(
                        new LogCommand("THIRD STAGE HANG", Level.SEVERE, "STARTING LEVEL 3 HANG COMMAND"),

                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),

                        new SetArmPosition().angleDegrees(94).alongWith(
                                new WaitUntilCommand(()-> arm.currentAngleDegrees() > 45).andThen(new SetArmPosition().extension(0.314))),

                        new WaitUntilCommand(gamepadCondition),

                        new ParallelCommandGroup(
                                new SetArmPosition().extension(0.13),
                                new WaitCommand(300).andThen(new SetArmPosition().angleDegrees(82)),
                                new SequentialCommandGroup(
                                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(0.8)),
                                        new WaitCommand(450),
                                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(0.1))
                                ),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN)
                        ),

                        new WaitCommand(500),
                        new SetArmPosition().extension(0.24),
                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(0)),

                        new SetArmPosition().extensionAndAngleDegrees(0.89, 80),
                        new SetArmPosition().angleDegrees(86),
                        new SetArmPosition().extension(0.81),

                        new InstantCommand(()->VLRSubsystem.getArm().enableRotatorPowerOverride(0)),
                        new InstantCommand(()->VLRSubsystem.getArm().enableSlidePowerOverride(0)),

                        new InstantCommand(()-> VLRSubsystem.getInstance(ClawSubsystem.class).disable()),
                        new InstantCommand(()-> VLRSubsystem.getInstance(Chassis.class).stop()),
                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.HANG)),
                        new SetPattern().blank()


                        //REMOVE LATER:
//                        new InstantCommand(()->VLRSubsystem.getArm().disableSlidePowerOverride()),
//                        new InstantCommand(()->VLRSubsystem.getArm().disableRotatorPowerOverride())


//                        new WaitUntilCommand(gamepadCondition),
//                        new InstantCommand(()->VLRSubsystem.getArm().disableSlidePowerOverride()),
//                        new InstantCommand(()->VLRSubsystem.getArm().disableRotatorPowerOverride()),
//                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(0.1)),
//                        new InstantCommand(()-> VLRSubsystem.getArm().setSlidePowerLimit(0.9)),
//
//                        new ParallelCommandGroup(
//                                new SetArmPosition().extension(0),
//
//                                new SequentialCommandGroup(
//                                        new WaitUntilCommand(()-> arm.currentExtension() < 0.3),
//                                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(-0.1)),
//                                        new WaitCommand(600),
//                                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(0)),
//                                        new InstantCommand(()-> VLRSubsystem.getHang().disable())
//                                ),
//
//                                new SequentialCommandGroup(
//                                        new WaitCommand(300),
//                                        new SetArmPosition().angleDegrees(105),
//                                        new WaitUntilCommand(()-> arm.currentExtension() < 0.36),
//                                        new SetArmPosition().angleDegrees(50),
//                                        new WaitCommand(120),
//                                        new SetArmPosition().angleDegrees(30)
//                                )
//                                new SequentialCommandGroup(
//                                        new PerpetualCommand(
//                                                new SequentialCommandGroup(
//                                                        new InstantCommand(()->VLRSubsystem.getArm().enableSlidePowerOverride(0)),
//                                                        new WaitCommand(1000),
//                                                        new InstantCommand(()->VLRSubsystem.getArm().disableSlidePowerOverride()),
//                                                        new InstantCommand(()-> arm.setRotatorPowerLimit(0.75)),
//                                                        new WaitCommand(1000)
//                                                )
//                                        ).withTimeout(3000),
//                                        new InstantCommand(()->VLRSubsystem.getArm().disableSlidePowerOverride()),
//                                        new LogCommand("SKIBIDI", Level.SEVERE, "SKIBIDI ROTATOR SKIBIDI DISABLED"),
//                                        new InstantCommand(()-> VLRSubsystem.getArm().setRotatorPowerLimit(0.8))
//                                ),
//
//                                new SequentialCommandGroup(
//                                        new WaitCommand(500),
//                                        new PerpetualCommand(
//                                                new SequentialCommandGroup(
//                                                        new InstantCommand(()-> arm.setThirdSlideMotorEnable(false)),
//                                                        new WaitCommand(1000),
//                                                        new InstantCommand(()-> arm.setThirdSlideMotorEnable(true)),
//                                                        new WaitCommand(1000)
//                                                )
//                                        ).withTimeout(3000),
//                                        new LogCommand("SKIBIDI", Level.SEVERE, "SKIBIDI SLIDE SKIBIDI DISABLED"),
//                                        new InstantCommand(()-> arm.setThirdSlideMotorEnable(true)),
//                                        new WaitCommand(400),
//                                        new InstantCommand(()-> arm.setThirdSlideMotorEnable(false))
//                                )
                        //)
                ),
                ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
        );
    }


    public Command retractAfterAuto(){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                new SetClawState(ClawConfiguration.GripperState.CLOSED)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new SetArmPosition().extensionAndAngleDegrees(0.71, 95, MainArmConfiguration.GAME_PIECE_TYPE.SAMPLE),
                                                  new WaitCommand(100),
                                                new SetArmPosition().extensionAndAngleDegrees(0, 52, MainArmConfiguration.GAME_PIECE_TYPE.SAMPLE)
                                        ),
                                        new WaitCommand(150).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP))
                                ),
                                new WaitCommand(200),
                                new SetArmPosition().angleDegrees(0),
                                new SetArmPosition().setArmState(ArmState.State.IN_ROBOT)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                new WaitCommand(400),
                                new SetArmPosition().extensionAndAngleDegrees(0, 55),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new WaitCommand(500),
                                new SetArmPosition().angleDegrees(0),
                                new SetArmPosition().setArmState(ArmState.State.IN_ROBOT)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SPECIMEN_SCORE_BACK)
                )
        );
    }
}