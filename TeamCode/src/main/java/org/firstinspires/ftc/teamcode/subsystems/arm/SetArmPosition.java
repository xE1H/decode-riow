package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem.getArm;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.ScheduleRuntimeCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.BrakeArmMotors;
import org.firstinspires.ftc.teamcode.helpers.utils.Point;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OFFSET_REFERENCE_PLANE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OPERATION_MODE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.SAMPLE_SCORE_HEIGHT;
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
                                new WaitCommand(40),

                                new CustomConditionalCommand(
                                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.HOLD_POINT)),
                                        ()-> !arm.isCurrentOperationMode(OPERATION_MODE.HANG)
                                )

//                                new ConditionalCommand(
//                                        new LogCommand("SET ARM POS COMMAND", Level.WARNING, "TARGET REACHED SUCCESSFULLY"),
//                                        new LogCommand("SET ARM POS COMMAND", Level.WARNING, "TARGET NOT REACHED"),
//                                        arm::reachedTargetPosition
//                                )
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

    public Command extensionAndAngleDegreesNOTSAFEJUSTFORHANG(double magnitude, double angleDegrees){
        return new SequentialCommandGroup(
                new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.HANG)),
                new InstantCommand(()-> arm.setTargetPoint(new Point(magnitude, angleDegrees)))
        );
    }

    public Command extensionAndAngleDegrees(double magnitude, double angleDegrees){
        return safeSetTargetPoint(new Point(magnitude, angleDegrees), MainArmConfiguration.GAME_PIECE_TYPE.SPECIMEN);
    }

    public Command XY(double x_cm, double y_cm, OFFSET_REFERENCE_PLANE reference, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return new ScheduleRuntimeCommand(
                ()-> new ConditionalCommand(
                        new LogCommand("SET ARM POSITION.XY", Level.SEVERE, "ARM ALREADY MOVING, WAITING PREVIOUS COMMAND TO FINISH")
                                .andThen(new WaitUntilCommand(arm::motionProfilePathsAtParametricEnd)
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
                        new LogCommand("SET ARM POSITION.ANGLE_DEGREES", Level.SEVERE, "ROTATOR ALREADY MOVING"),
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

    public Command setArmState(ArmState.State state){
        return new InstantCommand(()-> ArmState.set(state));
    }


    public Command setArmOperationMode(OPERATION_MODE operationMode){
        return new InstantCommand(()-> arm.setOperationMode(operationMode));
    }

    private Command grabSequence(double extension, double angle, double twist){
        return new SequentialCommandGroup(
                new SetClawAngle(angle),
                new WaitCommand(140),
                new SetClawTwist(twist),
                new WaitUntilCommand(()-> arm.currentExtension() > extension - 0.265),
                new SetArmPosition().angleDegrees(0),
                new WaitCommand((long) ((Math.abs(0.5 - twist) * 50)))
        );
    }


    private Command intake(double extension, double angle, double twist, boolean specimenGrab){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        retract(),
                        ()-> !ArmState.isCurrentState(ArmState.State.IN_ROBOT, ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_SCORE_FRONT, ArmState.State.SPECIMEN_SCORE_BACK)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("INTAKE SAMPLE COMMAND", "INTAKING SAMPLE FROM IN ROBOT STATE"),

                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new ParallelCommandGroup(
                                        new SetArmPosition().extensionAndAngleDegrees(extension, 5),
                                        new WaitCommand(50).andThen(grabSequence(extension, angle, twist))
                                )
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("INTAKE SAMPLE COMMAND", "INTAKING SAMPLE FROM IN SAMPLE SCORE STATE"),

                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(110),
                                new CustomConditionalCommand(
                                        new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN).andThen(new WaitCommand(90)),
                                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE)
                                ),

                                new ParallelCommandGroup(
                                        new SetArmPosition().extensionAndAngleDegrees(0, 63, MainArmConfiguration.GAME_PIECE_TYPE.SAMPLE),
                                        new WaitCommand(250).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP)),

                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentExtension() < 0.19),
                                                new ParallelCommandGroup(
                                                        new SetArmPosition().angleDegrees(4),
                                                        new WaitUntilCommand(()-> arm.currentAngleDegrees() < 39).andThen(new SetArmPosition().extension(extension)),

                                                        new WaitCommand(240).andThen(
                                                                new ConditionalCommand(
                                                                        new SequentialCommandGroup(
                                                                                new SetClawAngle(angle),
                                                                                new WaitCommand(80),
                                                                                new SetArmPosition().angleDegrees(0),
                                                                                new WaitCommand(50)
                                                                        ),
                                                                        grabSequence(extension, angle, twist),
                                                                        ()-> specimenGrab
                                                                )
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
                                new WaitCommand(80),

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
                                                new SetArmPosition().angleDegrees(2)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentAngleDegrees() < 21),
                                                new ParallelCommandGroup(
                                                        new SetArmPosition().extension(extension),
                                                        new ConditionalCommand(
                                                                new SequentialCommandGroup(
                                                                        new WaitCommand(50),
                                                                        new SetClawAngle(angle),
                                                                        new WaitUntilCommand(()-> arm.currentExtension() > extension - 0.2),
                                                                        new SetArmPosition().angleDegrees(0)
                                                                ),
                                                                grabSequence(extension, angle, twist),
                                                                ()-> specimenGrab
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
        return intake(extension, ClawConfiguration.VerticalRotation.DOWN.pos, ClawConfiguration.HorizontalRotation.NORMAL.pos, false)
                .andThen(setArmState(ArmState.State.SAMPLE_INTAKE));
    }

    public Command intakeSampleAuto(double extension, double twist) {
        return intake(extension, ClawConfiguration.VerticalRotation.DOWN.pos, twist, false)
                .andThen(setArmState(ArmState.State.SAMPLE_INTAKE));
    }

    public Command intakeSpecimen(double extension) {
        return intake(extension, 0.825, ClawConfiguration.HorizontalRotation.NORMAL.pos, true)
                .andThen(setArmState(ArmState.State.SPECIMEN_INTAKE));
    }

    public Command retract(){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                       new SequentialCommandGroup(
                               new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SPECIMEN SCORE STATE"),

                               new SetClawState(ClawConfiguration.GripperState.OPEN),
                               new WaitCommand(100),

                               new ParallelCommandGroup(
                                       new SetArmPosition().extension(0),
                                       new SequentialCommandGroup(
                                               new WaitCommand(50),
                                               new SetClawAngle(0.7),
                                               new WaitCommand(150),
                                               new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                               new SetClawAngle(ClawConfiguration.VerticalRotation.UP)
                                       ),
                                       new SequentialCommandGroup(
                                               new WaitUntilCommand(()-> arm.currentExtension() < 0.14),
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
                                new ConditionalCommand(
                                        new WaitCommand(80),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN)
                                        ),
                                        ()-> ArmState.isCurrentState(ArmState.State.SPECIMEN_SCORE_BACK)
                                ),

                                new ParallelCommandGroup(
                                        new SetArmPosition().extensionAndAngleDegrees(0, 64, MainArmConfiguration.GAME_PIECE_TYPE.SAMPLE),
                                        new WaitUntilCommand(()-> VLRSubsystem.getArm().currentExtension() < 0.185).andThen(new SetArmPosition().angleDegrees(0)),

                                        new WaitCommand(200).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP))
                                ),

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
                                new WaitCommand(200),
                                new ParallelCommandGroup(
                                        new SetArmPosition().extension(0),
                                        new SequentialCommandGroup(
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                                new WaitCommand(150),
                                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL)
                                        )
                                ),

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
                                        new SetClawTwist(1),
                                        new SetArmPosition().angleDegrees(108).andThen(new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL)),
                                        new WaitUntilCommand(()-> arm.currentAngleDegrees() > 19.5).andThen(new SetArmPosition().extension(sampleScoreHeight.extension)),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentExtension() > sampleScoreHeight.extension - 0.26),
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
                                new WaitCommand(200),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),

                                new ParallelCommandGroup(
                                        new SetArmPosition().extension(0),

                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentExtension() < 0.2),
                                                new ParallelCommandGroup(
                                                        new SetClawTwist(1),
                                                        new WaitCommand(300).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN)),
                                                        new SetArmPosition().angleDegrees(108).andThen(new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL)),
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
                                        new SetArmPosition().angleDegrees(69),

                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentAngleDegrees() > 19),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                                new SetArmPosition().extension(0.39)
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
                                //new SetArmPosition().angleDegrees(110.5),

                                setArmOperationMode(OPERATION_MODE.NORMAL_SLOWER),
                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(110.5),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> (arm.currentAngleDegrees() > 40)),
                                                new SetArmPosition().extension(0.33)
                                        ),
                                        new WaitUntilCommand(()-> arm.currentAngleDegrees() > 80).andThen(new SetClawAngle(0.15))
                                ),
                                setArmState(ArmState.State.SPECIMEN_SCORE_BACK)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SCORE SPECIMEN BACK", Level.SEVERE, "SCORING SPECIMEN BACK FROM SPECIMEN INTAKE STATE"),

                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new WaitCommand(200),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),


                                new ParallelCommandGroup(
                                        new WaitCommand(100).andThen(new SetClawTwist(0.25)),
                                        new SetArmPosition().extension(0),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentExtension() < 0.15),
                                                setArmOperationMode(OPERATION_MODE.NORMAL_SLOWER),
                                                new ParallelCommandGroup(
                                                        new WaitCommand(300).andThen(new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL)),
                                                        new SetArmPosition().angleDegrees(110.5),
                                                        new SequentialCommandGroup(
                                                                new WaitUntilCommand(()-> (arm.currentAngleDegrees() > 20 && !arm.areSlidesMoving())),
                                                                new SetArmPosition().extension(0.31) // 0.31
                                                        ),
                                                        new WaitUntilCommand(()-> arm.currentAngleDegrees() > 80).andThen(new SetClawAngle(0.15))
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
                        new SetArmPosition().angleDegrees(100).alongWith(
                                new WaitUntilCommand(()-> arm.currentAngleDegrees() > 45).andThen(new SetArmPosition().extension(0.314))),

                        new WaitUntilCommand(gamepadCondition),
                        new ParallelCommandGroup(
                                new SetArmPosition().extension(0.1),
                                new WaitCommand(200).andThen(new SetArmPosition().angleDegrees(48))
//                                new SequentialCommandGroup(
//                                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(0.8)),
//                                        new WaitCommand(450),
//                                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(0.3))
//                                )
                        ),
                        new SetArmPosition().setArmOperationMode(OPERATION_MODE.HANG),

                        new WaitCommand(250),
                        new SetArmPosition().extension(0.02),

                        new WaitUntilCommand(gamepadCondition),
                        new SetArmPosition().extension(0.12),
                        new WaitCommand(200),
                        new SetArmPosition().angleDegrees(57),

                        //new SetArmPosition().extension(0.0),
                        setArmState(ArmState.State.HANG_SECOND_STAGE)
                ),
                ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
        );
    }


    public Command level_3_hang(BooleanSupplier gamePadCondition, BooleanSupplier hangStartCondition){
        return new CustomConditionalCommand(
                new SequentialCommandGroup(
                        new LogCommand("THIRD STAGE HANG", Level.SEVERE, "STARTING LEVEL 3 HANG COMMAND"),

                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),

                        new SetArmPosition().angleDegrees(100).alongWith(
                                new WaitUntilCommand(() -> arm.currentAngleDegrees() > 45).andThen(new SetArmPosition().extension(0.314))),

                        new WaitUntilCommand(gamePadCondition),

                        new ParallelCommandGroup(
                                new SetArmPosition().extension(0.13),
                                new WaitCommand(300).andThen(new SetArmPosition().setArmOperationMode(OPERATION_MODE.NORMAL_SLOWER), new SetArmPosition().angleDegrees(82)),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> VLRSubsystem.getHang().setPower(0.65)),
                                        new WaitCommand(450),
                                        new InstantCommand(() -> VLRSubsystem.getHang().setPower(0.1))
                                ),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN)
                        ),

                        new WaitCommand(500),
                        new SetArmPosition().extension(0.24),
                        // new InstantCommand(()-> VLRSubsystem.getHang().setPower(0)),

                        new SetArmPosition().setArmOperationMode(OPERATION_MODE.NORMAL_SLOWER),
                        new SetArmPosition().extensionAndAngleDegrees(0.85, 82),
                        new WaitCommand(300),

                        new SetArmPosition().setArmOperationMode(OPERATION_MODE.NORMAL_SLOWER),
                        new SetArmPosition().angleDegrees(94.5),
                        new WaitCommand(800),
                        new SetArmPosition().extension(0.75),

                        new WaitCommand(500),

                        new InstantCommand(() -> VLRSubsystem.getArm().proceedToLevel3()),
                        new WaitUntilCommand(hangStartCondition),
                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),

                        new InstantCommand(() -> VLRSubsystem.getArm().setOperationMode(OPERATION_MODE.HANG)),
                        new InstantCommand(() -> VLRSubsystem.getHang().setPower(0.1)),

                        new WaitUntilCommand(() -> VLRSubsystem.getArm().currentExtension() < 0.22),
                        new InstantCommand(() -> VLRSubsystem.getHang().setPower(-0.6)),
                        new WaitCommand(120),
                        new InstantCommand(() -> VLRSubsystem.getHang().setPower(0))
//                        new WaitUntilCommand(() -> VLRSubsystem.getArm().currentExtension() < 0.05),

                        //new InstantCommand(() -> VLRSubsystem.getHang().setPower(0.1)),
//                        new InstantCommand(()-> VLRSubsystem.getArm().setLevel3hangFinished())
                ),
                ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
        );
    }

//    public Command level_3_hang(BooleanSupplier gamePadCondition, BooleanSupplier hangStartCondition){
//        return new CustomConditionalCommand(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> VLRSubsystem.getHang().setPower(0)),
//                        new WaitUntilCommand(() -> VLRSubsystem.getArm().currentExtension() < 0.05 && VLRSubsystem.getArm().currentAngleDegrees() > 45),
//                        new WaitCommand(2000),
////                        new InstantCommand(() -> VLRSubsystem.getHang().setPower(0.2)),
//
//                        new WaitUntilCommand(gamePadCondition),
//                        new InstantCommand(()-> VLRSubsystem.getArm().enableSlidePowerOverride(0)),
//                        new InstantCommand(()-> VLRSubsystem.getArm().enableRotatorPowerOverride(0))
//                ),
//                ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
//        );
//    }


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
                                new WaitCommand(100),
                                new ResetRotator(),
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