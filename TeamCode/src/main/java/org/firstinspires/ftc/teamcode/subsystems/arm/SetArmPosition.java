package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem.getArm;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.COORDINATE_IDENTIFIER;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OFFSET_REFERENCE_PLANE;
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

    private ConditionalCommand safeSetTargetPoint(double magnitudeOrX, double thetaOrY, COORDINATE_IDENTIFIER identifier, InstantCommand setCommand){
        return new ConditionalCommand(
                        new SequentialCommandGroup(
                                setCommand
                        )
                        .raceWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new InstantCommand(()-> System.out.println("LOGGER TEST"))
                                )
                        ),
                        new InstantCommand(),
                        ()-> true
        );
    }

//    private ConditionalCommand safeSetTargetPoint(double magnitudeOrX, double thetaOrY, COORDINATE_IDENTIFIER identifier, InstantCommand setCommand){
//        return new ConditionalCommand(
//                new ConditionalCommand(
//                                new SequentialCommandGroup(
//                                                //new LogCommand("SET ARM COMMAND", "RUNNING SET COMMAND"),
//                                                setCommand
//                                                //new WaitUntilCommand(()-> arm.reachedTargetPosition())
//                                ).raceWith(
//                                        new SequentialCommandGroup(
//                                                        //new WaitUntilCommand(()-> arm.motionProfilePathsAtParametricEnd()),
//                                                        new WaitCommand(500),
//                                                        new InstantCommand(()-> System.out.println("LOGGER TEST"))
//
//
//                                                        //TODO THIS OMEGA STUPID LOG FREEZES THE ENTIRE COMMAND THREAD, FIGURE OUT WHY
//                                                        //new LogCommand("SET ARM COMMAND", "ARM DID NOT REACH TARGET POSITION IN TIME, INTERRUPTING THE WAIT COMMAND")
//                                        )
//                                ),
//
//                        new InstantCommand(),
//                        ()-> true
//                        //moveToTargetWhileAvoidingCamera(magnitudeOrX, thetaOrY, identifier),
//                        //()-> !arm.isCameraInTheWayToNewTarget(arm.coordinatesToTheta(magnitudeOrX, thetaOrY, identifier))
//                ),
//
//                new LogCommand("SET ARM POSITION COMMAND", Level.INFO, "TARGET POINT IS INVALID, SKIPPING COMMAND"),
//                ()-> arm.isTargetPointValid(arm.coordinatesToTheta(magnitudeOrX, thetaOrY, identifier))
//        );
//    }

    public ConditionalCommand magnitudeAndExtension(double magnitude, double angleDegrees){
        return safeSetTargetPoint(magnitude, angleDegrees, COORDINATE_IDENTIFIER.POLAR, new InstantCommand(()-> arm.setTargetPoint(magnitude, angleDegrees)));
    }

    public ConditionalCommand XY(double x_cm, double y_cm, OFFSET_REFERENCE_PLANE reference){
        return safeSetTargetPoint(x_cm, y_cm, COORDINATE_IDENTIFIER.CARTESIAN, new InstantCommand(()-> arm.setTargetPoint(arm.calculateTargetPointWithRealWordCoordinates(x_cm, y_cm, reference))));
    }

    private SequentialCommandGroup moveToTargetWhileAvoidingCamera(double magnitudeOrX, double thetaOrY, COORDINATE_IDENTIFIER coordinateType){
        return new SequentialCommandGroup(
                new LogCommand("SET ARM POSITION COMMAND", "CAN'T GO TO TARGET DIRECTLY, MAKING AN AVOIDANCE MANEUVER"),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new WaitCommand(100),
                new SetArmPosition().extension(0),
                new SetArmPosition().angleDegrees(arm.coordinatesToTheta(magnitudeOrX, thetaOrY, coordinateType)),
                new SetArmPosition().extension(arm.coordinatesToExtension(magnitudeOrX, thetaOrY, coordinateType))
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