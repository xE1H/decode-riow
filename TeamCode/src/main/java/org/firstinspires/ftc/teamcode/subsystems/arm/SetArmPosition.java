package org.firstinspires.ftc.teamcode.subsystems.arm;

import static org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem.getArm;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.COORDINATE_IDENTIFIER;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OFFSET_REFERENCE_PLANE;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;

import java.util.logging.Level;

public class SetArmPosition extends SequentialCommandGroup{

    public SetArmPosition(boolean interpolation){
        MainArmSubsystem arm = getArm();

        addRequirements(arm);
        arm.setInterpolation(interpolation);
    }

    public SetArmPosition() {this(false);}

    private ConditionalCommand safeSetTargetPoint(double magnitudeOrX, double thetaOrY, COORDINATE_IDENTIFIER identifier, InstantCommand setCommand){
        return new ConditionalCommand(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                setCommand,
                                new WaitUntilCommand(()-> getArm().reachedTargetPosition()).raceWith(
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> VLRSubsystem.getArm().motionProfilePathsAtParametricEnd()),
                                                new WaitCommand(500),
                                                new LogCommand("SET ARM POSITION COMMAND", Level.INFO, "WAIT UNTIL REACHED POSITION TIMEOUT TRIGGERED, OVERRIDING")
                                        )
                                )
                        ),
                        moveToTargetWhileAvoidingCamera(magnitudeOrX, thetaOrY, identifier),
                        ()-> getArm().isNewTargetSafeToMoveDirectly(getArm().coordinatesToTheta(magnitudeOrX, thetaOrY, identifier))
                ),

                new LogCommand("SET ARM POSITION COMMAND", Level.INFO, "TARGET POINT IS INVALID, SKIPPING COMMAND"),
                ()-> getArm().isTargetPointValid(getArm().coordinatesToTheta(magnitudeOrX, thetaOrY, identifier))
        );
    }

    private ConditionalCommand magnitudeAndExtension(double magnitude, double angleDegrees){
        return safeSetTargetPoint(magnitude, angleDegrees, COORDINATE_IDENTIFIER.POLAR, new InstantCommand(()-> getArm().setTargetPoint(magnitude, angleDegrees)));
    }

    private ConditionalCommand XY(double x_cm, double y_cm, OFFSET_REFERENCE_PLANE reference){
        return safeSetTargetPoint(x_cm, y_cm, COORDINATE_IDENTIFIER.CARTESIAN, new InstantCommand(()-> getArm().setTargetPointWithRealWordCoordinates(x_cm, y_cm, reference)));
    }

    private SequentialCommandGroup moveToTargetWhileAvoidingCamera(double magnitudeOrX, double thetaOrY, COORDINATE_IDENTIFIER coordinateType){
        return new SequentialCommandGroup(
                new LogCommand("SET ARM POSITION COMMAND", Level.INFO, "ARM IS ON A COLLISION TRAJECTORY WITH CAMERA, MAKING AN AVOIDANCE MANEUVER"),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new WaitCommand(100),
                new SetArmPosition().extension(0),
                new WaitUntilCommand(()-> getArm().reachedTargetPosition()),
                new SetArmPosition().angleDegrees(getArm().coordinatesToTheta(magnitudeOrX, thetaOrY, coordinateType)),
                new WaitUntilCommand(()-> getArm().reachedTargetPosition()),
                new SetArmPosition().extension(getArm().coordinatesToExtension(magnitudeOrX, thetaOrY, coordinateType))
        );
    }

    public ConditionalCommand extension(double extension){
        return magnitudeAndExtension(extension, getArm().getTargetAngleDegrees());
    }

    public ConditionalCommand angleDegrees(double angleDegrees){
        return magnitudeAndExtension(getArm().getTargetExtension(), angleDegrees);
    }

    public ConditionalCommand X(double x, OFFSET_REFERENCE_PLANE reference){
        return XY(x, getArm().getTargetY(), reference);
    }

    public ConditionalCommand Y(double y, OFFSET_REFERENCE_PLANE reference){
        return XY(getArm().getTargetX(), y, reference);
    }
}