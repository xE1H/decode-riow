package org.firstinspires.ftc.teamcode.subsystems.claw;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem.mapToRange;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;


public class ClawSubsystem extends VLRSubsystem<ClawSubsystem> implements ClawConfiguration {
    private Servo angleServo, twistServo, grabServos;

    private VerticalRotation targetAngle = VerticalRotation.UP;
    private GripperState clawState = GripperState.CLOSED;

    public GripperState getClawState() {
        return clawState;
    }

    protected void initialize(HardwareMap hardwareMap) {
        angleServo = hardwareMap.get(Servo.class, ANGLE_SERVO);
        twistServo = hardwareMap.get(Servo.class, TWIST_SERVO);
        grabServos = hardwareMap.get(Servo.class, GRAB_SERVO);

        setTargetAngle(VerticalRotation.UP);
        setHorizontalRotation(HorizontalRotation.NORMAL);
        setTargetState(GripperState.CLOSED);
    }


    public void setTargetAngle(VerticalRotation rotation) {
        this.targetAngle = rotation;
        angleServo.setPosition(rotation.pos);
    }

    public void setTargetAngle(double pos) {
        angleServo.setPosition(pos);
    }

    public VerticalRotation getTargetAngle() {
        return targetAngle;
    }

    public void setHorizontalRotation(HorizontalRotation rotation) {
        twistServo.setPosition(rotation.pos);
    }

    public void setHorizontalRotation(double rotationPos) {
        twistServo.setPosition(clamp((1 + rotationPos) * 0.5, HORIZONTAL_ROTATION_MIN, HORIZONTAL_ROTATION_MAX));
    }

    public void setTargetState(GripperState state) {
        clawState = state;
        grabServos.setPosition(state.pos);
    }
}