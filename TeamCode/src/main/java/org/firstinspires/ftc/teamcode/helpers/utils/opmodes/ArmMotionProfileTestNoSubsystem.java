package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ACCELERATION_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ACCELERATION_JERK;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.DECELERATION_JERK;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ENCODER_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ENCODER_TICKS_PER_ROTATION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_DERIVATIVE_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_INTEGRAL_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_PROPORTIONAL_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDFORWARD_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MAX_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.VELOCITY_GAIN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;


@Config
//@Disabled
@TeleOp(name="")
public class ArmMotionProfileTestNoSubsystem extends LinearOpMode
{
    ArmRotatorSubsystem rotator;
    public static double targetAngle = 0;
    private double prevTarget = 0;
    private DcMotorEx motor;
    private MotionProfile motionProfile;
    private DcMotorEx thoughBoreEncoder;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        thoughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motionProfile = new MotionProfile(
                FtcDashboard.getInstance().getTelemetry(),
                "ARM",
                MotionProfile.Type.ACCELERATION_LIMITED,
                ACCELERATION_JERK,
                DECELERATION_JERK,
                MAX_VELOCITY,
                0,
                FEEDBACK_PROPORTIONAL_GAIN,
                FEEDBACK_INTEGRAL_GAIN,
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);

        motionProfile.enableTelemetry(true);

        waitForStart();

        while (opModeIsActive()){
            if (targetAngle != prevTarget){
                motionProfile.setTargetPosition(targetAngle);
                prevTarget = targetAngle;
                motionProfile.updateCoefficients(
                        ACCELERATION_JERK,
                        DECELERATION_JERK,
                        MAX_VELOCITY,
                        FEEDBACK_PROPORTIONAL_GAIN,
                        FEEDBACK_INTEGRAL_GAIN,
                        FEEDBACK_DERIVATIVE_GAIN,
                        VELOCITY_GAIN,
                        ACCELERATION_GAIN
                );
            }

            double encoderPosition = -thoughBoreEncoder.getCurrentPosition();

            double currentAngle = encoderPosition / ENCODER_TICKS_PER_ROTATION * 360d;
            double feedForwardPower = Math.cos(Math.toRadians(currentAngle)) * FEEDFORWARD_GAIN;
            double power = motionProfile.getPower(currentAngle) + feedForwardPower;

            //motor.setPower(power);
            telemetry.update();
        }

    }
}
