package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionState;


@Config
@Disabled
@TeleOp()
public class MotionProfileGenerationTest extends OpMode
{
    private MotionProfile motionProfile;
    public static double targetPosition = 0;
    private double prevTarget = 0;
    private double startTime = System.nanoTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motionProfile = new MotionProfile(telemetry, "test", MotionProfile.Type.JERK_LIMITED, 2, 1, 8, 0, 0, 0, 0, 0, 0);
    }


    @Override
    public void loop() {
        if (targetPosition != prevTarget){
            startTime = System.nanoTime();
            prevTarget = targetPosition;
        }
        MotionState motionState = motionProfile.computeJerkMotionState(targetPosition, (System.nanoTime() - startTime) / Math.pow(10, 9));
        telemetry.addData("position", motionState.position);
        telemetry.addData("velocity", motionState.velocity);
        telemetry.addData("acceleration", motionState.acceleration);
        telemetry.addData("jerk", motionState.jerk);
    }
}
