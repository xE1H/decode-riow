package org.firstinspires.ftc.teamcode.helpers.utils.opmodes.ArmTests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRTestOpMode;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;

@Config
@Photon
@TeleOp()
public class ArmCommandMagnitudeAndExtensionTest extends VLRTestOpMode {
    public static double extension = 0;
    public static double angle = 0;

    private double prevExtension = 0;
    private double prevAngle = 0;

    boolean prevTriangle = false;
    boolean prevCross = false;


    @Override
    public void Start(){
        //CommandScheduler.getInstance().schedule(new SetArmPosition().X(20, MainArmConfiguration.OFFSET_REFERENCE_PLANE.FRONT));
    }

    @Override
    public void Loop() {
//        if (extension != prevExtension){
//            prevExtension = extension;
//            CommandScheduler.getInstance().schedule(new SetArmPosition().extension(extension));
//        }
//
//        if (angle != prevAngle){
//            prevAngle = angle;
//            CommandScheduler.getInstance().schedule(new SetArmPosition().angleDegrees(angle));
//        }


//        if (gamepad1.triangle && !prevTriangle){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().scoreSample(MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET));
//        }
//
//        else if (gamepad1.cross && !prevCross){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().retract());
//        }

        if (gamepad1.triangle && !prevTriangle){
            CommandScheduler.getInstance().schedule(new SetArmPosition().XY(40, 30, MainArmConfiguration.OFFSET_REFERENCE_PLANE.FRONT));
        }

        else if (gamepad1.cross && !prevCross){
            CommandScheduler.getInstance().schedule(new SetArmPosition().XY(10, 0, MainArmConfiguration.OFFSET_REFERENCE_PLANE.FRONT));
        }


        prevTriangle = gamepad1.triangle;
        prevCross = gamepad1.cross;

        telemetry.update();
    }
}