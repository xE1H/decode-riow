package org.firstinspires.ftc.teamcode.helpers.utils.opmodes.ArmTests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRTestOpMode;
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


        if (gamepad1.triangle && !prevTriangle){
            CommandScheduler.getInstance().schedule(new SetArmPosition().intake(0.4, 0, 0.5));
        }

        else if (gamepad1.cross && !prevCross){
            //CommandScheduler.getInstance().schedule(new SetArmPosition().retract());
        }

        prevTriangle = gamepad1.triangle;
        prevCross = gamepad1.cross;


        telemetry.update();
    }
}