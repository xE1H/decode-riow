package org.firstinspires.ftc.teamcode.helpers.testOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wiper.Wiper;

@Config
@Photon
@TeleOp()
public class WiperTest extends VLRLinearOpMode {
    @Override
    public void run(){
        VLRSubsystem.requireSubsystems(Wiper.class);
        VLRSubsystem.initializeAll(hardwareMap);


        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.triangle){
                VLRSubsystem.getInstance(Wiper.class).wipe(0);
            }
            else if (gamepad1.cross){
                VLRSubsystem.getInstance(Wiper.class).wipe(1);
            }
        }
    }

}