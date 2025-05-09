package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRTestOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.BlinkinSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wiper.Wiper;

import pedroPathing.tuners.constants.FConstants;
import pedroPathing.tuners.constants.LConstants;

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