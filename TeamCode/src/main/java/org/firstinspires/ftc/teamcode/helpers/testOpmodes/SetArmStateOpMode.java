package org.firstinspires.ftc.teamcode.helpers.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;

@TeleOp(group = "Utils")
public class SetArmStateOpMode extends VLRLinearOpMode {
    @Override
    public void run(){
        ArmState.set(ArmState.State.SPECIMEN_SCORE_BACK);

        waitForStart();
        if (opModeIsActive()) {
            ArmState.set(ArmState.State.IN_ROBOT);
        }
    }
}
