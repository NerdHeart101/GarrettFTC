package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Vuforia Red Front", group = "auto")
public class AutoVuforiaRedFront extends AutoBase {

    @Override
    public void runOpMode() {
        initAuto();
        initVuforia();
        waitForStart();
    }
}
