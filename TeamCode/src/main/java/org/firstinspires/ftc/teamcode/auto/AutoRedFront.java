package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by HSstudent on 12/5/2017.
 */

@Autonomous(name="Auto: Front Red", group="auto")
public class AutoRedFront extends AutoBase {

    @Override
    public void runOpMode() {

        initAuto();
        initVuforia();
        waitForStart();
        runAuto(true,true, true, true, true);
    }
}
