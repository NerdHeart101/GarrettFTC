package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by HSstudent on 12/5/2017.
 */

@Autonomous(name="Auto: Front Blue", group="auto")
public class AutoBlueFront extends AutoBase {

    @Override
    public void runOpMode() {

        initAuto();
        waitForStart();
        runAuto(false,true);
    }
}
