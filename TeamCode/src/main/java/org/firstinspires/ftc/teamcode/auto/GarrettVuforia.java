package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.core.HardwareCompbot;

/**
 * Created by Lane Doak of Recharged Orange on 9/9/17.
 */
@Autonomous(name = "VuforiaRelic", group = "Vuforia")
public class GarrettVuforia extends LinearOpMode
{
    OpenGLMatrix lastLocation = null; // WARNING: VERY INACCURATE, USE ONLY TO ADJUST TO FIND IMAGE AGAIN! DO NOT BASE MAJOR MOVEMENTS OFF OF THIS!!
    double tX; // X value extracted from our the offset of the traget relative to the robot.
    double tZ; // Same as above but for Z
    double tY; // Same as above but for Y
    // -----------------------------------
    double rX; // X value extracted from the rotational components of the tartget relitive to the robot
    double rY; // Same as above but for Y
    double rZ; // Same as above but for Z
    HardwareCompbot hardwareCompbot = new HardwareCompbot();

    VuforiaLocalizer vuforia;

    public void runOpMode()
    {

        hardwareCompbot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdkcvTz/////AAABmV0utTCVrEudjNYKd+gZfKxLpQIVP2jHrTjRv2yUhamciSF1n6gvMQakOa9KnuwMshgg43kBlpL1rkR2DjYMQcd2VrXcnJw8wXWy8KqYpWIrgUrKb34Fn5hGm49PKHrJzt3UMKGgBfwDED+myhCyKuBR0r9x4ywQDUgrm6Q8VhVpfDE9aUz+OYzgwHvzU71G//z13/jYOWFovWIO4l6FRxh0XW2jTWyRosnSX9njzDeBR7UInRnhOkoScTxxZ/vvDu/aAgCGusbNzVtuR+RWddp0D7EOyWxta9ovv1r2TLKIKx5Ga+3XO9M2LbvA7MfKtoTSb3eRucMCbGLt+CE0jbVk9LuY4pjr4MXPo0fsdJ9N";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; // Use FRONT Camera (Change to BACK if you want to use that one)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // Display Axes

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        waitForStart();

        relicTrackables.activate(); // Activate Vuforia

        while (opModeIsActive())
        {
            hardwareCompbot.frontLeft.setPower(.1);
            hardwareCompbot.frontRight.setPower(.1);
            hardwareCompbot.backLeft.setPower(.1);
            hardwareCompbot.backRight.setPower(.1);

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // Test to see if image is visable

                hardwareCompbot.frontLeft.setPower(0);
                hardwareCompbot.frontRight.setPower(0);
                hardwareCompbot.backLeft.setPower(0);
                hardwareCompbot.backRight.setPower(0);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); // Get Positional value to use later
                telemetry.addData("Pose", format(pose));
                if (pose != null)
                {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    tX = trans.get(0);
                    tY = trans.get(1);
                    tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot. NOTE: VERY IMPORTANT IF BASING MOVEMENT OFF OF THE IMAGE!!!!
                    rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    rZ = rot.thirdAngle;
                }
                if (vuMark == RelicRecoveryVuMark.LEFT)
                { // Test to see if Image is the "LEFT" image and display value.
                    telemetry.addData("VuMark is", "Left");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                } else if (vuMark == RelicRecoveryVuMark.RIGHT)
                { // Test to see if Image is the "RIGHT" image and display values.
                    telemetry.addData("VuMark is", "Right");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                } else if (vuMark == RelicRecoveryVuMark.CENTER)
                { // Test to see if Image is the "CENTER" image and display values.
                    telemetry.addData("VuMark is", "Center");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);

                }
            } else
            {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }
    }
    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}

