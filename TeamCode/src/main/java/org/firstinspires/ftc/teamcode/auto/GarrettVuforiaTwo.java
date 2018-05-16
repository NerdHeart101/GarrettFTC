/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.core.HardwareCompbot;


@Autonomous(name=" VuMark Two", group ="Concept")

public class GarrettVuforiaTwo extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    static final double     TURN_RADIUS             = 8.24;     // In inches
    static final double     COUNTS_PER_MOTOR_REV    = 1556;     // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5 ;
    // Note: TURN_SPEED is used for the first part of the turn, FINE_TURN for the last part of it
    static final double     TURN_SPEED              = 0.25;
    static final double     FINE_TURN               = 0.1 ;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    HardwareCompbot robot = new HardwareCompbot();
    private ElapsedTime runtime = new ElapsedTime();
    private boolean color;

    double tX; // X value extracted from our the offset of the traget relative to the robot.
    double tZ; // Same as above but for Z
    double tY; // Same as above but for Y
    // -----------------------------------
    double rX; // X value extracted from the rotational components of the tartget relitive to the robot
    double rY; // Same as above but for Y
    double rZ; // Same as above but for Z

    @Override public void runOpMode() {

        final double up = 0.0;
        final double down = 0.7;

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        robot.init(hardwareMap);

        robot.colorSensor.enableLed(true);

        robot.bottomLeftClaw.setPosition(1);
        robot.bottomRightClaw.setPosition(0);

        robot.jewelArm.setPosition(up);
        robot.jewelShoulder.setPosition(0.5);

        robot.gyroSensor.calibrate();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AdkcvTz/////AAABmV0utTCVrEudjNYKd+gZfKxLpQIVP2jHrTjRv2yUhamciSF1n6gvMQakOa9KnuwMshgg43kBlpL1rkR2DjYMQcd2VrXcnJw8wXWy8KqYpWIrgUrKb34Fn5hGm49PKHrJzt3UMKGgBfwDED+myhCyKuBR0r9x4ywQDUgrm6Q8VhVpfDE9aUz+OYzgwHvzU71G//z13/jYOWFovWIO4l6FRxh0XW2jTWyRosnSX9njzDeBR7UInRnhOkoScTxxZ/vvDu/aAgCGusbNzVtuR+RWddp0D7EOyWxta9ovv1r2TLKIKx5Ga+3XO9M2LbvA7MfKtoTSb3eRucMCbGLt+CE0jbVk9LuY4pjr4MXPo0fsdJ9N";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        runtime.reset();
        // BEGIN AUTONOMOUS

        // Grab glyph, get arm in sensing position
        robot.topLeftClaw.setPosition(1);
        robot.topRightClaw.setPosition(0);
        glyphGrab(true);
        glyph(0.5, 1);

        robot.jewelArm.setPosition(down);

        sleep(2000);

        relicTrackables.activate();


        while (opModeIsActive() && (runtime.seconds() < 4.0)) {

            if (robot.colorSensor.red() != robot.colorSensor.blue()) {
                if ((robot.colorSensor.blue() > robot.colorSensor.red())) {
                    robot.jewelShoulder.setPosition(0.7);
                } else {
                    robot.jewelShoulder.setPosition(0.3);
                }
                sleep(2000);
                break;
            } else {
                robot.jewelShoulder.setPosition(robot.jewelShoulder.getPosition() + .001);
                // Failsafe
                if (robot.jewelShoulder.getPosition() > 0.7) {
                    break;
                }
            }
            robot.jewelShoulder.setPosition(0.0);
            robot.jewelArm.setPosition(up);

        }


        runtime.reset();


        while (opModeIsActive() && (runtime.seconds() < 12.0)) {
            robot.frontLeft.setPower(.07);
            robot.frontRight.setPower(.07);
            robot.backLeft.setPower(.07);
            robot.backRight.setPower(.07);


            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {


                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (vuMark == RelicRecoveryVuMark.LEFT) { // Test to see if Image is the "LEFT" image and display value.
                    telemetry.addData("VuMark is", "Left");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);

                } else if (vuMark == RelicRecoveryVuMark.RIGHT) { // Test to see if Image is the "RIGHT" image and display values.
                    telemetry.addData("VuMark is", "Right");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);


                } else if (vuMark == RelicRecoveryVuMark.CENTER) { // Test to see if Image is the "CENTER" image and display values.
                    telemetry.addData("VuMark is", "Center");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);

                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();



        }
        runtime.reset();


    }

        String format (OpenGLMatrix transformationMatrix){
            return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
        }

        public void glyph ( double power, double time){
            double endTime = runtime.seconds() + time;
            robot.glyphLift.setPower(power);
            while (opModeIsActive() && runtime.seconds() < endTime) {
            }
            robot.glyphLift.setPower(0.0);
        }

        // Method by FIRST to drive a certain number of inches
        public void encoderDrive ( double inches, double angle){
            encoderDrive(inches, angle, DRIVE_SPEED, inches / DRIVE_SPEED);
        }

        public void encoderDrive ( double inches, double angle, double speed){
            encoderDrive(inches, angle, speed, inches / DRIVE_SPEED);
        }

        public void encoderDrive ( double inches, double angle, double speed, double timeout){
            int fl, fr, bl, br;
            if (opModeIsActive()) {

                angle += 45;
                fl = robot.frontLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * Math.cos(Math.toRadians(angle)) * inches);
                fr = robot.frontRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * Math.sin(Math.toRadians(angle)) * inches);
                bl = robot.backLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * Math.sin(Math.toRadians(angle)) * inches);
                br = robot.backRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * Math.cos(Math.toRadians(angle)) * inches);

                telemetry.addData("Path", "fl %d :: br %d :: fr %d :: bl %d", fl, br, fr, bl);
                telemetry.update();

                robot.frontLeft.setTargetPosition(fl);
                robot.frontRight.setTargetPosition(fr);
                robot.backLeft.setTargetPosition(bl);
                robot.backRight.setTargetPosition(br);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                runtime.reset();
                robot.frontLeft.setPower(speed * Math.abs(Math.cos(Math.toRadians(angle))));
                robot.frontRight.setPower(speed * Math.abs(Math.sin(Math.toRadians(angle))));
                robot.backLeft.setPower(speed * Math.abs(Math.sin(Math.toRadians(angle))));
                robot.backRight.setPower(speed * Math.abs(Math.cos(Math.toRadians(angle))));

                while (robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
                        robot.backLeft.isBusy() && robot.backRight.isBusy() &&
                        runtime.seconds() < timeout && opModeIsActive()) {
                /* int frontEnc = Math.abs(robot.frontLeft.getCurrentPosition());
                int backEnc = Math.abs(robot.backLeft.getCurrentPosition());
                int compareEnc = Math.max(frontEnc, backEnc); */

                    telemetry.addData("Path", "fl %d :: br %d :: fr %d :: bl %d", fl, br, fr, bl);
                    telemetry.addData("Position", "fl %d :: br %d :: fr %d :: bl %d",
                            robot.frontLeft.getCurrentPosition(), robot.backRight.getCurrentPosition(),
                            robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition());
                    telemetry.addData("Timeout", "%.2f : %.2f", timeout, runtime.seconds());
                    telemetry.update();
                /* if (compareEnc > Math.max(Math.abs(fl), Math.abs(bl))) {
                    break;
                } */
                }

                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        // Positive degrees is left
        public void gyroRotate ( int degrees, double power){
            double rotatePower;

            // Get target heading
            int targetHeading = (robot.gyroSensor.getHeading() + degrees + 360) % 360;

            // Set power to the appropriate sign depending on direction
            if (degrees > 0) {
                rotatePower = power;
            } else {
                rotatePower = -power;
            }

            // Apply power
            robot.frontRight.setPower(rotatePower);
            robot.backRight.setPower(rotatePower);
            robot.frontLeft.setPower(-rotatePower);
            robot.backLeft.setPower(-rotatePower);

            while (opModeIsActive()) {

                telemetry.addData("target heading", targetHeading);
                telemetry.addData("current heading", robot.gyroSensor.getHeading());
                telemetry.addData("heading check", Math.abs(targetHeading - robot.gyroSensor.getHeading()) % 360);
                telemetry.update();

                // When we are within 15 degrees of the target, move slower
                if (Math.abs(targetHeading - robot.gyroSensor.getHeading()) % 360 <= 15) {
                    robot.frontRight.setPower(rotatePower / power * FINE_TURN);
                    robot.backRight.setPower(rotatePower / power * FINE_TURN);
                    robot.frontLeft.setPower(-rotatePower / power * FINE_TURN);
                    robot.backLeft.setPower(-rotatePower / power * FINE_TURN);
                }

                // If we are within 3 degrees of the target, end the rotation
                if (Math.abs(targetHeading - robot.gyroSensor.getHeading()) % 360 <= 3) {
                    break;
                }
            }

            // Turn motors off
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);

        }

        public void glyphGrab ( boolean grabbing){
            if (grabbing) {
                robot.bottomLeftClaw.setPosition(0);
                robot.bottomRightClaw.setPosition(1);
            } else {
                robot.bottomLeftClaw.setPosition(0.5);
                robot.bottomRightClaw.setPosition(0.5);
            }
            sleep(500);
        }

    }

