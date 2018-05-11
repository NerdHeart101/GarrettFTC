package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.core.HardwareCompbot;

/**
 * Created by HSstudent on 12/5/2017.
 */

@Autonomous(name="Auto: ALL", group="auto")
public class AutoAll extends LinearOpMode {

    HardwareCompbot robot = new HardwareCompbot();
    private ElapsedTime runtime = new ElapsedTime();

    // Encoder variables
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

    VuforiaLocalizer vuforia;

    // Auto control variables
    // autoPrefs is a placeholder for a GUI to select each variable
    // color: true means red, false means blue
    // position: true means front, false means back
    private boolean[] autoPrefs = {true,false};

    private boolean color = autoPrefs[0];
    private boolean position = autoPrefs[1];

    @Override
    public void runOpMode() {

        // Arm positions
        final double up = 0.0;
        final double down = 0.7;

        // Initialization procedures
        robot.init(hardwareMap);

        robot.colorSensor.enableLed(true);

        robot.bottomLeftClaw.setPosition(1);
        robot.bottomRightClaw.setPosition(0);

        robot.jewelArm.setPosition(up);
        robot.jewelShoulder.setPosition(0.5);

        robot.gyroSensor.calibrate();

        waitForStart();

        // BEGIN AUTONOMOUS

        // Grab glyph, get arm in sensing position
        robot.topLeftClaw.setPosition(1);
        robot.topRightClaw.setPosition(0);
        glyphGrab(true);
        glyph(0.5,1);

        robot.jewelArm.setPosition(down);

        sleep(2000);

        // Sense color and knock jewel
        // Note: higher shoulder values mean farther forward
        while(opModeIsActive()) {
            telemetry.addData("Status", "Sensing");
            telemetry.update();
            if (robot.colorSensor.red() != robot.colorSensor.blue()) {
                if ((robot.colorSensor.blue() > robot.colorSensor.red()) == color) {
                    robot.jewelShoulder.setPosition(0.7);
                } else {
                    robot.jewelShoulder.setPosition(0.3);
                }
                sleep(2000);
                break;
            } else {
                robot.jewelShoulder.setPosition(robot.jewelShoulder.getPosition() + .001);
                // Failsafe
                if(robot.jewelShoulder.getPosition() > 0.7) {
                    break;
                }
            }
        }

        // Reset arm
        robot.jewelArm.setPosition(up);
        robot.jewelShoulder.setPosition(0.5);

        // Drive to safe zone an score glyph

        if (position) {
            // Red
            if(color) {
                encoderDrive(28.0, 0);
                gyroRotate(-45, TURN_SPEED);
                encoderDrive(10,0);
                glyph(-0.5,1);
                glyphGrab(false);
            }
            // Blue
            else {
                encoderDrive(26.0, 180);
                gyroRotate(135, TURN_SPEED);
                encoderDrive(10,0);
                glyph(-0.5,1);
                glyphGrab(false);
            }
        }
        // Back stones
        else {
            // Red
            if(color) {
                encoderDrive(28.0,0);
                gyroRotate(45,TURN_SPEED);
                encoderDrive(4,0);
                glyph(-0.5,1);
                glyphGrab(false);
                //encoderDrive(2,0);
            }
            // Blue
            else {
                encoderDrive(26.0,180);
                gyroRotate(225,TURN_SPEED);
                encoderDrive(4,0);
                glyph(-0.5,1);
                glyphGrab(false);
                //encoderDrive(2,0);
            }
        }

        telemetry.addData("Status", "Complete");
        telemetry.update();

        while(opModeIsActive()){}
    }

    public void glyph(double power, double time) {
        double endTime = runtime.seconds() + time;
        robot.glyphLift.setPower(power);
        while(opModeIsActive() && runtime.seconds() < endTime) {}
        robot.glyphLift.setPower(0.0);
    }

    // Method by FIRST to drive a certain number of inches
    public void encoderDrive(double inches, double angle) {
        encoderDrive(inches, angle, DRIVE_SPEED, inches / DRIVE_SPEED);
    }

    public void encoderDrive(double inches, double angle, double speed) {
        encoderDrive(inches, angle, speed, inches / DRIVE_SPEED);
    }

    public void encoderDrive(double inches, double angle, double speed, double timeout) {
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

            while ( robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
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
    public void gyroRotate(int degrees, double power) {
        double rotatePower;

        // Get target heading
        int targetHeading = (robot.gyroSensor.getHeading() + degrees + 360) % 360;

        // Set power to the appropriate sign depending on direction
        if(degrees > 0) {
            rotatePower = power;
        } else {
            rotatePower = -power;
        }

        // Apply power
        robot.frontRight.setPower(rotatePower);
        robot.backRight.setPower(rotatePower);
        robot.frontLeft.setPower(-rotatePower);
        robot.backLeft.setPower(-rotatePower);

        while(opModeIsActive()) {

            telemetry.addData("target heading",targetHeading);
            telemetry.addData("current heading",robot.gyroSensor.getHeading());
            telemetry.addData("heading check",Math.abs(targetHeading-robot.gyroSensor.getHeading()) % 360);
            telemetry.update();

            // When we are within 15 degrees of the target, move slower
            if(Math.abs(targetHeading - robot.gyroSensor.getHeading()) % 360 <= 15) {
                robot.frontRight.setPower(rotatePower / power * FINE_TURN);
                robot.backRight.setPower(rotatePower / power * FINE_TURN);
                robot.frontLeft.setPower(-rotatePower / power * FINE_TURN);
                robot.backLeft.setPower(-rotatePower / power * FINE_TURN);
            }

            // If we are within 3 degrees of the target, end the rotation
            if(Math.abs(targetHeading - robot.gyroSensor.getHeading()) % 360 <= 3) {
                break;
            }
        }

        // Turn motors off
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);

    }

    public void glyphGrab(boolean grabbing) {
        if(grabbing) {
            robot.bottomLeftClaw.setPosition(0);
            robot.bottomRightClaw.setPosition(1);
        } else {
            robot.bottomLeftClaw.setPosition(0.5);
            robot.bottomRightClaw.setPosition(0.5);
        }
        sleep(500);
    }

    // DEPRECATED: For tank drive
    // Positive degrees is left (Counter clockwise)
    public void rotateInPlace(double degrees) {
        degrees += 5;   // 5 degrees are added to account for errors in the encoders

        double arc = TURN_RADIUS * Math.toRadians(degrees);
        double leftArc = -arc;
        double rightArc = arc;
        double rotateTimeout = Math.abs(arc)*TURN_SPEED*2;

        // Use the encoderDrive method to perform the move
        encoderDrive(TURN_SPEED, leftArc, rightArc, rotateTimeout);
    }

    // Get the bonus position for the glyph. If nothing is found, use the position default
    public int glyphBonus() {
        // No camera monitor, for competition
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        // With camera monitor, for debugging
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Enter our license key
        parameters.vuforiaLicenseKey = "AShjAuD/////AAAAGQ1/wLnLiEA0ioTqRWYn+SxShC+UUo94K2KMWDmywIJ7j7mBSh8V5XGWJN/9oBiD/pAzdAj3NSoJ2IJ1Nu0ZKSf7NKxeFlWFYrexIs25lYjryT/ag7+RQYT158sa1H0Fe9+Y//H+qZvO63odc6QhBadD3yEmkYfqbANDud8IcesvB/FdCnKdEpaAdyzDJBBmPGW3MFTn18Zb3Vm+44MVSTnk9a32HE2D4dViN477aIGh/jacPTW+xdlpSQSfwXb1+i8rFPF7chm1XY8LGUvtiDaSsS9LuuiOrJ7OsINLmm5xAGxaqHvf1LbF+aUD1iKrLEWG4EMlyIpPC8mZCsw6cp7LwQJLgWsvsIqRcLps2gEu";

        // Use the front camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        // Load the VuMark targets for Relic Recovery
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        // Start searching for a trackable
        relicTrackables.activate();

        //while(vuMark);
        return 0;
    }
}
