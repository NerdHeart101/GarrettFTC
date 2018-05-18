package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.core.HardwareCompbot;

/**
 * Created by HSstudent on 12/5/2017.
 */

@Disabled
@Autonomous(name="Auto: BASE", group="auto")
public class AutoBase extends LinearOpMode {

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
    static final double     CENTER_OFFSET           = 2   ;     // The offset of the center of the bot from the center of the balancing stone

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    public static final String TAG = "Vuforia VuMark Sample";

    double tX; // X value extracted from our the offset of the target relative to the robot.
    double tZ; // Same as above but for Z
    double tY; // Same as above but for Y
    // -----------------------------------
    double rX; // X value extracted from the rotational components of the target relative to the robot
    double rY; // Same as above but for Y
    double rZ; // Same as above but for Z

    OpenGLMatrix lastLocation = null;

    public void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdkcvTz/////AAABmV0utTCVrEudjNYKd+gZfKxLpQIVP2jHrTjRv2yUhamciSF1n6gvMQakOa9KnuwMshgg43kBlpL1rkR2DjYMQcd2VrXcnJw8wXWy8KqYpWIrgUrKb34Fn5hGm49PKHrJzt3UMKGgBfwDED+myhCyKuBR0r9x4ywQDUgrm6Q8VhVpfDE9aUz+OYzgwHvzU71G//z13/jYOWFovWIO4l6FRxh0XW2jTWyRosnSX9njzDeBR7UInRnhOkoScTxxZ/vvDu/aAgCGusbNzVtuR+RWddp0D7EOyWxta9ovv1r2TLKIKx5Ga+3XO9M2LbvA7MfKtoTSb3eRucMCbGLt+CE0jbVk9LuY4pjr4MXPo0fsdJ9N";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    // Auto control variables
    // autoPrefs is a placeholder for a GUI to select each variable
    // color: true means red, false means blue
    // position: true means front, false means back

    // Initialization procedures
    public void initAuto() {

        robot.init(hardwareMap);

        robot.colorSensor.enableLed(true);

        robot.bottomLeftClaw.setPosition(1);
        robot.bottomRightClaw.setPosition(0);

        robot.jewelArm.setPosition(0.0);
        robot.jewelShoulder.setPosition(0.5);

        robot.gyroSensor.calibrate();
    }

    public void runAuto(boolean red, boolean front, boolean jewel, boolean glyph, boolean vuforia) {
        // Grab glyph, get arm in sensing position
        robot.topLeftClaw.setPosition(1);
        robot.topRightClaw.setPosition(0);
        glyphGrab(true);
        glyph(0.5,1);

        if(jewel) {
            robot.jewelArm.setPosition(0.725);

            sleep(2000);

            // Sense color and knock jewel
            // Note: higher shoulder values mean farther forward
            while (opModeIsActive()) {
                telemetry.addData("Status", "Sensing");
                telemetry.update();
                if (robot.colorSensor.red() != robot.colorSensor.blue()) {
                    if ((robot.colorSensor.blue() > robot.colorSensor.red()) == red) {
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
            }

            // Reset arm
            robot.jewelArm.setPosition(0.0);
            robot.jewelShoulder.setPosition(0.5);
        }

        // Drive to safe zone and score glyph
        if(glyph) {
            // Front stones
            if (front) {
                // Red
                if (red) {
                    if(vuforia) {
                        // Get the bonus target while doing the first move
                        RelicRecoveryVuMark bonusTarget = vuforiaDrive(36 + CENTER_OFFSET, 0);
                        // Rotate the appropriate amount to get to the correct column
                        if(bonusTarget == RelicRecoveryVuMark.LEFT) {
                            gyroRotate(-60, TURN_SPEED);
                        } else if (bonusTarget == RelicRecoveryVuMark.RIGHT) {
                            gyroRotate(-120, TURN_SPEED);
                        } else {
                            gyroRotate(-90, TURN_SPEED);
                        }
                        // Move into position
                        encoderDrive(6.0, 0);
                    } else {
                        // Move into position
                        encoderDrive(36 + CENTER_OFFSET, 0);
                        gyroRotate(-90, TURN_SPEED);
                        encoderDrive(6, 0);
                    }
                    // Place the cube
                    glyph(-0.5, 1);
                    glyphGrab(false);
                    encoderDrive(1, 180);
                }
                // Blue
                else {
                    if(vuforia) {
                        // Get the bonus target while doing the first move
                        RelicRecoveryVuMark bonusTarget = vuforiaDrive(36 + CENTER_OFFSET, 0);
                        // Rotate the appropriate amount to get to the correct column
                        if(bonusTarget == RelicRecoveryVuMark.LEFT) {
                            gyroRotate(-60, TURN_SPEED);
                        } else if (bonusTarget == RelicRecoveryVuMark.RIGHT) {
                            gyroRotate(-120, TURN_SPEED);
                        } else {
                            gyroRotate(-90, TURN_SPEED);
                        }
                        // Move into position
                        encoderDrive(6.0, 0);
                    } else {
                        encoderDrive(36 - CENTER_OFFSET, 180);
                        gyroRotate(90, TURN_SPEED);
                        encoderDrive(6, 0);
                    }
                    glyph(-0.5, 1);
                    glyphGrab(false);
                    encoderDrive(1, 180);
                }
            }
            // Back stones
            else {
                // Red
                if (red) {
                    if(vuforia) {
                        encoderDrive(24 + CENTER_OFFSET, 0);
                        gyroRotate(45, TURN_SPEED);
                        encoderDrive(5, 0);
                    } else {
                        encoderDrive(24 + CENTER_OFFSET, 0);
                        gyroRotate(45, TURN_SPEED);
                        encoderDrive(5, 0);
                    }
                    glyph(-0.5, 1);
                    glyphGrab(false);
                    encoderDrive(1, 180);
                }
                // Blue
                else {
                    if(vuforia) {
                        encoderDrive(24 + CENTER_OFFSET, 0);
                        gyroRotate(45, TURN_SPEED);
                        encoderDrive(5, 0);
                    } else {
                        encoderDrive(24 - CENTER_OFFSET, 180);
                        gyroRotate(135, TURN_SPEED);
                        encoderDrive(8, 0);
                    }
                    glyph(-0.5, 1);
                    glyphGrab(false);
                    encoderDrive(1, 180);
                }
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

                telemetry.addData("Path", "fl %d :: br %d :: fr %d :: bl %d", fl, br, fr, bl);
                telemetry.addData("Position", "fl %d :: br %d :: fr %d :: bl %d",
                        robot.frontLeft.getCurrentPosition(), robot.backRight.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition());
                telemetry.addData("Timeout", "%.2f : %.2f", timeout, runtime.seconds());
                telemetry.update();
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

    // These methods are garbage but at least they work probably
    public RelicRecoveryVuMark vuforiaDrive(double inches, double angle) {
        return vuforiaDrive(inches, angle, DRIVE_SPEED, inches / DRIVE_SPEED);
    }

    public RelicRecoveryVuMark vuforiaDrive(double inches, double angle, double speed) {
        return vuforiaDrive(inches, angle, speed, inches / DRIVE_SPEED);
    }

    public RelicRecoveryVuMark vuforiaDrive(double inches, double angle, double speed, double timeout) {

        // Return variable
        RelicRecoveryVuMark vuMarkReturn = RelicRecoveryVuMark.UNKNOWN;

        // Encoder target variables
        int fl, fr, bl, br;

        if (opModeIsActive()) {

            // Motor positions determined by trigonometry
            angle += 45;
            fl = robot.frontLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * Math.cos(Math.toRadians(angle)) * inches);
            fr = robot.frontRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * Math.sin(Math.toRadians(angle)) * inches);
            bl = robot.backLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * Math.sin(Math.toRadians(angle)) * inches);
            br = robot.backRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * Math.cos(Math.toRadians(angle)) * inches);

            // Path telemetry
            telemetry.addData("Path", "fl %d :: br %d :: fr %d :: bl %d", fl, br, fr, bl);
            telemetry.update();

            // Apply positions to motors
            robot.frontLeft.setTargetPosition(fl);
            robot.frontRight.setTargetPosition(fr);
            robot.backLeft.setTargetPosition(bl);
            robot.backRight.setTargetPosition(br);

            // Set motor mode
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply motor powers proportional to how far they have to move
            runtime.reset();
            robot.frontLeft.setPower(speed * Math.abs(Math.cos(Math.toRadians(angle))));
            robot.frontRight.setPower(speed * Math.abs(Math.sin(Math.toRadians(angle))));
            robot.backLeft.setPower(speed * Math.abs(Math.sin(Math.toRadians(angle))));
            robot.backRight.setPower(speed * Math.abs(Math.cos(Math.toRadians(angle))));

            // Activate Vuforia targets
            relicTrackables.activate();

            while ( robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
                    robot.backLeft.isBusy() && robot.backRight.isBusy() &&
                    runtime.seconds() < timeout && opModeIsActive()) {

                // Motor telemetry
                telemetry.addData("Path", "fl %d :: br %d :: fr %d :: bl %d", fl, br, fr, bl);
                telemetry.addData("Position", "fl %d :: br %d :: fr %d :: bl %d",
                        robot.frontLeft.getCurrentPosition(), robot.backRight.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition());
                telemetry.addData("Timeout", "%.2f : %.2f", timeout, runtime.seconds());

                // Vuforia nonsense
                if (vuMarkReturn == RelicRecoveryVuMark.UNKNOWN) {
                    // Only look for vuforia if we do not already have a return value
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    vuMarkReturn = vuMark;
                }

                telemetry.addData("VuMark", vuMarkReturn);
                telemetry.update();

            }

            // End motor movement
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            // Get motors ready for TeleOp once again
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Return VuMark value
        return vuMarkReturn;
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

    String format (OpenGLMatrix transformationMatrix){
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    @Override
    public void runOpMode() {

        telemetry.addData("ERROR","what did you do this should be disabled");
        telemetry.update();
        while(opModeIsActive()) {}
    }
}
