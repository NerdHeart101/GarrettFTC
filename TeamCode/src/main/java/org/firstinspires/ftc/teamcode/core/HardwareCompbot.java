package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class is used to define our hardware object definitions and registers them into our robot configuration profiles
 * and names them accordingly. It also sets default values we want for motors, servos, sensors, etc.
 */

public class HardwareCompbot {

    // Define all the hardware objects we need
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor relicShoulder = null;
    public DcMotor relicElbow = null;
    public DcMotor glyphLift = null;
    public Servo jewelArm = null;
    public Servo jewelShoulder = null;
    public Servo relicWrist = null;
    public Servo relicHand = null;
    public Servo bottomLeftClaw = null;
    public Servo bottomRightClaw = null;
    public Servo topLeftClaw = null;
    public Servo topRightClaw = null;
    public ColorSensor colorSensor = null;
    public ModernRoboticsI2cGyro gyroSensor = null;

    //Create the hardware map object for our robot
    HardwareMap hwMap = null;

    //Initialize the hardware for out bot
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Find and configure MOTORS
        frontRight      = hwMap.dcMotor.get("front_right");
        backRight       = hwMap.dcMotor.get("back_right");
        frontLeft       = hwMap.dcMotor.get("front_left");
        backLeft        = hwMap.dcMotor.get("back_left");
        relicShoulder   = hwMap.dcMotor.get("relic_shoulder");
        relicElbow      = hwMap.dcMotor.get("relic_elbow");
        glyphLift       = hwMap.dcMotor.get("glyph_lift");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        relicShoulder.setDirection(DcMotor.Direction.REVERSE);
        relicElbow.setDirection(DcMotor.Direction.FORWARD);
        glyphLift.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glyphLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        relicShoulder.setPower(0);
        relicElbow.setPower(0);
        glyphLift.setPower(0);

        // Find and configure SERVOS
        jewelArm = hwMap.get(Servo.class, "jewel_arm");
        jewelShoulder = hwMap.get(Servo.class, "jewel_shoulder");
        relicWrist = hwMap.get(Servo.class, "relic_wrist");
        relicHand = hwMap.get(Servo.class, "relic_hand");
        bottomLeftClaw = hwMap.get(Servo.class, "bottom_left_glyph");
        bottomRightClaw = hwMap.get(Servo.class, "bottom_right_glyph");
        topLeftClaw = hwMap.get(Servo.class, "top_left_glyph");
        topRightClaw = hwMap.get(Servo.class, "top_right_glyph");

        jewelArm.setPosition(0);
        jewelShoulder.setPosition(0.5);
        relicWrist.setPosition(0);
        relicHand.setPosition(0);
        bottomLeftClaw.setPosition(0);
        bottomRightClaw.setPosition(1);
        topLeftClaw.setPosition(0);
        topRightClaw.setPosition(1);

        // Find and configure SENSORS
        colorSensor = hwMap.get(ColorSensor.class, "color_sensor");
        gyroSensor = hwMap.get(ModernRoboticsI2cGyro.class, "gyro_sensor");

    }

}
