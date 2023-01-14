package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class LinearSlidesDrive {

    // wheels
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // arm motors
    private DcMotor arm;
    private DcMotor arm2;

    // touch sensor for arm and intake to tell when something is fully in / down
    private TouchSensor armTouch;
    private TouchSensor intakeTouch;

    // touch sensors that touch the wall and get us straight
    private TouchSensor straightenRight;
    private TouchSensor straitenLeft;

    // distance sensor on arm
    private DistanceSensor armHeight;

    // intake servos (continuous)
    private CRServo rightintake;
    private CRServo leftintake;

    // servos for touch sensors
    private Servo rightTurnSensor;
    private Servo leftTurnSensor;


    // color sensors
    private ColorSensor colorBack;
    private ColorSensor colorLeft;
    private ColorSensor colorRight;

    // cameras (each go into webcam name for universal coding) OOP is cool my guys!
    private WebcamName webcamName; // webcam
    private WebcamName webcamName1;
    private WebcamName webcamName2;

    private final static int REVERSE = -1;
    private final static double POWER = 0.3;
    private int FORWARD;
    private int STRAFE;

    private int STPL;
    //starting place
    private int dropTime = 5000;
    //time it takes to drop cone
    private int loopCounter;
    private int sum = (loopCounter+1);
}
