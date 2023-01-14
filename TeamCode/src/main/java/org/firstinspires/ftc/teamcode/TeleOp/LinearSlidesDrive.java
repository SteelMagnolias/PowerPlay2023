package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
@TeleOp (name = "LinearSlidesDrive" , group = "Interative OpMode")
public class LinearSlidesDrive extends OpMode
{
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


    public void runOpMode() throws InterruptedException {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // in config --> port 1 --> "leftBack"
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); // in config --> port 2 --> "rightBack
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // in config --> port 0 --> "leftFront"
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); // in config --> port 3 --> "rightFront"
        intakeTouch = hardwareMap.get(TouchSensor.class, "intakeTouch");
        armTouch = hardwareMap.get(TouchSensor.class, "armTouch");  // in config --> digital port 5 --> "touchy"
        rightintake = hardwareMap.get(CRServo.class, "rightClaw"); // in config --> port 3 --> "rightintake"
        leftintake = hardwareMap.get(CRServo.class, "leftClaw"); // in config --> port 4 --> "leftintake"
        straightenRight = hardwareMap.get(TouchSensor.class, "straitenRight");
        straitenLeft = hardwareMap.get(TouchSensor.class, "straightenLeft");
        rightTurnSensor= hardwareMap.get(Servo.class, "rightTurnSensor");
        leftTurnSensor = hardwareMap.get(Servo.class, "leftTurnSensor");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        armHeight = hardwareMap.get(DistanceSensor.class, "armHeight");

        webcamName1 = hardwareMap.get(WebcamName.class, "Webcam 2");
        webcamName2 = hardwareMap.get(WebcamName.class, "Webcam 1");

    }
