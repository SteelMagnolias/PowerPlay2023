package org.firstinspires.ftc.teamcode.Autonomous;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BackIn", group = "IterativeOpMode")

public class BackIn extends LinearOpMode{
    // declaration
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private TouchSensor touchy;
    private Servo camera;
    private CRServo rightintake;
    private CRServo leftintake;
    private DcMotor arm;
    private DcMotor arm2;


    private final static int REVERSE = -1; // this is the reverse
    private double pow = 0.3;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack"); // in config --> port 1 --> "leftBack"
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); // in config --> port 2 --> "rightBack
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // in config --> port 0 --> "leftFront"
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); // in config --> port 3 --> "rightFront"
        touchy = hardwareMap.get(TouchSensor.class, "touchy");  // in config --> digital port 5 --> "touchy"
        camera = hardwareMap.get(Servo.class, "camera"); // in config --> webcam1 --> camera
        rightintake = hardwareMap.get(CRServo.class, "rightSpin"); // in config --> port 3 --> "rightintake"
        leftintake = hardwareMap.get(CRServo.class, "leftSpin"); // in config --> port 4 --> "leftintake"
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        telemetry.addData("Status", "Initialized");

        waitForStart();

        drive(REVERSE * pow, REVERSE * pow, REVERSE * pow, REVERSE * pow, 2500);
    }

    public void drive (double lf, double rf, double lb, double rb, int time){
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        sleep(time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(10);
    }
}
