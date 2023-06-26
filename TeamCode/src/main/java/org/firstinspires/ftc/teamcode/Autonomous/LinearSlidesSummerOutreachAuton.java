package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name = "LinearSlidesSummerOutreachAuton", group="Iterative OpMode")
public class LinearSlidesSummerOutreachAuton extends LinearOpMode {

    // define hardware
    // wheels
    private DcMotor leftFront; //Left wheel front
    private DcMotor rightFront; //right wheel front
    private DcMotor leftBack; //left wheel back
    private DcMotor rightBack; //right wheel back

    // arm motors
    private DcMotor arm;
    private DcMotor arm2;

    // touch sensor for arm and intake to tell when something is fully in / down
    private TouchSensor armTouch;
    private TouchSensor intakeTouch;

    // touch sensors that touch the wall and get us straight
    private TouchSensor straightenRight; //Not used, using distance sensors
    private TouchSensor straightenLeft; //NOT used using distance sensor

    // intake servos (continuous)
    private CRServo rightspin; //intake severo 1
    private CRServo leftspin; //intake servo 2

    // constants
    public final static double MOTOR_SPEED = 0.3;
    public final static int INTAKE_SPEED = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize hardware
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // in config --> port 1 --> "leftBack"
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); // in config --> port 2 --> "rightBack
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // in config --> port 0 --> "leftFront"
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); // in config --> port 3 --> "rightFront"
        intakeTouch = hardwareMap.get(TouchSensor.class, "intakeTouch");
        armTouch = hardwareMap.get(TouchSensor.class, "armTouch");  // in config --> digital port 5 --> "touchy"
        rightspin = hardwareMap.get(CRServo.class, "rightspin"); // in config --> port 3 --> "rightintake"
        leftspin = hardwareMap.get(CRServo.class, "leftspin"); // in config --> port 4 --> "leftintake"
        //straightenRight = hardwareMap.get(TouchSensor.class, "straightenRight");
        //straightenLeft = hardwareMap.get(TouchSensor.class, "straightenLeft");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");

        // reverse necessary motors for correct movement
        rightspin.setDirection(CRServo.Direction.REVERSE); // reversed so servos move opposite ways to pull in / out
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates and makes it go the correct way

        waitForStart(); // until start button pushed

        // intake until button pressed
        rightspin.setPower(INTAKE_SPEED);
        leftspin.setPower(INTAKE_SPEED);

        while(!intakeTouch.isPressed()) {
            if (intakeTouch.isPressed()) {
                rightspin.setPower(0);
                leftspin.setPower(0);
            }
        }


        // lift arm for 2 seconds
        moveArm(MOTOR_SPEED, 2000);

        // drive forward for 2 seconds
        drive(MOTOR_SPEED, MOTOR_SPEED, MOTOR_SPEED, MOTOR_SPEED, 2000);

        // outtake for 3 seconds
        rightspin.setPower(-INTAKE_SPEED);
        leftspin.setPower(-INTAKE_SPEED);

        sleep(3000); // 3 seconds later

        // turn intake out
        rightspin.setPower(0);
        leftspin.setPower(0);

        // back for 2 seconds
        drive(-MOTOR_SPEED, -MOTOR_SPEED, -MOTOR_SPEED, -MOTOR_SPEED, 2000);


        // bring arm back down until arm touch pressed
        arm.setPower(-MOTOR_SPEED);
        arm2.setPower(-MOTOR_SPEED);

        while(!armTouch.isPressed()) {
            if (armTouch.isPressed()) {
                arm.setPower(0);
                arm2.setPower(0);
            }
        }

    }

    public void drive (double lf, double rf, double lb, double rb, double time){
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        sleep ((int)time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(10);
    }

    public void moveArm(double pow, int AH) {
        // lift, but with an assigned power as well.

        arm.setPower(pow);
        arm2.setPower(pow);
        sleep(AH);
        arm.setPower(0);
        arm2.setPower(0);
        sleep(10);
    }
}
