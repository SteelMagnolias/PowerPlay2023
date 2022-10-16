package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="EarlySeasonSignalDropParkRed", group = "Iterative OpMode")
public class EarlySeasonSignalDropParkRed extends LinearOpMode {

    // declare motor wheels
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    // declare arm motors
    private DcMotor arm;
    private DcMotor arm2;

    // declare servos
    private CRServo leftSpin;
    private CRServo rightSpin;

    private int signal = 0; // this is the signal that is read by the camera

    @Override
    public void runOpMode() throws InterruptedException {
        // run code

        // initialize motors and servos later smol one
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        leftSpin = hardwareMap.get(CRServo.class, "leftSpin");
        rightSpin = hardwareMap.get(CRServo.class, "rightSpin");

        // actual fun part

        // scan signals

        waitForStart();

        // cap medium pole

        // drive forward 1 square
        // strafe right 1/2 sqaure
        // move forward a little
        // raise arm to medium goal height
        // outtaking / drop cone

        // move to parking
        // move back the same distance we moved forward after strafing

        if (signal == 1) {
            // move to location 1 by strafing left 2 tiles
        }
        else if (signal == 2) {
            // move to location 2 by strafing left 1 tile
        }
        else {
            // must be location 3 - strafe right 1 tile
        }

        // yey we cool
    }

    public void drive(double lf, double rf, double lb, double rb, int time) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);

        sleep(time); // sleep for set time.

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        sleep(time);
    }
}
