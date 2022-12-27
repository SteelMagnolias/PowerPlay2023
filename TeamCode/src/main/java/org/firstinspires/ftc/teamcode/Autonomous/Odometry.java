package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//https://gm0.org/en/latest/docs/software/concepts/odometry.html

@Autonomous(name = "Odometry", group = "IterativeOpMode")

public class Odometry extends LinearOpMode {

    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    // wheels lol

    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;

    // describing the robot's position {x, y, angle}
    int[] pose = {0, 0, 0};

    int prevLeft = 0; // previous left encoder position
    int prevRight = 0; // previous right encoder position
    int prevBack = 0; // previous back encoder position

    int changeLeft = 0; // change in left encoder
    int changeRight = 0; // change in right encoder
    int changeBack = 0; // change in back encoder

    int changeX = 0; // change in x
    int changeY = 0; // change in y
    int theta = 0; // what angle is robot at?

    int trackWidth = 0; // this is the distance between the two encoders in inches?

    @Override
    public void runOpMode() {
        // this is what happens in the autonomous code.
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftEncoder = leftFront; // basically, we just need to tie each of these to a port, which happens to be with a motor.
        rightEncoder = rightFront; // i could access the motor, and try to remember which is which encoder, but this is easier
        backEncoder = rightBack;

        // reset encoders
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // robot did not move, so no change
        prevLeft = 0;
        prevRight = 0;
        prevBack = 0;

        waitForStart();

        // find change in each encoder
        changeLeft = leftEncoder.getCurrentPosition() - prevLeft;
        changeRight = rightEncoder.getCurrentPosition() - prevRight;
        changeBack = backEncoder.getCurrentPosition() - prevBack;

        // change in x

        // change in y

        // angle of robot
        theta = (changeLeft - changeRight) / trackWidth;

        // pose = old data + new data

        // update pose
    }
}
