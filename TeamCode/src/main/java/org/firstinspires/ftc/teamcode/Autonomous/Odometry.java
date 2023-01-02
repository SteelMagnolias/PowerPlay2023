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

    // describing the robot's position {x, y, heading}
    double[] pose = {0, 0, 0};

    double prevLeft = 0; // previous left encoder position
    double prevRight = 0; // previous right encoder position
    double prevBack = 0; // previous back encoder position

    double curLeft = 0; // current left encoder position
    double curRight = 0; // current right encoder position
    double curBack = 0; // current back encoder position

    double changeLeft = 0; // change in left encoder
    double changeRight = 0; // change in right encoder
    double changeBack = 0; // change in back encoder

    double changeMiddlePosition = 0; // this is measuring how the center of the robot has changed position based off the average of the left and right encoder readings
    double changeBackEncoderPos = 0; // change in the back encoder position

    double changeX = 0; // change in x
    double changeY = 0; // change in y
    double theta = 0; // what angle is robot at?

    double trackWidth = 0; // this is the distance between the two encoders in inches?
    double forwardOffset = 0; // this is the distance from the perpendicular encoder to the center of robot in inches


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

        // get current encoder readings
        curLeft = leftEncoder.getCurrentPosition();
        curRight = rightEncoder.getCurrentPosition();
        curBack = backEncoder.getCurrentPosition();

        // find change in each encoder
        changeLeft = curLeft - prevLeft;
        changeRight = curRight - prevRight;
        changeBack = curBack - prevBack;

        // angle of robot
        theta = (changeLeft - changeRight) / trackWidth;

        // change in middle position of robot
        changeMiddlePosition = (changeLeft + changeRight)/2;

        // change in backEncoder's Position
        changeBackEncoderPos = changeBack - forwardOffset * theta;

        // change in x - basically using triangles to figure out the length of change in x
        changeX = changeMiddlePosition * Math.cos(pose[2]) - changeBackEncoderPos * Math.sin(pose[2]);

        // change in y - basically using triangles to figure out the length of change in y
        changeY = changeMiddlePosition * Math.sin(pose[2]) - changeBackEncoderPos * Math.cos(pose[2]);

        // updated pose = old data + new data
        pose[0] += changeX; // update the x position
        pose[1] += changeY; // update the y position
        pose[2] += theta; // update the angle the robot is pointing

        // update previous values
        prevLeft = curLeft;
        prevRight = curRight;
        prevBack = curBack;

    }
}
