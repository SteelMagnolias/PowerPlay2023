package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "WheeledIntakeTest", group = "Iterative Opmode")
public class WheeledIntakeTest extends OpMode {


    // two motors of w lift
    private CRServo left;
    private CRServo right;

    private double leftY; // left joystick

    @Override
    public void init() {
        left = hardwareMap.get(CRServo.class, "left");
        // = hardwareMap.get(CRServo.class, "right"); // initialize motors.  Device name is name in config

        //right.setDirection(CRServo.Direction.REVERSE);

        telemetry.addData("Status:", "initialized");
    }

    @Override
    public void loop() {
        leftY = gamepad1.left_stick_y; // get the value of the gamepad left stick

        if (leftY > 0.1) {
            left.setPower(1);
            //right.setPower(-1);
        } else if (leftY < -0.1) {
            left.setPower(-1);
            //right.setPower(1);
        } else {
            left.setPower(0.0);
            //right.setPower(0.0);

        }

        telemetry.addData("Left Servo", left.getPower());
        //telemetry.addData("Right Servo", right.getPower());

        telemetry.update();
    }

    @Override
    public void stop() {

    }
}