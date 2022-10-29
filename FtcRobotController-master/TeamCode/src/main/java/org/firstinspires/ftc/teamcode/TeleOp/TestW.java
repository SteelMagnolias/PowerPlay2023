package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="EarlySeasonDrive", group="Iterative Opmode")
public class TestW extends OpMode {

    // two motors of w lift
    private DcMotor left;
    private DcMotor right;

    private double leftY; // left joystick

    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right"); // initialize motors.  Device name is name in config

        telemetry.addData("Status:", "initialized");
    }

    @Override
    public void loop() {
        leftY = gamepad1.left_stick_y; // get the value of the gamepad left stick

        if (leftY>0.1) {
            left.setPower(0.3);
            right.setPower(0.3);
        }
        else if (leftY<-0.1) {
            left.setPower(-0.3);
            right.setPower(-0.3);
        }
        else {
            left.setPower(0.0);
            right.setPower(0.0);

        }
    }

    @Override
    public void stop() {

    }
}
