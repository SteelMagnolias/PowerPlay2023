package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "LinearSlideTest", group = "Iterative Opmode")
public class LinearSlideTest extends OpMode {

    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    @Override
    public void loop() {
        double leftY1 = gamepad1.left_stick_y; // gamepad left y joystick on gamepad 1

        if (leftY1 >= 0.1) {
            // up
            motor.setPower(0.5);
        }
        else if (leftY1 <= 0.1) {
            // down
            motor.setPower(-0.5);
        }
        else {
            // off
            motor.setPower(0);
        }
    }

    @Override
    public void stop() {
    }
}