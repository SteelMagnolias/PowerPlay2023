// imports
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

// teleop - driver controlled
@TeleOp(name="JerseyBot", group="Iterative Opmode")
public class JerseyBot extends OpMode
{
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    // wheels lol


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // initialize motors and link to configuration, configuartion is "testOct9" in the expansion hub
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack"); // in config --> port 1 --> "leftBack"
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); // in config --> port 2 --> "rightBack"
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // in config --> port 0 --> "leftFront"
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); // in config --> port 3 --> "rightFront"
        telemetry.addData("Status", "Initialized");

        leftFront.setDirection(DcMotor.Direction.REVERSE);  // reverse for correct movement
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    // this is where we loop all of our code in teleop
    public void loop() {
        // Assigning & Data
        double lefty1 = -(gamepad1.left_stick_y); // this is the value of gamepad1's left joystick y value
        double leftx1 = gamepad1.left_stick_x; // this is the value of gamepad1's left joystick x value
        double rightx1 = gamepad1.right_stick_x; // this is the value of gamepad1's right joystick x value
        double righty1 = (gamepad1.right_stick_y); // this the value of gamepad1's right joystick y value
        double lefty2 = -(gamepad2.left_stick_y); // this is the value of gamepad2's left joystick y value
        double leftx2 = gamepad2.left_stick_x; // this is the value of gamepad2's left joystick x value
        double rightx2 = gamepad2.right_stick_x; // this the value of gamepad2's right joystick x value
        double righty2 = (gamepad2.right_stick_y); // this is the value of gamepad2's right joystick y value
        boolean buttonUp = gamepad1.dpad_up; // this is the value of gamepad1's up button on the dpad
        boolean buttonDown = gamepad1.dpad_down; // this is the value of gamepad1's down button on the dpad
        boolean buttonLeft = gamepad1.dpad_left; // this is the value of the gamepad1's left button on the dpad
        boolean buttonRight = gamepad1.dpad_right; // this is the value of the gamepad1's right button on the dpad
        boolean lb = gamepad1.left_bumper; // this is the value of the gamepad1's left bumper
        boolean rb = gamepad1.right_bumper; // this is the value of the gamepad1's right bumper
        boolean a1 = gamepad1.a; // this is the value of the a button on gamepad1
        boolean x1 = gamepad1.x; // this is the value of the x button on gamepad1
        boolean y1 = gamepad1.y; // this is the value of the y button on gamepad1
        boolean rt = gamepad1.right_stick_button; // this is the value of the button behind the right stick on gamepad1

        boolean buttonUp2 = gamepad2.dpad_up; // this is the value of the up button on gamepad2
        boolean buttonDown2 = gamepad2.dpad_down; // this is  the value of the down button on gamepad2
        boolean b2 = gamepad2.b; // this is the value of the b button on gamepad2
        boolean a2 = gamepad2.a; // this is the value of the a button on gamepad2
        boolean y2 = gamepad2.y; // this is the value of the y button on gamepad2
        boolean x2 = gamepad2.x; // this is the value of the x button on gamepad2

        // print values to console
        telemetry.addData("lefty1", lefty1);
        telemetry.addData("leftx1", leftx1);
        telemetry.addData("rightx1", rightx1);
        telemetry.addData("lefty2", lefty2);
        telemetry.addData("leftx2", leftx2);
        telemetry.addData("rightx2", rightx2);
        telemetry.addData("buttonUp", buttonUp);
        telemetry.addData("buttonDown", buttonDown);
        telemetry.addData("buttonRight", buttonRight);
        telemetry.addData("buttonLeft", buttonLeft);
        telemetry.addData("lb", lb);
        telemetry.addData("rb", rb);
        telemetry.addData("a", a2);
        telemetry.addData("b", b2);
        telemetry.addData("x", x2);
        telemetry.addData("y", y2);
        telemetry.addData("rt", rt);

        double pow;
        if (a1) pow = 1; // turbo mode
        else pow = .8;
        double c = Math.hypot(leftx1, lefty1);
        double perct = pow * c;
        if (c <= .1) perct = 0;
        //
        double theta;

        if (leftx1 <= 0 && lefty1 >= 0) {
            theta = Math.atan(Math.abs(leftx1) / Math.abs(lefty1));
            theta += (Math.PI / 2);
        } else if (leftx1 < 0 && lefty1 <= 0) {
            theta = Math.atan(Math.abs(lefty1) / Math.abs(leftx1));
            theta += (Math.PI);
        } else if (leftx1 >= 0 && lefty1 < 0) {
            theta = Math.atan(Math.abs(leftx1) / Math.abs(lefty1));
            theta += (3 * Math.PI / 2);
        } else {
            theta = Math.atan(Math.abs(lefty1) / Math.abs(leftx1));
        }

        double dir = 1;
        if (theta >= Math.PI) {
            theta -= Math.PI;
            dir = -1;
        }
        //if (leftx1 <= 0 && lefty1 >= 0 || leftx1 >= 0 && lefty1 <= 0){
        //   theta += (Math.PI/2);
        //}

        telemetry.addData("pow", pow);
        telemetry.addData("dir", dir);
        telemetry.addData("c", c);
        telemetry.addData("theta", theta);

        double fr = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4));
        if (fr > 1) fr = 1;
        if (fr < -1) fr = -1;
        fr = (perct * fr);
        if (leftx1 == 0 && lefty1 == 0) fr = 0;

        double bl = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4));
        if (bl > 1) bl = 1;
        if (bl < -1) bl = -1;
        bl = (perct * bl);
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) bl = 0;

        double fl = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (fl > 1) fl = 1;
        if (fl < -1) fl = -1;
        fl = (perct * fl);
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) fl = 0;

        double br = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (br > 1) br = 1;
        if (br < -1) br = -1;
        br = (perct * br);
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) br = 0;

        telemetry.addData("fl", fl);
        telemetry.addData("fr", fr);
        telemetry.addData("bl", bl);
        telemetry.addData("br", br);

        telemetry.addData("rlf", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rrf", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbl", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbr", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));

        leftFront.setPower(fl + rightx1);
        leftBack.setPower(bl + rightx1);
        rightFront.setPower(fr - rightx1);
        rightBack.setPower(br - rightx1);



        // Below: precision (slower) movement
        pow *= 0.5;
        if (buttonUp) {
            // slowly moves forwards
            leftFront.setPower(pow);
            leftBack.setPower(pow);
            rightFront.setPower(pow);
            rightBack.setPower(pow);
        } else if (buttonDown) {
            // slowly moves backwards
            leftFront.setPower(-pow);
            leftBack.setPower(-pow);
            rightFront.setPower(-pow);
            rightBack.setPower(-pow);
        } else if (buttonRight) {
            // slowly moves right
            leftFront.setPower(pow);
            leftBack.setPower(-pow);
            rightFront.setPower(-pow);
            rightBack.setPower(pow);
        } else if (buttonLeft) {
            // slowly moves left
            leftFront.setPower(-pow);
            leftBack.setPower(pow);
            rightFront.setPower(pow);
            rightBack.setPower(-pow);
        } else {
            // stops movement
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }

        pow *= .6;

        if (rb) {
            // slowly moves clockwise
            leftFront.setPower(pow);
            leftBack.setPower(pow);
            rightFront.setPower(-pow);
            rightBack.setPower(-pow);
        } else if (lb) {
            // slowly moves counter-clockwise
            leftFront.setPower(-pow);
            leftBack.setPower(-pow);
            rightFront.setPower(pow);
            rightBack.setPower(pow);
        } else {
            // stops movement
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
    }
    @Override
    public void stop() {
    }

}