package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Wliftdrive", group="Iterative Opmode")
public class Wliftdrive extends OpMode
{


    // two motors of w lift
    private DcMotor armleft; //arm
    private DcMotor armright; //arm
    private DcMotor leftfront; //wheelies
    private DcMotor rightfront; //wheelies
    private DcMotor leftback; //wheelies
    private DcMotor rightback; //wheelies
    private CRServo claw1;//servo on claw
    private CRServo claw2;//servo on claw


    private static final int REVERSE = -1;
    private static final double DEAD_ZONE = 0.1;
    private static final double OFF = 0;

    @Override
    public void init() {
        armleft = hardwareMap.get(DcMotor.class, "left");
        armright = hardwareMap.get(DcMotor.class, "right"); // initialize motors.  Device name is name in config
        leftfront=hardwareMap.get(DcMotor.class, "leftfront"); //init motors
        rightfront=hardwareMap.get(DcMotor.class, "rightfront");
        leftback=hardwareMap.get(DcMotor.class, "leftback");
        rightback=hardwareMap.get(DcMotor.class, "rightback");
        claw1=hardwareMap.get(CRServo.class,"claw1");
        claw2=hardwareMap.get(CRServo.class, "claw2");

        telemetry.addData("Status:", "initialized");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        /*
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        */

        // motors must move together.

        armleft.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates and makes it go the correct way
        armright.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates

        claw1.setDirection(CRServo.Direction.REVERSE); // reversed so servos move opposite ways to pull in / out
        claw2.setDirection(DcMotorSimple.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        leftback.setPower(OFF);
        rightback.setPower(OFF);
        leftfront.setPower(OFF);
        rightfront.setPower(OFF);
        armleft.setPower(OFF);
        armright.setPower(OFF);

        // set servos to 0
        claw1.setPower(OFF);
        claw2.setPower(OFF);
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

        leftfront.setPower(fl + rightx1);
        leftback.setPower(bl + rightx1);
        rightfront.setPower(fr - rightx1);
        rightback.setPower(br - rightx1);


        if (Math.abs(lefty2) >= DEAD_ZONE) {
            if (lefty2 < 0) {
                armleft.setPower(lefty2 * pow);
                armright.setPower(lefty2 * pow);
            }
            if (lefty2 > 0) {
                armleft.setPower(lefty2 * pow);
                armright.setPower(lefty2 * pow);
            }
        } else if (buttonDown2) {
            armleft.setPower(-pow / 2);
            armright.setPower(-pow / 2);
        } else if (buttonUp2) {
            armleft.setPower(pow / 2);
            armright.setPower(pow / 2);
        } else {
            armleft.setPower(0);
            armright.setPower(0);
        }

        // add information on arm powers
        telemetry.addData("arm", armleft.getPower());
        telemetry.addData("arm2", armright.getPower());

        // Below: precision (slower) movement
        pow *= 0.5;
        if (buttonUp) {
            // slowly moves forwards
            leftfront.setPower(pow);
            leftback.setPower(pow);
            rightfront.setPower(pow);
            rightback.setPower(pow);
        } else if (buttonDown) {
            // slowly moves backwards
            leftfront.setPower(-pow);
            leftback.setPower(-pow);
            rightfront.setPower(-pow);
            rightback.setPower(-pow);
        } else if (buttonRight) {
            // slowly moves right
            leftfront.setPower(pow);
            leftback.setPower(-pow);
            rightfront.setPower(-pow);
            rightback.setPower(pow);
        } else if (buttonLeft) {
            // slowly moves left
            leftfront.setPower(-pow);
            leftback.setPower(pow);
            rightfront.setPower(pow);
            rightback.setPower(-pow);
        } else {
            // stops movement
            leftfront.setPower(0);
            leftback.setPower(0);
            rightfront.setPower(0);
            rightback.setPower(0);
        }

        pow *= .6;

        if (rb) {
            // slowly moves clockwise
            leftfront.setPower(pow);
            leftback.setPower(pow);
            rightfront.setPower(-pow);
            rightback.setPower(-pow);
        } else if (lb) {
            // slowly moves counter-clockwise
            leftfront.setPower(-pow);
            leftback.setPower(-pow);
            rightfront.setPower(pow);
            rightback.setPower(pow);
        } else {
            // stops movement
            leftfront.setPower(0);
            leftback.setPower(0);
            rightfront.setPower(0);
            rightback.setPower(0);
        }

        pow = 1; // this is the speed in which we will turn the servos


        telemetry.addData("Right Joystick (righty2)", righty2);
        telemetry.addData("leftSpin power", claw1.getPower());
        telemetry.addData("rightSpin power", claw2.getPower());


        }


    }


