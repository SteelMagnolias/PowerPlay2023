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
public class Wliftdrive extends OpMode {


    // two motors of w lift
    private DcMotor armleft; //arm
    private DcMotor armright; //arm
    private DcMotor leftfront; //wheelies
    private DcMotor rightfront; //wheelies
    private DcMotor leftback; //wheelies
    private DcMotor rightback; //wheelies
    private CRServo clawl1;//servo on claw
    private CRServo clawr2;//servo on claw
    private TouchSensor touch;// touch sensor for resetting levels


    private static final int REVERSE = -1;
    private static final double DEAD_ZONE = 0.1;
    private static final double OFF = 0;


    // naming levels as so connect under time
    ElapsedTime timer;

    private enum ArmState {
        BOTTOM,
        LOW,
        MIDDLE,
        TALL,
        RESET
    }
    ArmState levels;

    @Override
    public void init() {
        armleft = hardwareMap.get(DcMotor.class, "left"); //arm Dcmotor on left
        armright = hardwareMap.get(DcMotor.class, "right"); // initialize motors.  Device name is name in config
        leftfront = hardwareMap.get(DcMotor.class, "leftfront"); //init motors
        rightfront = hardwareMap.get(DcMotor.class, "rightfront"); //right front DC motor wheels
        leftback = hardwareMap.get(DcMotor.class, "leftback"); // left back wheels Dcmotor
        rightback = hardwareMap.get(DcMotor.class, "rightback"); // right back wheels Dcmotor
        clawl1 = hardwareMap.get(CRServo.class, "claw1"); // Claw left 1 Continues servo
        clawr2 = hardwareMap.get(CRServo.class, "claw2"); // claw right 2 continues servo
        touch = hardwareMap.get(TouchSensor.class, "touch"); // Touch senor bottom of robot for levels

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

        clawl1.setDirection(CRServo.Direction.REVERSE); // reversed so servos move opposite ways to pull in / out
        clawr2.setDirection(CRServo.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        //Set initial powers to off
        leftback.setPower(OFF);
        rightback.setPower(OFF);
        leftfront.setPower(OFF);
        rightfront.setPower(OFF);
        armleft.setPower(OFF);
        armright.setPower(OFF);

        // set servos to 0
        clawl1.setPower(OFF);
        clawr2.setPower(OFF);


        // used for timed movements
        levels = ArmState.BOTTOM;
        timer = new ElapsedTime();
        timer.reset();
        ElapsedTime timer;


        //Right back button is pressed= reset
        boolean wasRBPressed = false;// Not sure if at the bottom(driver hasnt signaled if on bottom state) VERY IMPORTANT

    }
    //Variables for automating arm position with push of button
    // constant(s) for movement:


    // finite state machine that defines the position of the arm in relation to certain events.
    // bottom is the default

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    // this is where we loop all of our code in teleop
    public void loop() {
        // Assigning & Data
        //Gamepad 1= wheels/spin/etc.   Gamepad 2=Arm and claw
        double lefty1 = -(gamepad1.left_stick_y); // this is the value of gamepad1's left joystick y value-Wheel
        double leftx1 = gamepad1.left_stick_x; // this is the value of gamepad1's left joystick x value- Wheel
        double rightx1 = gamepad1.right_stick_x; // this is the value of gamepad1's right joystick x value- Wheel
        double righty1 = (gamepad1.right_stick_y); // this the value of gamepad1's right joystick y value-Wheel
        double lefty2 = -(gamepad2.left_stick_y); // this is the value of gamepad2's left joystick y value- Arm
        double leftx2 = gamepad2.left_stick_x; // this is the value of gamepad2's left joystick x value- UNUSED
        double rightx2 = gamepad2.right_stick_x; // this the value of gamepad2's right joystick x value-Claw
        double righty2 = (gamepad2.right_stick_y); // this is the value of gamepad2's right joystick y value UNUSED
        boolean buttonUp = gamepad1.dpad_up; // this is the value of gamepad1's up button on the dpad-Slow wheels
        boolean buttonDown = gamepad1.dpad_down; // this is the value of gamepad1's down button on the dpad-Slow wheels
        boolean buttonLeft = gamepad1.dpad_left; // this is the value of the gamepad1's left button on the dpad-slow wheels
        boolean buttonRight = gamepad1.dpad_right; // this is the value of the gamepad1's right button on the dpad- slow wheels
        boolean lb = gamepad1.left_bumper; // this is the value of the gamepad1's left bumper- Unused
        boolean rb = gamepad1.right_bumper; // this is the value of the gamepad1's right bumper-unused
        boolean a1 = gamepad1.a; // this is the value of the a button on gamepad1-Turbo mode
        boolean x1 = gamepad1.x; // this is the value of the x button on gamepad1- Unused
        boolean y1 = gamepad1.y; // this is the value of the y button on gamepad1-UNUSED
        boolean rt = gamepad1.right_stick_button; // this is the value of the button behind the right stick on gamepad1- UNUSED

        boolean buttonUp2 = gamepad2.dpad_up; // this is the value of the up button on gamepad2- SLow arm
        boolean buttonDown2 = gamepad2.dpad_down; // this is  the value of the down button on gamepad2-slow arm
        boolean b2 = gamepad2.b; // this is the value of the b button on gamepad2-Middle arm level (aka medium pole)
        boolean a2 = gamepad2.a; // this is the value of the a button on gamepad2- Bottom arm level (aka ground junction)
        boolean y2 = gamepad2.y; // this is the value of the y button on gamepad2- Tall arm level (aka tall pole)
        boolean x2 = gamepad2.x; // this is the value of the x button on gamepad2-Low arm level (aka low pole)
        boolean rb2 = gamepad2.right_bumper; //this is the value of right bumper on gamepad2- Reset arm level (aka go all the way down)


        // print values to console
        telemetry.addData("lefty1", lefty1);
        telemetry.addData("leftx1", leftx1);
        telemetry.addData("rightx1", rightx1);
        telemetry.addData("lefty2", lefty2);
        telemetry.addData("leftx2", leftx2);
        telemetry.addData("rightx2", rightx2);
        telemetry.addData("righty1", righty1);
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
        //When RB is pushed code knows that the arm is in the lowest state, can only be pushed once

        //When a certain button is pushed robot reacts fast and moves it's arm to level indicated by button
        //A2 is pushed will bring to bottom level
        //X2 is pushed will bring to low Level
        //B2 is pushed will bring to middle level
        //Y2 is pushed will bring to tall level
    
        // Finite State Machine - Levels (need to edit distances on time once tested)
        final int low = 100;
        final int middle = 300;
        final int tall = 600;

        switch (levels) {
            // at bottom continue to bottom or respond to button push
            case BOTTOM:
                if (x2) {
                    timer.reset();
                    levels = ArmState.LOW;
                }
                if (b2) {
                    timer.reset();
                    levels = ArmState.MIDDLE;
                }
                if (y2) {
                    timer.reset();
                    levels = ArmState.TALL;
                }
                break;
            // at low  continue to low or respond to button push
            case LOW:
                if (timer.milliseconds() < low) {
                    armleft.setPower(.8);
                    armright.setPower(.8);
                } else {
                    armleft.setPower(0);
                    armright.setPower(0);
                }
                if (rb2) {
                    levels = ArmState.RESET;
                }
                break;
            // at middle  continue to middle or respond to button push
            case MIDDLE:
                if (timer.milliseconds() < middle) {
                    armleft.setPower(.8);
                    armright.setPower(.8);
                } else {
                    armleft.setPower(0);
                    armright.setPower(0);
                }
                if (rb2) {
                    levels = ArmState.RESET;
                }
                break;
            // at Tall  continue to tall or respond to button push
            case TALL:
                if (timer.milliseconds() < tall) {
                    armleft.setPower(.8);
                    armright.setPower(.8);
                } else {
                    armleft.setPower(0);
                    armright.setPower(0);
                }
                if (rb2) {
                    levels = ArmState.RESET;
                }
                break;
            // at reset  continue to reset or respond to button push
            case RESET:
                if (!touch.isPressed()) {
                    armleft.setPower(-.8);
                    armright.setPower(-.8);
                } else {
                    levels = ArmState.BOTTOM;
                }
                break;
            default:
                levels = ArmState.BOTTOM;
        }

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

        // We think this code was for spinners
        //lefty2=spinners?
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
        telemetry.addData("leftclaw power", clawl1.getPower());
        telemetry.addData("rightclaw power", clawr2.getPower());

        if (Math.abs(righty2) <= DEAD_ZONE) {
            // nothing - stop spinning!
            clawl1.setPower(0);
            clawr2.setPower(0);
        } else if (righty2 > DEAD_ZONE) {
            // intake
            clawl1.setPower(REVERSE * pow);
            clawr2.setPower(REVERSE * pow);

        } else {
            clawl1.setPower(pow);
            clawr2.setPower(pow);
        }

    }


}


