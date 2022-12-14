// imports
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;


// teleop - driver controlled
@TeleOp(name="EarlySeasonDrive", group="Iterative Opmode")
public class EarlySeasonDrive extends OpMode
{
    private DcMotor leftBack; //Motor
    private DcMotor rightBack; //Motor
    private DcMotor leftFront; // Motor
    private DcMotor rightFront; //Motor
    // wheels lol

    // arm motors, one on each side
    private DcMotor arm; //Arm
    private DcMotor arm2; //Arm

    // servos
    private CRServo leftSpin; // left on robot looking from the back
    private CRServo rightSpin; // right from the back perspective

    // touch sensors
    private TouchSensor intakeTouch; // Touch sensor on arm for intake system
    private TouchSensor armTouch; // Touch sensor for levels for bottom of robot to hit for reset levels

    // used for timed movements
    ElapsedTime timer;

    // finite state machine that defines the position of the arm in relation to certain events.
    // bottom is the default
    public enum ArmState {
        BOTTOM,
        LOWER,
        MIDDLE,
        UPPER,
        RESET ,
        GROUND, // Ground Junctions
    }
    ArmState levels;

    // constant(s) for movement:

    // reverse by multiplying by this number
    private static final int REVERSE = -1;
    private static final double DEAD_ZONE = 0.1;
    private static final double OFF = 0;

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

        // now in the same configuration in the control hub
        arm = hardwareMap.get(DcMotor.class, "arm"); // in config --> port 0 --> "arm"
        arm2 = hardwareMap.get(DcMotor.class, "arm2"); // in config --> port 3 --> "arm2"

        // intake
        leftSpin = hardwareMap.get(CRServo.class, "leftSpin"); // intake system servo
        rightSpin = hardwareMap.get(CRServo.class, "rightSpin"); // initialize our servos

        //touch sensors
        intakeTouch = hardwareMap.get(TouchSensor.class, "intakeTouch");//Touch sensor on intake system to stop spinning
        armTouch = hardwareMap.get(TouchSensor.class, "armTouch"); // Touch sensor for levels to reset on bottom of robot

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

      /*
      rightFront.setDirection(DcMotor.Direction.REVERSE);
      rightBack.setDirection(DcMotor.Direction.REVERSE);
      */

        // motors must move together.

        arm.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates and makes it go the correct way
        //arm2.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates

        rightSpin.setDirection(CRServo.Direction.REVERSE); // reversed so servos move opposite ways to pull in / out

        levels = ArmState.BOTTOM; // sets the current level according to finite state machine to the bottom.
        timer = new ElapsedTime(); // make a timer
        timer.reset(); // put timer at 0

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        leftBack.setPower(OFF);
        rightBack.setPower(OFF);
        leftFront.setPower(OFF);
        rightFront.setPower(OFF);
        arm.setPower(OFF);
        arm2.setPower(OFF);

        // set servos to 0
        leftSpin.setPower(OFF);
        rightSpin.setPower(OFF);
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
        boolean rb2 =gamepad2.right_bumper;// Reset
        boolean lb2 = gamepad2.left_bumper; // levels for stack of cones, keep clicking until reached prefered level

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
        telemetry.addData("lb2", lb2);

        //When a certain button is pushed robot reacts fast and moves it's arm to level indicated by button
        //A2 is pushed will bring to bottom level
        //X2 is pushed will bring to low Level
        //B2 is pushed will bring to middle level
        //Y2 is pushed will bring to tall level

        // Finite State Machine - Levels (need to edit distances on time once tested)
        final int ground = 200;
        final int low = 800;
        final int tall = 1200;

        switch (levels) {
            // at bottom continue to bottom or respond to button push
            case BOTTOM:
                arm.setPower(0);
                arm2.setPower(0);
                if (a2) {
                    timer.reset();
                    levels = EarlySeasonDrive.ArmState.LOWER;
                }
                if (b2) {
                    timer.reset();
                    levels = EarlySeasonDrive.ArmState.GROUND;
                }
                if (y2) {
                    timer.reset();
                    levels = ArmState.UPPER;
                }

                break;
            // at low  continue to low or respond to button push
            case LOWER:
                if (timer.milliseconds() < low) {
                    arm.setPower(.8);
                    arm2.setPower(.8);
                } else {
                    arm.setPower(0);
                    arm2.setPower(0);
                }
                if (rb2) {
                    levels = EarlySeasonDrive.ArmState.RESET;
                }
                break;
            // at middle  continue to middle or respond to button push
            case GROUND:
                if (timer.milliseconds() < ground) {
                    arm.setPower(.8);
                    arm2.setPower(.8);
                } else {
                    arm.setPower(0);
                    arm2.setPower(0);
                }
                if (rb2) {
                    levels = EarlySeasonDrive.ArmState.RESET;
                }
                break;
            // at Tall  continue to tall or respond to button push
            case UPPER:
                if (timer.milliseconds() < tall) {
                    arm.setPower(.8);
                    arm2.setPower(.8);
                } else {
                    arm.setPower(0);
                    arm2.setPower(0);
                }
                if (rb2) {
                    levels = EarlySeasonDrive.ArmState.RESET;
                }
                break;
            // at reset  continue to reset or respond to button push
            case RESET:
                if (!armTouch.isPressed()) {
                    arm.setPower(-.8);
                    arm2.setPower(-.8);
                } else {
                    levels = EarlySeasonDrive.ArmState.BOTTOM;
                }
                break;
            default:
                levels = EarlySeasonDrive.ArmState.BOTTOM;
        }

        double pow;
        if (a1) pow = 1; // turbo mode
        else pow =0.5;
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

        telemetry.addData("rightBack", rightBack.getPower());

        if (Math.abs(lefty2) >= DEAD_ZONE) {
            if (lefty2 < 0) {
                if (armTouch.isPressed()) {
                    // if button is pressed, stop moving
                    arm.setPower(0);
                    arm2.setPower(0);
                }
                else {
                    arm.setPower(lefty2 * pow);
                    arm2.setPower(lefty2 * pow);
                }            }
            if (lefty2 > 0) {
                arm.setPower(lefty2 * pow);
                arm2.setPower(lefty2 * pow);
            }
        } else if (buttonDown2) {
            arm.setPower(-pow / 2);
            arm2.setPower(-pow / 2);
        } else if (buttonUp2) {
            arm.setPower(pow / 2);
            arm2.setPower(pow / 2);
        } else {
            arm.setPower(0);
            arm2.setPower(0);
        }

        // add information on arm powers
        telemetry.addData("arm", arm.getPower());
        telemetry.addData("arm2",arm2.getPower());

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

        pow = 1; // this is the speed in which we will turn the servos

        telemetry.addData("Touchy", intakeTouch.isPressed());
        telemetry.addData("Right Joystick (righty2)", righty2);
        telemetry.addData("leftSpin power", leftSpin.getPower());
        telemetry.addData("rightSpin power", rightSpin.getPower());
        if (Math.abs(righty2) <= DEAD_ZONE) {
            // nothing - stop spinning!
            leftSpin.setPower(0);
            rightSpin.setPower(0);
        }
        else if (righty2 > DEAD_ZONE) {
            // intake
            leftSpin.setPower(REVERSE * pow);
            rightSpin.setPower(REVERSE * pow);

        } else  {
            // outtake

            if (intakeTouch.isPressed()) {
                leftSpin.setPower(0);
                rightSpin.setPower(0);
            }
            else {
                leftSpin.setPower(pow);
                rightSpin.setPower(pow);
            }

        }


        telemetry.update(); // print output


    }
    @Override
    public void stop() {
    }

}