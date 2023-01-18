package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
@TeleOp (name = "LinearSlidesDrive" , group = "Iterative Opmode")
public class LinearSlidesDrive extends OpMode {
    // wheels
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // arm motors
    private DcMotor arm;
    private DcMotor arm2;

    // touch sensor for arm and intake to tell when something is fully in / down
    private TouchSensor armTouch;
    private TouchSensor intakeTouch;

    // touch sensors that touch the wall and get us straight
    private TouchSensor straightenRight;
    private TouchSensor straightenLeft;

    // distance sensor on arm
    private DistanceSensor armHeight;

    // intake servos (continuous)
    private CRServo rightspin;
    private CRServo leftspin;

    private ColorSensor colorLeft;
    private ColorSensor colorRight;

    // cameras (each go into webcam name for universal coding) OOP is cool my guys!
    private WebcamName webcamName; // webcam
    private WebcamName webcamName1;
    private WebcamName webcamName2;

    public enum ArmState {
        BOTTOM,
        LOWER,
        MIDDLE,
        UPPER,
        RESET ,
        GROUND, // Ground Junctions
    }
    ArmState levels;

    private final static int REVERSE = -1;
    private final static double POWER = 0.3;
    private int FORWARD;
    private int STRAFE;

    private int dropTime = 5000;
    //time it takes to drop cone

    private static final double DEAD_ZONE = 0.1;
    private static final double OFF = 0;

    // finite state machine that defines the position of the arm in relation to certain events.
    // bottom is the default

    private boolean found = false; // tells whether we have reached target height once

    @Override
    public void init() {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // in config --> port 1 --> "leftBack"
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); // in config --> port 2 --> "rightBack
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // in config --> port 0 --> "leftFront"
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); // in config --> port 3 --> "rightFront"

        intakeTouch = hardwareMap.get(TouchSensor.class, "intakeTouch");
        armTouch = hardwareMap.get(TouchSensor.class, "armTouch");  // in config --> digital port 5 --> "touchy"

        rightspin = hardwareMap.get(CRServo.class, "rightspin"); // in config --> port 3 --> "rightintake"
        leftspin = hardwareMap.get(CRServo.class, "leftspin"); // in config --> port 4 --> "leftintake"

        straightenRight = hardwareMap.get(TouchSensor.class, "straightenRight");
        straightenLeft = hardwareMap.get(TouchSensor.class, "straightenLeft");

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");

        armHeight = hardwareMap.get(DistanceSensor.class, "armHeight");

        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");

        webcamName1 = hardwareMap.get(WebcamName.class, "Webcam 2");
        webcamName2 = hardwareMap.get(WebcamName.class, "Webcam 1");

        //arm.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates and makes it go the correct way
        //arm2.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates

        rightspin.setDirection(CRServo.Direction.REVERSE); // reversed so servos move opposite ways to pull in / out

        rightBack.setDirection(CRServo.Direction.REVERSE);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        leftBack.setPower(OFF);
        rightBack.setPower(OFF);
        leftFront.setPower(OFF);
        rightFront.setPower(OFF);
        arm.setPower(OFF);
        arm2.setPower(OFF);

        // set servos to 0
        leftspin.setPower(OFF);
        rightspin.setPower(OFF);

        //levels = ArmState.RESET;
        // edit 11 - start the armstate at BOTTOM so we can choose.  The way it is rewritten currently does not allow for the arm to come all the way down straight away. we have to start with it on the bottom, or we can use the manual controls
        levels = ArmState.BOTTOM;
    }

    // this is where we loop all of our code in teleop
    @Override
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
        boolean rb2 = gamepad2.right_bumper;// Reset
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

        final double bottom = 4.0;
        final double low = 19.5;
        final double middle = 29.5;
        final double tall = 35.0;

        switch (levels) {
            // at bottom continue to bottom or respond to button push
            case BOTTOM:
                arm.setPower(0);
                arm2.setPower(0);

                found = false;

                if (a2) {
                    levels = ArmState.LOWER;
                    
                    // edit 1 - get rid of this if it doesn't work.  We're going set the motors here, so that it only happens once and won't be starting over and over and over, sending a burst of energy each time
                    arm.setPower(.6);
                    arm2.setPower(.6);
                }
                if (b2) {
                    levels = ArmState.MIDDLE;
                    
                    // edit 2 - get rid of this if it doesn't work.  like edit 1, same pupose.
                    arm.setPower(.6);
                    arm2.setPower(.6);
                }
                if (y2) {
                    levels = ArmState.UPPER;
                    
                    // edit 3, same as 1 and 2
                    arm.setPower(.6);
                    arm2.setPower(.6);
                }
                break;
            // at low  continue to low or respond to button push
            case LOWER:
                /*
                if (armHeight.getDistance(DistanceUnit.INCH) < low && !found) {
                    arm.setPower(.6);
                    arm2.setPower(.6);
                } else {
                    arm.setPower(0);
                    arm2.setPower(0);
                    found = true;
                }
                */
                
                // edit 4 - replacement code that just stops the motors when it reaches low. (old code commented out above)
                if (armHeight.getDistance(DistanceUnit.INCH) >= low) {
                    arm.setPower(0);
                    arm2.setPower(0);
                }
                
                if (rb2) {
                    levels = ArmState.RESET;
                    
                    // edit 7 - change so the arm only sets motors once
                    arm.setPower(-0.3);
                    arm2.setPower(-0.3);
                }
                break;
            // at middle  continue to middle or respond to button push
            // at Tall  continue to tall or respond to button push
            case MIDDLE:
                /*
                if (armHeight.getDistance(DistanceUnit.INCH) < middle && !found) {
                arm.setPower(.6);
                arm2.setPower(.6);
                } else {
                    arm.setPower(0);
                    arm2.setPower(0);
                    found = true;
                }
                */
                
                // edit 5 - replacement code that just stops the motors when it reaches mid. (old code commented out above)
                if (armHeight.getDistance(DistanceUnit.INCH) >= middle) {
                    arm.setPower(0);
                    arm2.setPower(0);
                }
                
                if (rb2) {
                    levels = ArmState.RESET;
                    
                    // edit 8 - change so the arm only sets motors once
                    arm.setPower(-0.3);
                    arm2.setPower(-0.3);
                }
                break;
            case UPPER:
                
                /*
                if (armHeight.getDistance(DistanceUnit.INCH) < tall  &&  !found) {
                    arm.setPower(.6);
                    arm2.setPower(.6);
                } else {
                    arm.setPower(0);
                    arm2.setPower(0);
                    found = true;
                }
                */
                
                // edit 6 - replacement code that just stops the motors when it reaches tall. (old code commented out above)
                if (armHeight.getDistance(DistanceUnit.INCH) >= tall) {
                    arm.setPower(0);
                    arm2.setPower(0);
                }
                
                if (rb2) {
                    levels = ArmState.RESET;
                    
                    // edit 9 - change so the arm only sets motors once
                    arm.setPower(-0.3);
                    arm2.setPower(-0.3);
                }
                break;
            // at reset  continue to reset or respond to button push
            case RESET:
                /*
                if (!armTouch.isPressed()) {
                    arm.setPower(-.3);
                    arm2.setPower(-.3);
                } else {
                    levels = ArmState.BOTTOM;
                }
                */
                
                // edit 10, old code commented out above.  Arm would already be moving downward
                if (armTouch.isPressed()) {
                    // bottom case has it stop motor power
                    levels = ArmState.BOTTOM;
                }
                break;
            default:
                levels = ArmState.BOTTOM;
        }


        double pow;
        if (a1) pow = 1; // turbo mode
        else pow = 0.5;
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

        telemetry.addData("distance" , armHeight.getDistance(DistanceUnit.INCH));
        telemetry.addData("red", colorLeft.red() );
        telemetry.addData("blue", colorLeft.blue());
        telemetry.addData("red", colorRight.red());
        telemetry.addData("blue",colorRight.blue());
        telemetry.addData("touchsensor1", armTouch.isPressed());
        telemetry.addData("touchsensor2", armTouch.isPressed());
        telemetry.addData("touchsensor3" , armTouch.isPressed());
        telemetry.addData("touchsensor4" , armTouch.isPressed());
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

        pow = 0.8;

        if (Math.abs(lefty2) >= DEAD_ZONE) {
            if (lefty2 < 0) {
                if (armTouch.isPressed()) {
                    // if button is pressed, stop moving
                    arm.setPower(0);
                    arm2.setPower(0);
                } else {
                    arm.setPower(lefty2 * pow);
                    arm2.setPower(lefty2 * pow);
                }
            }
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
        telemetry.addData("arm2", arm2.getPower());

        // Below: precision (slower) movement
        pow = 0.5;
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

        pow = .6;

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
        telemetry.addData("leftSpin power", leftspin.getPower());
        telemetry.addData("rightSpin power", rightspin.getPower());
        if (Math.abs(righty2) <= DEAD_ZONE) {
            // nothing - stop spinning!
            leftspin.setPower(0);
            rightspin.setPower(0);
        } else if (righty2 > DEAD_ZONE) {
            // intake
            leftspin.setPower(REVERSE * pow);
            rightspin.setPower(REVERSE * pow);

        } else {
            // outtake

            if (intakeTouch.isPressed()) {
                leftspin.setPower(0);
                rightspin.setPower(0);
            } else {
                leftspin.setPower(pow);
                rightspin.setPower(pow);
            }


        }

        telemetry.update(); // print output
    }

    @Override
    public void stop() {

    }

    public void lift (double AH) {
        if (armHeight.getDistance(DistanceUnit.INCH)>=AH){
            while (armHeight.getDistance(DistanceUnit.INCH)>=AH){
                arm.setPower(-0.3);
                arm2.setPower(-0.3);
            }
        }
        else if (armHeight.getDistance(DistanceUnit.INCH)<AH){
            while (armHeight.getDistance(DistanceUnit.INCH)<AH){
                arm.setPower(0.3);
                arm2.setPower(0.3);
            }
        }
        arm.setPower(0);
        arm2.setPower(0);
    }
}
