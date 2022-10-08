// imports
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

// teleop - driver controlled
@TeleOp(name="Oct9Run", group="Iterative Opmode")
public class Oct9Run extends OpMode
{
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    // wheels lol

    // arm motors, one on each side
    private DcMotor arm;
    private DcMotor arm2;

    // used for timed movements
    ElapsedTime timer;

    // finite state machine that defines the position of the arm in relation to certain events.
    // bottom is the default
    public enum ArmState {
        BOTTOM,
        LOWER,
        MIDDLE,
        UPPER,
        RESET
    };
    ArmState levels;

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

        /*
        ducky = hardwareMap.get(DcMotor.class, "ducky");
        intake = hardwareMap.get(DcMotor.class, "intake");

        colorServo = hardwareMap.get(Servo.class, "colorServo");

        frontColor = hardwareMap.get(ColorSensor.class, "frontColor");
        frontColor.enableLed(false);
        // front sensor initialize

        midColor = hardwareMap.get(ColorSensor.class, "midColor");
        midColor.enableLed(false);
        // mid sensor initialize

        touch = hardwareMap.get(TouchSensor.class, "touch");
        //  touch.setMode(DigitalChannel.Mode.INPUT);
        // touch sensor that reads environment

         */

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        /*
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        */

        // motors must move together.

        arm.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates and makes it go the correct way
        arm2.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates

        levels = ArmState.BOTTOM; // sets the current level according to finite state machine to the bottom.
        timer = new ElapsedTime(); // make a timer
        timer.reset(); // put timer at 0

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    // this is where we loop all of our code in teleop
    public void loop() {
        // Assigning & Data
        double lefty1 = -(gamepad1.left_stick_y); // this is the value of gamepad1's left joystick y value
        double leftx1 = gamepad1.left_stick_x;
        double rightx1 = gamepad1.right_stick_x;
        double righty1 = (gamepad1.right_stick_y);
        double lefty2 = -(gamepad2.left_stick_y);
        double leftx2 = gamepad2.left_stick_x;
        double rightx2 = gamepad2.right_stick_x;
        double righty2 = (gamepad2.right_stick_y);
        boolean buttonUp = gamepad1.dpad_up;
        boolean buttonDown = gamepad1.dpad_down;
        boolean buttonLeft = gamepad1.dpad_left;
        boolean buttonRight = gamepad1.dpad_right;
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean a1 = gamepad1.a;
        boolean x1 = gamepad1.x;
        boolean y1 = gamepad1.y;
        boolean rt = gamepad1.right_stick_button;

        boolean buttonUp2 = gamepad2.dpad_up;
        boolean buttonDown2 = gamepad2.dpad_down;
        boolean b2 = gamepad2.b;
        boolean a2 = gamepad2.a;
        boolean y2 = gamepad2.y;
        boolean x2 = gamepad2.x;

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


        // Finite State Machine - Levels
        final int lower = 100;
        final int middle = 300;
        final int upper = 600;

        switch (levels) {
            case BOTTOM:
                if (a2) {
                    timer.reset();
                    levels = ArmState.LOWER;
                }
                if (b2) {
                    timer.reset();
                    levels = ArmState.MIDDLE;
                }
                if (y2) {
                    timer.reset();
                    levels = ArmState.UPPER;
                }
                break;
            case LOWER:
                if (timer.milliseconds() < lower) {
                    arm.setPower(.8);
                    arm2.setPower(.8);
                } else {
                    arm.setPower(0);
                    arm2.setPower(0);
                }
                if (x2) {
                    levels = ArmState.RESET;
                }
                break;
            case MIDDLE:
                if (timer.milliseconds() < middle) {
                    arm.setPower(.8);
                    arm2.setPower(.8);
                } else {
                    arm.setPower(0);
                    arm2.setPower(0);
                }
                if (x2) {
                    levels = ArmState.RESET;
                }
                break;
            case UPPER:
                if (timer.milliseconds() < upper) {
                    arm.setPower(.8);
                    arm2.setPower(.8);
                } else {
                    arm.setPower(0);
                    arm2.setPower(0);
                }
                if (x2) {
                    levels = ArmState.RESET;
                }
                break;

                /*
            case RESET:
                if (!touch.isPressed()) {
                    arm.setPower(-.8);
                    arm2.setPower(-.8);
                } else {
                    levels = ArmState.BOTTOM;
                }
                break;

                 */

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

        leftFront.setPower(fl + rightx1);
        leftBack.setPower(bl + rightx1);
        rightFront.setPower(fr - rightx1);
        rightBack.setPower(br - rightx1);


        if (Math.abs(lefty2) >= .1) {
            if (lefty2 < 0) {
                arm.setPower(lefty2 * pow);
                arm2.setPower(lefty2 * pow);
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
        telemetry.addData("arm2",arm2.getPower());

        /*
        if (Math.abs(righty2) >= .1) {
            intake.setPower(-righty2);
        } else {
            intake.setPower(0);
        }
        */


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

        /*
        // move color servos
        if (x1) {
            colorServo.setPosition(.55);
        } else if (y1) {
            colorServo.setPosition(0);
        }
        */

        /*
        ducky.setPower(rightx2*.65);
         */

        // Ensures Data Updates
        telemetry.update();
    }
    @Override
    public void stop() {
    }

}