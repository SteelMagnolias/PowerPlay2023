
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
// hiiiiiiiiiiiii
@Autonomous(name = "NewAutonMultiColored", group="Iterative OpMode")
public class NewAutonMultiColored extends LinearOpMode {

    // wheels
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // touch sensor for arm and intake to tell when something is fully in / down
    private TouchSensor armTouch;
    private TouchSensor intakeTouch;

    private TouchSensor straightenRight;    // touch sensors that touch the wall and get us straight

    private TouchSensor straightenLeft;

    // distance sensor on arm
    private DistanceSensor armHeight;

    // intake servos (continuous)
    private CRServo rightspin;
    private CRServo leftspin;

    // arm motors
    private DcMotor arm;
    private DcMotor arm2;

    // color sensors
    private ColorSensor colorPinkSide;
    private ColorSensor colorYellowSide;
    private ColorSensor colorLeft;
    private ColorSensor colorRight;

    // cameras (each go into webcam name for universal coding) OOP is cool my guys!
    private WebcamName webcamName; // webcam
    private WebcamName webcamName1;
    private WebcamName webcamName2;

    private final static int REVERSE = -1;
    private final static double POWER = 0.3;
    private int FORWARD;
    private int STRAFE;

    private int STPL;
    //starting place
    private int dropTime = 5000;
    //time it takes to drop cone
    private int loopCounter;
    private int sum = (loopCounter+1);

    private static final String TFOD_MODEL_ASSET = "PowerPlayCustomV2.tflite";
    // this is where we can find the preset models

    private static final String[] LABELS = {
            "circle",
            "star",
            "triangle"
    };
    // these are labels that can be used to define what items might be seen.


    private static final String VUFORIA_KEY =
            "Ae/tYNP/////AAABmWJ3jgvBrksYtYG8QcdbeqRWGQWezSnxje7FgEIzwTeFQ1hZ42y6YmaQ0h5p7aqN9x+q1QXf2zRRrh1Pxln3C2cR+ul6r9mHwHbTRgd3jyggk8tzc/ubgaPBdn1q+ufcYqCk6tqj7t8JNYM/UHLZjtpSQrr5RNVs227kQwBoOx6l4MLqWL7TCTnE2vUjgrHaEW1sP1hBsyf1D4SiyRl/Ab1Vksqkgv7hwR1c7J4+7+Nt3rDd16Fr2XToT87t0JlfOn6vszaPj10qvU7836U+/rx9cs1w53UPEdfF+AmDChhdW2TymZf+aS2QfnckyxdXKHjXUhdDw3f09BegsNdnVxXnvGkp0jhg9N7fjJa39k+8";


    private VuforiaLocalizer vuforia;
    // this will later allow you to initialize vuforia. THIS is a particular instance of our vuforia engine

    private TFObjectDetector tfod;
    // this will later allow you to use TensorFlow.  This is a particular instance of the TensorFlow engine.

    //this should be the hardest to detect
    private int signal=2;

  /*
  Vuforia will feed its information and pictures it finds into TensorFlow for further analysis!
   */

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;

    double pow = 0.3;
    //power
    double fullPow = 1;
    //power for the intake
    double lowPow = 0.1;
    double tilef = 1400;
    //time it takes to go forward or backwards a tile
    double tiles = 2100;
    //time it takes to stafe a tile
    double low = 19.5;
    double med = 29.5;
    double high = 37.5;
    //height for poles

    String allianceColor = "blue"; // used to determine which alliance we are on

    int targetColor = 100; // this is minimum magnitude of color

    @Override
    public void runOpMode() throws InterruptedException {
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

        rightspin.setDirection(CRServo.Direction.REVERSE); // reversed so servos move opposite ways to pull in / out

        rightBack.setDirection(DcMotor.Direction.REVERSE);

        if (armTouch.isPressed()) {
            // A2 F5
            STPL = 1;
            //use camera 1
            webcamName = webcamName1;
        } else {
            // A5 F2
            STPL = -1;
            //use camera 2
            webcamName = webcamName2;
        }
        telemetry.addData("Status", "Initialized");

        arm.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates and makes it go the correct way
        arm2.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates

        telemetry.addData("ABBY AND ALLIE LISTEN UP", "blue corners, press button.  red corners, don't press button");


        initVuforia(); // initialize vuforia first
        initTFOD(); // then initialize tensor flow.  This is because vuforia is used to feed the images into tensor flow, meaning it needs to be connected first

        if (tfod != null) {
            // aka the tensor flow has been initialized successfully.

            tfod.activate();
            // turn the tensorflow on so it starts reading.

            tfod.setZoom(1.0, 16.0 / 9.0);
            // magnification must be at least 1.0
            // zooms into what tensor flow is seeing to mimic zooming with camera.  Makes everything more readable.
        }

        while (!opModeIsActive() && !isStopRequested()) {
            // while we are still running (time hasn't run out!)

            if (armTouch.isPressed()) {
                // if the arm button is pressed, we are on the right side of alliance
                STPL = 1;
            } else {
                // if the arm button is not pressed, we are on the left side of the alliance
                STPL = -1;
            }

            if (tfod != null) {
                // tensor flow is still running.

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                // curates a list of things that the camera recognized that is new!  Does not return if it is the same.  When it does not see anything new, it becomes NULL.
                // updatedRecongitions holds what was found.

                if (updatedRecognitions != null) {
                    // something is found

                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // says how many is found.

                    for (Recognition recognition : updatedRecognitions) {
                        // for each recogniton in updated recognitions (that's what the colon means!  You learn something new everyday :D)

                        telemetry.addData("Object", recognition.getLabel());
                        // gets what the recognized object is.

                        telemetry.addData("left", recognition.getLeft());
                        // get what's in the left

                        telemetry.addData("top", recognition.getTop());
                        // get what's in the top

                        telemetry.addData("right", recognition.getRight());
                        // get what's in the right

                        telemetry.addData("bottom", recognition.getBottom());
                        // get what's in the bottom

                        if (recognition.getLabel().equals("star")) {
                            // do the first position stuff
                            signal = 1;
                        } else if (recognition.getLabel().equals("triangle")) {
                            // do the second position stuff
                            signal = 2;
                        } else if (recognition.getLabel().equals("circle")) {
                            // this means we've seen the third thing (triangle) and should do that stuff
                            // 3 circle
                            signal = 3;
                        }
                    }


                    telemetry.addData("signal", signal);
                    telemetry.update();
                    // update telemetry.(aka update what it says in the console on phone!)
                }
            }
        }
            waitForStart();

            // preload
            intake();

            // drive backwards until reaching terminal
            if(STPL==1) {
                while (colorRight.blue() < targetColor && colorRight.red() < targetColor) {
                    backwards(75);
                    //posible choppy motion
                }

                // let's determine our alliance
                if (colorRight.blue() >= targetColor) {
                    // we are the blue alliance
                    allianceColor = "blue";
                }
                else if (colorRight.red() >= targetColor) {
                    // we are on the red alliance
                    allianceColor = "red";
                }

                    drive(0.1, -0.1, 0.1, -0.1, 750);
            }
            else if (STPL==-1) {
                while (colorLeft.blue() < targetColor && colorLeft.red() < targetColor) {
                    backwards(75);
                }

                // let's determine our alliance
                if (colorLeft.blue() >= targetColor) {
                    // we are the blue alliance
                    allianceColor = "blue";
                } else if (colorLeft.red() >= targetColor) {
                    // we are on the red alliance
                    allianceColor = "red";
                }

                    drive(-0.1, 0.1, -0.1, 0.1, 750);

            }

            // now strafe until in line with  high pole
            left(tiles*1);

            if (signal == 2) {
                forward(120);
            }
            else {
                forward(150);
            }

            left(tiles*1);

            if (STPL == 1) {
                drive(0.1, -0.1, 0.1, -0.1, 750);
            }
            else if (STPL == -1){
                drive(-0.1, 0.1, -0.1, 0.1, 750);
            }
            /*

            // raise arm
            lift(high);

            //get closer to pole
            forward(500);

            //drop cone
            outtake();

            //back away from pole
            backwards(500);

            //lower arm
            lower();

            //set loop counter to 1
            loopCounter=1;

            //start loop
            for (int i = 0; i < loopCounter; i++){
                //strafe to line up with cone stack
                //undershoot
                right(tiles*0.4);

                //raise arm
                lift(15);

                //aproach stack
                forward(tilef*2);

                //line up with line
                while(getColor(colorLeft, allianceColor) < targetColor || getColor(colorRight, allianceColor) < targetColor) {
                    if (getColor(colorLeft, allianceColor) < targetColor && getColor(colorRight, allianceColor) >= targetColor) {
                        drive(lowPow * STPL, -lowPow * STPL, lowPow * STPL, -lowPow * STPL, 100);
                    } else if (getColor(colorLeft, allianceColor) >= targetColor && getColor(colorRight, allianceColor) < targetColor) {
                        drive(-lowPow * STPL, lowPow * STPL, -lowPow * STPL, lowPow * STPL, 100);
                    }
                    else{
                        drive(lowPow * STPL, -lowPow * STPL, lowPow * STPL, -lowPow * STPL, 100);
                    }
                }

                //straighten robot
                while(!straightenLeft.isPressed() || !straightenRight.isPressed()) {
                    if (!straightenLeft.isPressed() && !straightenRight.isPressed()) {
                        //both not pressed drive forward
                        drive(lowPow, lowPow, lowPow, lowPow, 1000);
                    }
                    else if (!straightenLeft.isPressed() && straightenRight.isPressed()){
                        //right pressed power left wheeles
                        drive(lowPow, 0, lowPow, 0, 1000);
                    }
                    else if (straightenLeft.isPressed() && !straightenRight.isPressed()){
                        //left pressed power right wheeles
                        drive(0, lowPow, 0, lowPow, 1000);
                    }
                }

                //lower and intake
                arm.setPower(-lowPow);
                arm2.setPower(-lowPow);
                leftspin.setPower(fullPow);
                rightspin.setPower(fullPow);
                while (!intakeTouch.isPressed()){
                    sleep(1);
                }
                leftspin.setPower(0);
                rightspin.setPower(0);
                arm.setPower(0);
                arm2.setPower(0);

                //back up
                backwards(tiles*2);

                //line up with pole
                left(tiles*0.5);

                //raise arm pole height
                lift(high);

                //move closer to pole
                forward(50);

                //drop cone
                outtake();

                //back away from pole
                backwards(50);

                //lower arm
                lower();
            }


            //move to middle of park zone
            right(tiles*2);
            */
            if ((signal==1 && STPL==-1)||(signal==3 &&STPL==1)){
                //zone closest to wall
                left(tiles*0.1);
                forward(tilef*2.25);
                right(tiles*0.1);
            }
            else if (signal==2){
                //middle zone
                left(tiles*0.1);
                forward(tilef*1.25);
                right(tiles*0.25);
            }


            telemetry.addData("Color Sensor colorLeft Red:", colorLeft.red());
            telemetry.addData("Color Sensor colorLeft Blue:", colorLeft.blue());
            telemetry.addData("Color Sensor colorLeft Green: ", colorLeft.green());

            telemetry.addData("Color Sensor colorRight Red:", colorRight.red());
            telemetry.addData("Color Sensor colorRight Blue:", colorRight.blue());
            telemetry.addData("Color Sensor colorRight Green: ", colorRight.green());
            //do we need green here??

            telemetry.addData("armHeight (inches)", armHeight.getDistance(DistanceUnit.INCH));

            telemetry.addData("straightenLeft", straightenLeft.isPressed());
            telemetry.addData("straightenRight", straightenRight.isPressed());

            telemetry.addData("armTouch", armTouch.isPressed());
            telemetry.addData("intakeTouch", intakeTouch.isPressed());
            telemetry.update();

    }


    private void initVuforia () {
        // this will initialize vuforia.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        // creates a parameter object to collect necessary paramters and set to our vuforia localizer.

        parameters.vuforiaLicenseKey = VUFORIA_KEY; // sets the vuforia key, which gives us access

        parameters.cameraName = webcamName;
        // sets up a camera that will be used with this program

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // makes vuforia object with said paramters

    }

    private void initTFOD () {
        // this will initialize tensor flow lite

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // completely honest, not sure what this does, but I think it gets everything from the SDK to run this stuff

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        // creates the parameters with the default settings

        tfodParameters.minResultConfidence = 0.7f; // this is how sure the computer has to be to say somethhing is what it is
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        // creates the tensor flow, but also links it with vuforia

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS); // loads the objects that can be detected.;
    }

    public void outtake() {
        rightspin.setPower (-fullPow);
        leftspin.setPower (-fullPow);
        sleep (dropTime);
        rightspin.setPower(0);
        leftspin.setPower(0);
        sleep (10);
    }

    public void intake() {
        while(!intakeTouch.isPressed()) {
            rightspin.setPower(fullPow);
            leftspin.setPower(fullPow);
        }
        rightspin.setPower(0);
        leftspin.setPower(0);
    }

    public void cone(double IP, double time) {
        rightspin.setPower (IP);
        leftspin.setPower (IP);
        sleep ((int) time);
        rightspin.setPower(0);
        leftspin.setPower(0);
        sleep (10);
    }

    public void lower() {
        while(!armTouch.isPressed()) {
            arm.setPower(-pow);
            arm2.setPower(-pow);
        }
        arm.setPower(0);
        arm2.setPower(0);
        sleep (10);
    }

    public void lift(double AH){
        while (armHeight.getDistance(DistanceUnit.INCH)<AH){
            arm.setPower(0.5);
            arm2.setPower(0.5);
        }
        arm.setPower(0);
        arm2.setPower(0);
        sleep(10);
    }

    public void moveArm(double AH, double pow) {
        // lift, but with an assigned power as well.

        while (armHeight.getDistance(DistanceUnit.INCH)<AH){
            arm.setPower(pow);
            arm2.setPower(pow);
            arm.setPower(-pow);
        }
        arm.setPower(0);
        arm2.setPower(0);
        sleep(10);
    }

    public void drive (double lf, double rf, double lb, double rb, double time){
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        sleep ((int)time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(10);
    }

    public void forward(double time){
        leftFront.setPower(pow);
        rightFront.setPower(pow);
        leftBack.setPower(pow);
        rightBack.setPower(pow);
        sleep ((int)time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(10);
    }

    public void backwards(double time){
        leftFront.setPower(-pow);
        rightFront.setPower(-pow);
        leftBack.setPower(-pow);
        rightBack.setPower(-pow);
        sleep ((int)time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(10);
    }

    public void left(double time){
        leftFront.setPower(-pow*STPL);
        rightFront.setPower(pow*STPL);
        leftBack.setPower(pow*STPL);
        rightBack.setPower(-pow*STPL);
        sleep ((int)time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(10);
    }

    public void right(double time){
        leftFront.setPower(pow*STPL);
        rightFront.setPower(-pow*STPL);
        leftBack.setPower(-pow*STPL);
        rightBack.setPower(pow*STPL);
        sleep ((int)time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(10);
    }

    public int getColor(ColorSensor s, String alliance) {
        // if we are on the blue alliance, look for blue
        if (alliance.equalsIgnoreCase("blue")) {
            return s.blue();
        }
        else {
            // if we are on the red alliance, look for red
            return s.red();
        }
    }
}