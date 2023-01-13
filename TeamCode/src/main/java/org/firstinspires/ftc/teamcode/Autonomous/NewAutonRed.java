
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name = "NewAutonRed", group="Iterative OpMode")
    public class NewAutonRed extends LinearOpMode {

        // declaration
        private DcMotor leftFront;
        private DcMotor rightFront;
        private DcMotor leftBack;
        private DcMotor rightBack;
        private TouchSensor armTouch;
        private TouchSensor intakeTouch;
        private TouchSensor straightenRight;
        private TouchSensor straitenLeft;
        private DistanceSensor armHeight;
        private CRServo rightintake;
        private CRServo leftintake;
        private CRServo rightTurnSensor;
        private CRServo leftTurnSensor;
        private DcMotor arm;
        private DcMotor arm2;
        // wheelies!

        // color sensors
        private ColorSensor colorBack;
        private ColorSensor colorLeft;
        private ColorSensor colorRight;

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

        private int signal;
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
        double tiles = 2600;
        //time it takes to stafe a tile
        double low = 13.5;
        double med = 23.5;
        double high = 33.5;

        int targetColor = 250; // this is minimum magnitude of color

        @Override
        public void runOpMode() throws InterruptedException {
            leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // in config --> port 1 --> "leftBack"
            rightBack = hardwareMap.get(DcMotor.class, "rightBack"); // in config --> port 2 --> "rightBack
            leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // in config --> port 0 --> "leftFront"
            rightFront = hardwareMap.get(DcMotor.class, "rightFront"); // in config --> port 3 --> "rightFront"
            intakeTouch = hardwareMap.get(TouchSensor.class, "intakeTouch");
            armTouch = hardwareMap.get(TouchSensor.class, "armTouch");  // in config --> digital port 5 --> "touchy"
            rightintake = hardwareMap.get(CRServo.class, "rightClaw"); // in config --> port 3 --> "rightintake"
            leftintake = hardwareMap.get(CRServo.class, "leftClaw"); // in config --> port 4 --> "leftintake"
            straightenRight = hardwareMap.get(TouchSensor.class, "straitenRight");
            straitenLeft = hardwareMap.get(TouchSensor.class, "straightenLeft");
            rightTurnSensor= hardwareMap.get(CRServo.class, "rightTurnSensor");
            leftTurnSensor = hardwareMap.get(CRServo.class, "leftTurnSensor");
            arm = hardwareMap.get(DcMotor.class, "arm");
            arm2 = hardwareMap.get(DcMotor.class, "arm2");
            armHeight = hardwareMap.get(DistanceSensor.class, "armHeight");

            webcamName1 = hardwareMap.get(WebcamName.class, "Webcam 2");
            webcamName2 = hardwareMap.get(WebcamName.class, "Webcam 1");

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
                }
                else {
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
                            } else {
                                signal = 3;
                                // do 3 because it dont like reading circle
                            }
                        }


                        telemetry.addData("signal", signal);
                        telemetry.update();
                        // update telemetry.(aka update what it says in the console on phone!)
                    }
                }
                waitForStart();

                // lower arm if not there
                lower();

                // preload
                while(!intakeTouch.isPressed()) {
                    // intake
                    intake(pow, 10);
                }

                // drive backwards until reaching terminal
                while (colorBack.red() < targetColor) {
                    drive(-pow, -pow, -pow, -pow, 500);
                }

                // now strafe until at line with cones on it.
                drive(STPL * pow, STPL * -pow, STPL * -pow, STPL * pow, tiles*3.75);

                // raise arm
                lift(high);

                //drop cone
                intake(-fullPow, dropTime);

                //lower arm
                lower();

                //set loop counter to 0
                loopCounter=0;

                //start loop
                while(loopCounter!=2){
                    //strafe to line up with cone stack
                    //overshoot
                    drive(-pow*STPL, pow*STPL, -pow*STPL, pow*STPL, tiles*0.4);

                    //touch sensor down
                    turnSensor(-fullPow);

                    //raise arm
                    lift(15);

                    //aproach stack
                    drive(pow, pow, pow, pow, tilef*2);

                    //line up with line
                    while(colorLeft.red() < targetColor && colorRight.red() < targetColor) {
                        if (colorLeft.red() < targetColor && colorRight.red() > targetColor) {
                            drive(lowPow * STPL, -lowPow * STPL, lowPow * STPL, -lowPow * STPL, 100);
                        } else if (colorLeft.red() > targetColor && colorRight.red() < targetColor) {
                            drive(-lowPow * STPL, lowPow * STPL, -lowPow * STPL, lowPow * STPL, 100);
                        }
                        else{
                            drive(lowPow * STPL, -lowPow * STPL, lowPow * STPL, -lowPow * STPL, 100);
                        }
                    }

                    //straiten robot
                    while(!straitenLeft.isPressed()&&!straightenRight.isPressed()) {
                        if (!straitenLeft.isPressed() && !straightenRight.isPressed()) {
                            //both not pressed drive forward
                            drive(pow, pow, pow, pow, 1000);
                        }
                        else if (!straitenLeft.isPressed() && straightenRight.isPressed()){
                            //right pressed power left wheeles
                            drive(lowPow, 0, lowPow, 0, 1000);
                        }
                        else if (straitenLeft.isPressed() && !straightenRight.isPressed()){
                            //left pressed power left wheeles
                            drive(0, lowPow, 0, lowPow, 1000);
                        }
                    }

                    //lower arm to hit cone
                    arm.setPower(lowPow);
                    arm2.setPower(lowPow);
                    if (loopCounter==0){
                        sleep(500);
                    }
                    else if( loopCounter==1){
                        sleep(1000);
                    }
                    arm.setPower(0);
                    arm2.setPower(0);

                    //intake
                    intake(fullPow, dropTime);

                    //raise arm pole height
                    lift(high);

                    //back up
                    drive(-pow, -pow, -pow, -pow,tiles*2);

                    //turn sensor up
                    turnSensor(fullPow);

                    //line up with pole
                    drive(pow*STPL, -pow*STPL, pow*STPL, -pow*STPL, tiles*0.7);

                    //move closer to pole
                    drive(pow, pow, pow, pow, 50);

                    //drop cone
                    intake(fullPow, dropTime);

                    //back away from pole
                    drive(-pow, -pow, -pow, -pow, 50);

                    //lower arm
                    lower();

                    //update loop counter
                    sum=loopCounter;
                }

                if ((signal==1 && STPL==-1)||(signal==3 &&STPL==1)){
                    drive(-pow*STPL, pow*STPL, -pow*STPL, pow*STPL, tiles*0.5);
                    drive(pow, pow, pow, pow, tilef*2);
                    drive(pow*STPL, -pow*STPL, pow*STPL, -pow*STPL, tiles*0.5);
                }
                else if (signal==2){
                    drive(-pow*STPL, pow*STPL, -pow*STPL, pow*STPL, tiles*0.5);
                    drive(pow, pow, pow, pow, tilef*1);
                    drive(pow*STPL, -pow*STPL, pow*STPL, -pow*STPL, tiles*0.5);
                }
                else{
                }

                }
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



    public void intake(double IP, double time) {
        rightintake.setPower (IP);
        leftintake.setPower (IP);
        sleep ((int) time);
        rightintake.setPower(0);
        leftintake.setPower(0);
        sleep (10);
    }

    public void turnSensor(double TP) {
        rightintake.setPower (TP);
        leftintake.setPower (TP);
        sleep ((int) 10);
        rightintake.setPower(0);
        leftintake.setPower(0);
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
                arm.setPower(0.3);
                arm2.setPower(0.3);
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
    }