package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "autonforsmallrobotwithencoders", group="Iterative OpMode")
public class autonforsmallrobotwithencoders extends LinearOpMode {

    // declaration
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private TouchSensor armTouch;
    private TouchSensor intakeTouch;
    private Servo camera;
    private CRServo rightintake;
    private CRServo leftintake;
    private DcMotor arm;
    private DcMotor arm2;
    // wheelies!
    private CRServo leftSpin;
    // left on robot looking from the back
    private CRServo rightSpin;
    // right from the back perspective

    private WebcamName webcamName; // webcam
    private WebcamName webcamName1;
    private WebcamName webcamName2;

    private final static int REVERSE = -1;
    private final static double POWER = 0.3;
    private int FORWARD;
    private int STRAFE;
    private int leftBackPos;
    private int leftFrontPos;
    private int rightBackPos;
    private int rightFrontPos;
    private int low = 1800;
    // time it tales for arm at pow 0.3 to raise to height of low pole
    private int STPL;
    //starting place
    private int dropTime = 5000;
    //time it takes to release cone definitetly way extra but if something goes wrong we need ti to go longer so.
    private int tilef = 56;
    //ticks it takes to go forward or backwards a tile
    private int tiles = 112;
    //need to change
    //ticks it takes to stafe a tile

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
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    double pow= 0.3;
    //power
    double intakePow= 1;
    //power for the intake

    @Override
    public void runOpMode() throws InterruptedException {
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack"); // in config --> port 1 --> "leftBack"
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); // in config --> port 2 --> "rightBack
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // in config --> port 0 --> "leftFront"
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); // in config --> port 3 --> "rightFront"
        armTouch = hardwareMap.get(TouchSensor.class, "armTouch");  // in config --> digital port 5 --> "touchy"
        intakeTouch = hardwareMap.get(TouchSensor.class, "intakeTouch");  // in config --> digital port 5 --> "touchy"
        rightSpin = hardwareMap.get(CRServo.class, "rightSpin"); // in config --> port 3 --> "rightintake"
        leftSpin = hardwareMap.get(CRServo.class, "leftSpin"); // in config --> port 4 --> "leftintake"
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");

        webcamName1 = hardwareMap.get(WebcamName.class, "Webcam 2");
        webcamName2 = hardwareMap.get(WebcamName.class, "Webcam 1");

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (armTouch.isPressed()) {
            // A2 F5
            STPL=1;
            //use camera 1
            webcamName = webcamName1;
        }
        else {
            // A5 F2
            STPL=-1;
            //use camera 2
            webcamName = webcamName2;
        }
        telemetry.addData("Status", "Initialized");

        rightSpin.setDirection(CRServo.Direction.REVERSE); // reversed so servos move opposite ways to pull in / out
        arm.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates and makes it go the correct way
        arm2.setDirection(DcMotorSimple.Direction.REVERSE); // motor is backwards on robot, this compensates

        telemetry.addData("ABBY AND ALLIE LISTEN UP", "blue corners, press button.  red corners, don't press button");


        initVuforia(); // initialize vuforia first
        initTFOD(); // then initialize tensor flow.  This is because vuforia is used to feed the images into tensor flow, meaning it needs to be connected first

        if (tfod != null) {
            // aka the tensor flow has been initialized successfully.

            tfod.activate();
            // turn the tensorflow on so it starts reading.

            tfod.setZoom(1.0, 16.0/9.0);
            // magnification must be at least 1.0
            // zooms into what tensor flow is seeing to mimic zooming with camera.  Makes everything more readable.
        }

        while (!opModeIsActive() && !isStopRequested()) {
            // while we are still running (time hasn't run out!)

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

                        signal=3;
                        /*
                        we can remove this if it makes it do the 3 all the time again but it was
                        coming up as signal 0 when it wasnt reading anything i dont know why the
                        else we added didn't work so i put this back
                         */
                        if (recognition.getLabel().equals("star")){
                            // do the first position stuff
                            signal = 1;
                        }
                        else if (recognition.getLabel().equals("triangle")) {
                            // do the second position stuff
                            signal = 2;
                        }
                        else if (recognition.getLabel().equals("circle")){
                            // this means we've seen the third thing (triangle) and should do that stuff
                            // 3 circle
                            signal = 3;
                        }
                        else {
                            signal = 3;
                            // do 3 because it dont like reading circle
                        }
                    }


                    telemetry.addData("signal", signal);
                    telemetry.update();
                    // update telemetry.(aka update what it says in the console on phone!)
                }
            }
        }

        telemetry.addData("signal", signal);

        leftBackPos = 0;
        leftFrontPos = 0;
        rightFrontPos = 0;
        rightBackPos = 0;

        waitForStart();

        while (!armTouch.isPressed()) {
            lift( -pow, 10);
        }
        //arm goes down until arm button is pressed
        while (!intakeTouch.isPressed()){
            intake(intakePow, 10);
        }
        //intake spins until cone is picked up
        if (STPL==-1){
            drive(30, -30,30, -30, pow);
            //turn so drift is acounted
            //change value
        }
        else {
        }
        drive(-tilef,-tilef,-tilef,-tilef,pow);
        //back up to be in line with park zone
        drive(tiles*STPL, -tiles*STPL, -tiles*STPL, tiles*STPL, pow);
        //strafe to line up with low pole
        lift(pow, low);
        //raise arm
        drive(4, 4, 4, 4, pow);
        //get closer to pole
        //change value
        intake(-intakePow, dropTime);
        //drop cone
        drive(-4,-4,-4,-4, pow);
        //back away from pole
        while (!armTouch.isPressed()){
            lift(-pow, 10);
        }
        //lower arm

        if ((signal==1 && STPL==1) || (signal==3 && STPL==-1)) {
            drive(tiles*STPL, -tiles*STPL, -tiles*STPL, tiles*STPL, pow);
            //move in line with gap
            drive(tilef*2,tilef*2,tilef*2,tilef*2,pow);
            //get in park zone
            drive(-tiles*STPL, tiles*STPL, tiles*STPL, -tiles*STPL, pow);
            //move in middle of park zone
        }
        else if (signal==2){
            drive(tiles*STPL, -tiles*STPL, -tiles*STPL, tiles*STPL, pow);
            //move in line with gap
            drive(tilef,tilef,tilef,tilef,pow);
            //get in park zone
            drive(-tiles*STPL, tiles*STPL, tiles*STPL, -tiles*STPL, pow);
            //move in middle of park zone
        }



    }

    public void intake(double ip, double time) {
        leftSpin.setPower (ip);
        rightSpin.setPower (ip);
        sleep ((int) time);
        leftSpin.setPower(0);
        rightSpin.setPower(0);
        sleep (10);
    }

    public void lift(double ap, double time) {
        arm.setPower(ap);
        arm2.setPower(ap);
        sleep ((int) time);
        arm.setPower(0);
        arm2.setPower(0);
        sleep (10);
    }

    public void drive(double leftFrontTarget, double rightFrontTarget, double leftBackTarget, double rightBackTarget, double speed) {
        leftFrontPos += leftFrontTarget;
        rightFrontPos += rightFrontTarget;
        leftBackPos += leftBackTarget;
        rightBackPos += rightBackTarget;
        leftFront.setTargetPosition(leftFrontPos);
        rightFront.setTargetPosition(rightFrontPos);
        leftBack.setTargetPosition(leftBackPos);
        rightBack.setTargetPosition(rightBackPos);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            idle();
        }
        sleep (10);
    }

    private void initVuforia() {
        // this will initialize vuforia.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        // creates a parameter object to collect necessary paramters and set to our vuforia localizer.

        parameters.vuforiaLicenseKey = VUFORIA_KEY; // sets the vuforia key, which gives us access

        parameters.cameraName = webcamName;
        // sets up a camera that will be used with this program

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // makes vuforia object with said paramters
    }

    private void initTFOD() {
        // this will initialize tensor flow lite

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId","id",hardwareMap.appContext.getPackageName());
        // completely honest, not sure what this does, but I think it gets everything from the SDK to run this stuff

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        // creates the parameters with the default settings

        tfodParameters.minResultConfidence = 0.7f; // this is how sure the computer has to be to say somethhing is what it is
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        // creates the tensor flow, but also links it with vuforia

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS); // loads the objects that can be detected.
    }
}