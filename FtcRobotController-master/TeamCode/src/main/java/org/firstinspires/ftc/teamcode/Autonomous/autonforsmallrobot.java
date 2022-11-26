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

@Autonomous(name = "autonforsmallrobot", group="Iterative OpMode")
public class autonforsmallrobot extends LinearOpMode {

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

    private final static int REVERSE = -1;
    private final static double POWER = 0.3;
    private int FORWARD;
    private int STRAFE;
    private int ROTATE;
    private int one;
    private int two;
    private int three;
    private int low = 2000;
    private int med = 2;
    private int high = 3;
    private int STPL;
    private int dropTime = 5000;

    private static final String TFOD_MODEL_ASSET = "PowerPlayCustom.tflite";
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

    private int signal = -1; // holds the true false value of whether a duck is present.
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
    double intakePow= 1;
    double tilef = 1400;
    double tiles = 2600;

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

            tfod.setZoom(2.0, 16.0/9.0);
            // magnification must be at least 1.0
            // zooms into what tensor flow is seeing to mimic zooming with camera.  Makes everything more readable.
        }

        while (!opModeIsActive() && !isStopRequested()) {
            // while we are still running (time hasn't run out!)

            if (armTouch.isPressed()) {
                // A2 F5
                STPL=1;
                //starting place
                //use camera 1
                webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            }
            else {
                // A5 F2
                STPL=-1;
                //use camera 2
                webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");
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

                        if (recognition.getLabel().equals("star")){
                            // do the first position stuff
                            signal = 1;
                        }
                        else if (recognition.getLabel().equals("triangle")) {
                            // do the second position stuff
                            signal = 2;
                        }
                        else {
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

        telemetry.addData("signal", signal);

        waitForStart();


        while (!armTouch.isPressed()){
            arm.setPower(-0.3);
            arm2.setPower(-0.3);
        }
        arm.setPower(0);
        arm2.setPower(0);

        while (!intakeTouch.isPressed()){
            leftSpin.setPower(intakePow);
            rightSpin.setPower(intakePow);
        }
        leftSpin.setPower(0);
        rightSpin.setPower(0);
        signal = 3; // set this to test.  REMOVE IMBECILES

        if (signal==3){
            drive (-pow*STPL, -pow*STPL, -pow*STPL, -pow*STPL, tilef);
            if(STPL==1) {
                drive (pow*STPL, -pow*STPL, -pow*STPL, pow*STPL, tiles*1.3);
                arm.setPower (0.3);
                arm2.setPower (0.3);
                sleep (low);
                arm.setPower (0);
                arm2.setPower (0);
                drive (pow, pow, pow, pow, 50);
                leftSpin.setPower(intakePow*REVERSE);
                rightSpin.setPower(intakePow*REVERSE);
                //ajust time
                sleep (dropTime);
                leftSpin.setPower(0);
                rightSpin.setPower(0);
                drive (-pow, -pow, -pow, -pow, 50);
                //move arm down when button is not pressed
                while (!armTouch.isPressed()){
                    arm.setPower(-0.3);
                    arm2.setPower(-0.3);
                }
                arm.setPower(0);
                arm2.setPower(0);
            }
            else {
                drive(pow * STPL, -pow * STPL, -pow * STPL, pow * STPL, tiles * 0.5);
                drive(pow, pow, pow, pow, 150);
                leftSpin.setPower(intakePow * REVERSE);
                rightSpin.setPower(intakePow * REVERSE);
                //ajust time
                sleep(dropTime);
                leftSpin.setPower(0);
                rightSpin.setPower(0);
                arm.setPower(0.3);
                arm2.setPower(0.3);
                sleep(low);
                arm.setPower(0);
                arm2.setPower(0);
                drive(-pow, -pow, -pow, -pow, 150);
                //move arm down when button is not pressed
                drive(pow * STPL, -pow * STPL, -pow * STPL, pow * STPL, tiles * 0.9);
                while (!armTouch.isPressed()) {
                    arm.setPower(-0.3);
                    arm2.setPower(-0.3);
                }
                arm.setPower(0);
                arm2.setPower(0);
            }

        }
        if (signal==2){
            sleep (10);
            drive (pow*STPL, -pow*STPL, -pow*STPL, pow*STPL, tiles*0.6);
            arm.setPower (0.3);
            arm2.setPower (0.3);
            sleep (low);
            arm.setPower (0);
            arm2.setPower (0);
            drive (pow, pow, pow, pow, 100);
            leftSpin.setPower(intakePow*REVERSE);
            rightSpin.setPower(intakePow*REVERSE);
            sleep (dropTime);
            leftSpin.setPower(0);
            rightSpin.setPower(0);
            drive (-pow, -pow, -pow, -pow, 100);
            //move arm down when button is not pressed
            while (!armTouch.isPressed()){
                arm.setPower(-0.3);
                arm2.setPower(-0.3);
            }
            arm.setPower(0);
            arm2.setPower(0);
            sleep (10);
            drive (-pow*STPL, pow*STPL, pow*STPL, -pow*STPL, tiles);
            sleep (10);
            drive (pow*STPL, -pow*STPL, pow*STPL, -pow*STPL, 1500);
            sleep (10);
            drive (pow, pow, pow, pow, tilef*1.5);
        }
        if (signal==1){
            drive (pow*STPL, pow*STPL, pow*STPL, pow*STPL, tilef);
            if(STPL==-1) {
                drive (pow*STPL, -pow*STPL, -pow*STPL, pow*STPL, tiles*1.5);
                leftSpin.setPower(intakePow);
                rightSpin.setPower(intakePow);
                //ajust time
                sleep (dropTime);
                leftSpin.setPower(0);
                rightSpin.setPower(0);
                drive (pow, pow, pow, pow, 100);
                leftSpin.setPower(REVERSE*pow);
                rightSpin.setPower(REVERSE*pow);
                //ajust time
                sleep (1000);
                leftSpin.setPower(0);
                rightSpin.setPower(0);
                drive (-pow, -pow, -pow, -pow, 50);
                //move arm down when button is not pressed
                while (!armTouch.isPressed()){
                    arm.setPower(-0.3);
                    arm2.setPower(-0.3);
                }
                arm.setPower(0);
                arm2.setPower(0);
            }
            else {
                drive (pow*STPL, -pow*STPL, -pow*STPL, pow*STPL, tiles*0.5);
                drive (pow, pow, pow, pow, 150);
                leftSpin.setPower(intakePow * REVERSE);
                rightSpin.setPower(intakePow * REVERSE);
                //ajust time
                sleep (dropTime);
                leftSpin.setPower(0);
                rightSpin.setPower(0);
                arm.setPower(0.3);
                arm2.setPower(0.3);
                sleep (low);
                arm.setPower(0);
                arm2.setPower(0);
                drive (-pow, -pow, -pow, -pow, 150);
                //move arm down when button is not pressed
                drive (pow*STPL, -pow*STPL, -pow*STPL, pow*STPL, tiles*0.9);
                while (!armTouch.isPressed()){
                    arm.setPower(-0.3);
                    arm2.setPower(-0.3);
                }
                arm.setPower(0);
                arm2.setPower(0);
            }
        }



    }

    //If button is not pressed camera 1 on the left will activate just like poofff

    //If button is pressed camera 2 on the right will activate yayyyyy

    //If button is not pressed, the multiplier will be 1 which gives us default directions wahooy

    //If button is pressed, the multiplier will be -1  which gives us opposite directions

    //Camera reading

    // ITS GO TIME LEVYYYYY aka start :)


    // If the image is one (continue with multiplier) back into wall, drive left to lower goal.

    //}
    // If image is 2 drive left until reaching mid

    //iF image is 3 drive forward 1 square and drive left until high.

    //End of code woohhoooooooooooo

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

    private void initVuforia() {
        // this will initialize vuforia.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        // creates a parameter object to collect necessary paramters and set to our vuforia localizer.

        parameters.vuforiaLicenseKey = VUFORIA_KEY; // sets the vuforia key, which gives us access

        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 2");
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

