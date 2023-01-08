package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


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

    @Autonomous(name = "NEWautonplan", group="Iterative OpMode")
    public class NEWautonplan extends LinearOpMode {

        // declaration
        private DcMotor leftFront;
        private DcMotor rightFront;
        private DcMotor leftBack;
        private DcMotor rightBack;
        private TouchSensor armTouch;
        private TouchSensor intakeTouch;
        private CRServo rightintake;
        private CRServo leftintake;
        private DcMotor arm;
        private DcMotor arm2;
        // wheelies!

        private WebcamName webcamName; // webcam
        private WebcamName webcamName1;
        private WebcamName webcamName2;

        private final static int REVERSE = -1;
        private final static double POWER = 0.3;
        private int FORWARD;
        private int STRAFE;
        private int low = 1800;
        private int med = 3133;
        private int high = 4466;
        // might need to change
        // time it tales for arm at pow 0.3 to raise to height poles

        private int STPL;
        //starting place
        private int dropTime = 5000;
        //time it takes to drop cone

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
        double tilef = 1400;
        //time it takes to go forward or backwards a tile
        double tiles = 2600;
        //time it takes to stafe a tile

        @Override
        public void runOpMode() throws InterruptedException {
            leftBack  = hardwareMap.get(DcMotor.class, "leftBack"); // in config --> port 1 --> "leftBack"
            rightBack = hardwareMap.get(DcMotor.class, "rightBack"); // in config --> port 2 --> "rightBack
            leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // in config --> port 0 --> "leftFront"
            rightFront = hardwareMap.get(DcMotor.class, "rightFront"); // in config --> port 3 --> "rightFront"
            armTouch = hardwareMap.get(TouchSensor.class, "armTouch");  // in config --> digital port 5 --> "touchy"
            rightintake = hardwareMap.get(CRServo.class, "rightClaw"); // in config --> port 3 --> "rightintake"
            leftintake = hardwareMap.get(CRServo.class, "leftClaw"); // in config --> port 4 --> "leftintake"
            arm = hardwareMap.get(DcMotor.class, "arm");
            arm2 = hardwareMap.get(DcMotor.class, "arm2");

            webcamName1 = hardwareMap.get(WebcamName.class, "Webcam 2");
            webcamName2 = hardwareMap.get(WebcamName.class, "Webcam 1");

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
            waitForStart();
                {


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

                    private void initTFOD {
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
