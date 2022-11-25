package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "autonforsmallrobot", group="Iterative OpMode")
public class autonforsmallrobot extends LinearOpMode {

    // declaration
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private TouchSensor armTouch;
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

    private final static int REVERSE = -1;
    private final static double POWER = 0.3;
    private int FORWARD;
    private int STRAFE;
    private int ROTATE;
    private int one;
    private int two;
    private int three;
    private int low = 1;
    private int med = 2;
    private int high = 3;
    private int STPL;


    private static final String VUFORIA_KEY =
            "Ae/tYNP/////AAABmWJ3jgvBrksYtYG8QcdbeqRWGQWezSnxje7FgEIzwTeFQ1hZ42y6YmaQ0h5p7aqN9x+q1QXf2zRRrh1Pxln3C2cR+ul6r9mHwHbTRgd3jyggk8tzc/ubgaPBdn1q+ufcYqCk6tqj7t8JNYM/UHLZjtpSQrr5RNVs227kQwBoOx6l4MLqWL7TCTnE2vUjgrHaEW1sP1hBsyf1D4SiyRl/Ab1Vksqkgv7hwR1c7J4+7+Nt3rDd16Fr2XToT87t0JlfOn6vszaPj10qvU7836U+/rx9cs1w53UPEdfF+AmDChhdW2TymZf+aS2QfnckyxdXKHjXUhdDw3f09BegsNdnVxXnvGkp0jhg9N7fjJa39k+8";


    private static int signal;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaTrackables targets   = null ;
    private WebcamName webcamName       = null;

    private boolean targetVisible       = false;
    double pow= 0.3;
    int tilef = 1250;
    double tiles = 1250;

    @Override
    public void runOpMode() throws InterruptedException {
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack"); // in config --> port 1 --> "leftBack"
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); // in config --> port 2 --> "rightBack
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // in config --> port 0 --> "leftFront"
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); // in config --> port 3 --> "rightFront"
        armTouch = hardwareMap.get(TouchSensor.class, "touchy");  // in config --> digital port 5 --> "touchy"
        camera = hardwareMap.get(Servo.class, "camera"); // in config --> webcam1 --> camera
        rightintake = hardwareMap.get(CRServo.class, "rightSpin"); // in config --> port 3 --> "rightintake"
        leftintake = hardwareMap.get(CRServo.class, "leftSpin"); // in config --> port 4 --> "leftintake"
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        telemetry.addData("Status", "Initialized");

        telemetry.addData("ABBY AND ALLIE LISTEN UP", "blue closest to the audience touch button using cone\nblue farthest from audience dont have cone hit button\nred closest to the audience dont hit the button with cone\nred farthest from audience have cone hit button ");

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("StonesAndChips");

        VuforiaTrackable stones = targets.get(0);
        stones.setName("stones");  // Stones

        VuforiaTrackable chips  = targets.get(1);
        chips.setName("chips");  // Chips

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(0, "stones",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "chips",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        // identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        // identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        /*
         * WARNING:
         * In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
         * This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
         * CONSEQUENTLY do not put any driving commands in this loop.
         * To restore the normal opmode structure, just un-comment the following line:
         */

        /* Note: To use the remote camera preview:
         * AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
         * Tap the preview window to receive a fresh image.
         * It is not permitted to transition to RUN while the camera preview window is active.
         * Either press STOP to exit the OpMode, or use the "options menu" again, and select "Camera Stream" to close the preview window.
         */

        targets.activate();
        while (!opModeIsActive() && !isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                for (VuforiaTrackable trackable: allTrackables) {
                    if (trackable.getName().equals("chips")) {
                        signal = 2; // second parking lmao
                    }
                    else if (trackable.getName().equals("stones")) {
                        signal = 3; // third parking f
                    }
                    else {
                        signal = 1; // make signal 1 just in case there was a fluke, it will do something
                    }
                }

            }
            else {
                telemetry.addData("Visible Target", "none");
                signal = 1;
            }
            telemetry.addData("signal", signal);
            telemetry.update();
        }

        waitForStart();

        // Disable Tracking when we are done;
        targets.deactivate();
        //move arm down when button is not pressed

        while (!armTouch.isPressed()){
            arm.setPower(-0.3);
            arm2.setPower(-0.3);
        }
        arm.setPower(0);
        arm2.setPower(0);

        if (signal==3){
            drive (-pow*STPL, -pow*STPL, -pow*STPL, -pow*STPL, tilef);
            if(STPL==1) {
                drive (-pow*STPL, pow*STPL, -pow*STPL, pow*STPL, tiles*1.5);
                arm.setPower (0.3);
                arm2.setPower (0.3);
                sleep (low);
                arm.setPower (0);
                arm2.setPower (0);
                drive (pow, pow, pow, pow, 50);leftSpin.setPower(REVERSE*pow);
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
                drive (-pow*STPL, pow*STPL, -pow*STPL, pow*STPL, tiles*0.5);
                drive (pow, pow, pow, pow, 50);leftSpin.setPower(REVERSE*pow);
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
                drive (-pow*STPL, pow*STPL, -pow*STPL, pow*STPL, tiles);
            }

        }
        if (signal==2){
            drive (-pow*STPL, pow*STPL, -pow*STPL, pow*STPL, tiles*0.5);
            drive (pow, pow, pow, pow, 50);
            arm.setPower (0.3);
            arm2.setPower (0.3);
            sleep (low);
            arm.setPower (0);
            arm2.setPower (0);
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
            drive (-pow*STPL, pow*STPL, -pow*STPL, pow*STPL, tiles);
        }
        if (signal==1){
            drive (pow*STPL, pow*STPL, pow*STPL, pow*STPL, tilef);
            if(STPL==-1) {
                drive (-pow*STPL, pow*STPL, -pow*STPL, pow*STPL, tiles*1.5);
                arm.setPower (0.3);
                arm2.setPower (0.3);
                sleep (low);
                arm.setPower (0);
                arm2.setPower (0);
                drive (pow, pow, pow, pow, 50);leftSpin.setPower(REVERSE*pow);
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
                drive (-pow*STPL, pow*STPL, -pow*STPL, pow*STPL, tiles*0.5);
                drive (pow, pow, pow, pow, 50);leftSpin.setPower(REVERSE*pow);
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
                drive (-pow*STPL, pow*STPL, -pow*STPL, pow*STPL, tiles);
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

    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }}