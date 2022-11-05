package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name = "Earlyauton", group="Iterative OpMode")
public class Earlyauton extends LinearOpMode {

    // declaration
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private TouchSensor touchy;
    private CRServo camera;
    private CRServo rightintake;
    private CRServo leftintake;
    // wheelies!

    @Override
    public void runOpMode() throws InterruptedException {
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack"); // in config --> port 1 --> "leftBack"
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); // in config --> port 2 --> "rightBack
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // in config --> port 0 --> "leftFront"
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); // in config --> port 3 --> "rightFront"
        touchy = hardwareMap.get(TouchSensor.class, "touchy");  // in config --> digital port 5 --> "touchy"
        camera = hardwareMap.get(CRServo.class, "camera"); // in config --> webcam1 --> camera
        rightintake = hardwareMap.get(CRServo.class, "rightspin"); // in config --> port 3 --> "rightintake"
        leftintake = hardwareMap.get(CRServo.class, "leftspin"); // in config --> port 4 --> "leftintake"
        telemetry.addData("Status", "Initialized");

        // stuff in init

        waitForStart();
        if (touchy.isPressed()) {
            camera
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
}
