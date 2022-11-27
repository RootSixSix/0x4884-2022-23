package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleVision", group = "19380")
public class VisionTester extends OpMode {
    VisionBrain visionBrain;
    boolean loopAction;

    @Override
    public void init() {
        visionBrain = new org.firstinspires.ftc.teamcode.robot.VisionBrain();
        visionBrain.useWebCam = true;
        visionBrain.showCamera = true; // useful for sighting on phone only
        visionBrain.showCameraOD = true; // useful for seeing object detection on phone only
        visionBrain.zoom = 1f;  // 1.0 is no zoom, greater number is greater zoom
        visionBrain.init(this, telemetry);
        visionBrain.activate();
        telemetry.addData("Vision Activation Done", 1);
      //  visionBrain.initTfod();
      //  visionBrain.initVuforia();
    }

    @Override
    public void loop() {
        telemetry.addData("Loop has begun!",loopAction);
        //visionBrain.process(15);
        visionBrain.process2();
    }
    //teleDrive();
}

