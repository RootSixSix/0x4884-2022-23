package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


public class VisionBrain {
    OpMode opmode;
    private Telemetry telemetry = null;
    boolean useWebCam = true;
    boolean showCamera = false;
    boolean showCameraOD = false;
    float zoom = 0.8f;
    int returnvalue = 0;

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod = null;

    final String VUFORIA_KEY = "AY7lK0j/////AAABmffl0hEQlUFfjdc9h8Aw+t5/CrgiSiIgNkZKZcw3qdOlnNEv3HarcW4e1pfYY5Nq+4XVrrnhKKNBeR/S08U41ogd0NpmWwOPgttli7io4p8WtbgWj+c/WL9uDzZK9u03K3Kfx+XFxdk/vy0tnFKCPg5w9M5iy7QQP2SDHFDJuhcAOtsayV8n8hQvB528RDRDykBtXei/V6xhN/qLc+S1Gp7eS0ZzpDFnT+uED0CwYK+oaWKNsPPv+3u9tCwofQ5PaRHlN05kH4V97Nn0N7WquSmDpcCZpAVqI1QnMEi7Fm9rvJgET+4OIlx4ZueF3ZTuXtJJSaEJ8Y6CEy9F7FS0RnlVtt4QlqpQVSmWmJQWYBNu";

    private static final String[] LABELS = {
            "Panel",
            "Bulb",
            "Bolt"
    };

    public void init(OpMode theopmode, Telemetry t) {
        telemetry = t;
        opmode = theopmode;


        initVuforia();
        initTfod();
    }
    public void activate() {

        if (tfod != null) {
            tfod.activate();


            tfod.setZoom(zoom, 16.0 / 9.0);
        }

        opmode.telemetry.addData("Status", "Vision Activated");
        opmode.telemetry.update();

    }
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        if (showCamera) {
            int cameraMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        }
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        if (useWebCam)
            parameters.cameraName = opmode.hardwareMap.get(WebcamName.class, "Webcam 1");
        else // else assume phone back camera
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        //    FtcDashboard.getInstance().startCameraStream(vuforia, 0);


        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public void initTfod() {
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        if (showCameraOD) {
            int tfodMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        }
        tfodParameters.minResultConfidence = 0.2f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        //    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public void process(double timeout) {
        opmode.telemetry.addData("Status", "Processing!");

        if (tfod != null) {
            ElapsedTime timer = new ElapsedTime();

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = updatedRecognitions = tfod.getUpdatedRecognitions();

            while (updatedRecognitions == null && timer.seconds() < timeout) {
                try {
                    sleep(100);
                } catch (InterruptedException e) {
                }
                updatedRecognitions = tfod.getUpdatedRecognitions();
            }

            if (updatedRecognitions != null) {
                opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    i++;
                    opmode.telemetry.addLine()
                            .addData(String.format("label (%d)", i), recognition.getLabel())
                            .addData("Conf", "%.02f", recognition.getConfidence())
                            .addData("Loc", "(%.01f,%.01f,%.01f,%.01f)", recognition.getLeft(), recognition.getTop(), recognition.getRight(), recognition.getBottom());
                }
            } else opmode.telemetry.addData("Status", "Recognitions is NULL");
        } else opmode.telemetry.addData("Status", "TFOD is NULL");

        opmode.telemetry.update();
    }


}
