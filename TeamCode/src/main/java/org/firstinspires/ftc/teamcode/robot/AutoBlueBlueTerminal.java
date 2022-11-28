package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;



@Autonomous(name="AutoBlueAllainceBlueTerminal",group="19380")
public class AutoBlueBlueTerminal extends LinearOpMode {
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    //   private DcMotorEx arm = null;
    //   private Servo leftClaw = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    double driveMultiplicative = 1;
    boolean turboMode = false;
    double leftClawClosePosition = 0.41;
    double leftClawOpenPosition = 0;
    double rightClawClosePosition = 0.439215707789307;
    double rightClawOpenPosition = 0;
    double armConstant = 0.85;
    double clawConstant = 0.8;
    static final double ARM_RESET_POS = 0;
    static final double ARM_GROUND_JUNCTION_POS = 0;
    static final double ARM_LOW_BAR_POS = 0;
    static final double ARM_MIDDLE_BAR_POS = 0;
    static final double ARM_HIGH_BAR_POS = 0;
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    private static final String VUFORIA_KEY = "AY7lK0j/////AAABmffl0hEQlUFfjdc9h8Aw+t5/CrgiSiIgNkZKZcw3qdOlnNEv3HarcW4e1pfYY5Nq+4XVrrnhKKNBeR/S08U41ogd0NpmWwOPgttli7io4p8WtbgWj+c/WL9uDzZK9u03K3Kfx+XFxdk/vy0tnFKCPg5w9M5iy7QQP2SDHFDJuhcAOtsayV8n8hQvB528RDRDykBtXei/V6xhN/qLc+S1Gp7eS0ZzpDFnT+uED0CwYK+oaWKNsPPv+3u9tCwofQ5PaRHlN05kH4V97Nn0N7WquSmDpcCZpAVqI1QnMEi7Fm9rvJgET+4OIlx4ZueF3ZTuXtJJSaEJ8Y6CEy9F7FS0RnlVtt4QlqpQVSmWmJQWYBNu";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    double returnvalue = 0;

    public void runOpMode() throws InterruptedException {
        front_left = hardwareMap.get(DcMotor.class, "fldrive");
        front_right = hardwareMap.get(DcMotor.class, "frdrive");
        back_left = hardwareMap.get(DcMotor.class, "bldrive");
        back_right = hardwareMap.get(DcMotor.class, "brdrive");
        //   arm = hardwareMap.get(DcMotorEx.class, "armdrive");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        leftClaw.setPosition(0);
        rightClaw.setPosition(0);
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            process();

            if(returnvalue == 1){
                ElapsedTime runtime2 = new ElapsedTime();
                while(runtime2.seconds()<0.75){
                    front_right.setPower(0.5);
                    front_left.setPower(-0.5);
                    back_right.setPower(-0.5);
                    back_left.setPower(0.5);
                }
                setDriveStop();
                ElapsedTime runtime3 = new ElapsedTime();
                while(runtime3.seconds()<1.2){
                    front_right.setPower(0.5);
                    front_left.setPower(0.5);
                    back_right.setPower(0.5);
                    back_left.setPower(0.5);
                }
                setDriveStop();
            }

            if(returnvalue == 2){
                ElapsedTime runtime4 = new ElapsedTime();
                while(runtime4.seconds()<1.2){
                    front_left.setPower(0.5);
                    front_right.setPower(0.5);
                    back_left.setPower(0.5);
                    back_right.setPower(0.5);
                }
                setDriveStop();
            }

            if(returnvalue == 3){
                ElapsedTime runtime5 = new ElapsedTime();
                while(runtime5.seconds()<0.75){
                    front_right.setPower(-0.5);
                    front_left.setPower(0.5);
                    back_right.setPower(0.5);
                    back_left.setPower(-0.5);
                }
                setDriveStop();
                ElapsedTime runtime3 = new ElapsedTime();
                while(runtime3.seconds()<1.2){
                    front_right.setPower(0.5);
                    front_left.setPower(0.5);
                    back_right.setPower(0.5);
                    back_left.setPower(0.5);
                }
                setDriveStop();
            }

        /*    ElapsedTime runtime2 = new ElapsedTime();

            while (runtime2.seconds() < 1.5) {

                front_right.setPower(0.5);
                front_left.setPower(-0.5);
                back_right.setPower(-0.5);
                back_left.setPower(0.5);

            }
            front_left.setPower(0);
            front_right.setPower(0);
            back_right.setPower(0);
            back_left.setPower(0);*/
        }


    }

    public double process() {
        ElapsedTime timeout = new ElapsedTime();
        Recognition winner = null;
        while (timeout.seconds()<5) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                        if(winner == null){
                            winner = recognition;
                        }
                        else if(winner.getConfidence()<recognition.getConfidence()){
                            winner = recognition;
                        }
                        else if(winner.getConfidence()> recognition.getConfidence()){
                            winner = winner;
                        }
                        if(winner.getLabel().equals("1 Bolt")){
                            returnvalue = 1;
                        }
                        else if(winner.getLabel().equals("2 Bulb")){
                            returnvalue = 2;
                        }
                        else{
                            returnvalue = 3;
                        }
                        telemetry.addData("Returning",returnvalue);
                        telemetry.addData("Object:",winner.getLabel());
                    }
                    telemetry.update();
                }
            }
        }
        return returnvalue;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    public void setDriveStop() {
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
}