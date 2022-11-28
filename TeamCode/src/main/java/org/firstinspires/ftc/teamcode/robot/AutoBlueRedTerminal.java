package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="AutoBlueAllianceRedTerminal",group = "19380")
public class AutoBlueRedTerminal extends LinearOpMode {
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    public DcMotor[] driveMotors = new DcMotor[4];
    private DcMotor arm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    double driveMultiplicative = 1;
    boolean turboMode = false;
    double leftClawClosePosition = 0.41;
    double leftClawOpenPosition = 0;
    double rightClawClosePosition = 0.439215707789307;
    double rightClawOpenPosition = 0;
    double armMaintPower = 0;
    double armConstant = 0.85;
    double clawConstant = 0.8;
    static final double WHEEL_DIAMETER_INCHES = 2.25;     // For figuring circumference
    static final double COUNTS_PER_OUTPUT_REVOL = 537.7;
    static final double COUNTS_PER_INCH = (22.0 / 16.0) * (COUNTS_PER_OUTPUT_REVOL) / (WHEEL_DIAMETER_INCHES * Math.PI);
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

        front_left   = hardwareMap.get(DcMotor.class, "fldrive");
        front_right  = hardwareMap.get(DcMotor.class, "frdrive");
        back_left    = hardwareMap.get(DcMotor.class, "bldrive");
        back_right   = hardwareMap.get(DcMotor.class, "brdrive");
        arm = hardwareMap.get(DcMotor.class, "armdrive");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        leftClaw.setPosition(0);
        rightClaw.setPosition(0);
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
        initVuforia();
        initTfod();
        waitForStart();
        if(opModeIsActive()){
            process();
            ElapsedTime runtime2 = new ElapsedTime();

            while(runtime2.seconds()<1.5){

                front_right.setPower(-0.5);
                front_left.setPower(0.5);
                back_right.setPower(0.5);
                back_left.setPower(-0.5);

            }
            front_left.setPower(0);
            front_right.setPower(0);
            back_right.setPower(0);
            back_left.setPower(0);
 /*           ElapsedTime runtime2 = new ElapsedTime();
            while(runtime2.seconds()<2){
                front_left.setPower(0.5);
                back_right.setPower(0.5);

                front_right.setPower(-0.5);
                back_left.setPower(-0.5);

                runtime2.reset();
            }
            ElapsedTime runtime3 = new ElapsedTime();
            while(runtime3.seconds()<2){
                front_right.setPower(0.5);
                front_left.setPower(0.5);
                back_left.setPower(0.5);
                back_right.setPower(0.5);

                runtime3.reset();
            }
            ElapsedTime runtime4 = new ElapsedTime();
            while(runtime4.seconds()<0.15){
                front_left.setPower(0.5);
                back_right.setPower(0.5);

                front_right.setPower(-0.5);
                back_left.setPower(-0.5);

                runtime4.reset();
            }

            //arm goes up
            clawOpen();
            //arm goes down
            //reverses the actions
            ElapsedTime runtime5 = new ElapsedTime();
            while(runtime5.seconds()<0.15){
                front_left.setPower(-0.5);
                back_right.setPower(-0.5);

                front_right.setPower(0.5);
                back_left.setPower(0.5);

                runtime5.reset();
            }
            ElapsedTime runtime6 = new ElapsedTime();
            while(runtime6.seconds()<2){
                front_right.setPower(-0.5);
                front_left.setPower(-0.5);
                back_left.setPower(-0.5);
                back_right.setPower(-0.5);

                runtime6.reset();
            }
            ElapsedTime runtime7 = new ElapsedTime();
            while(runtime7.seconds()<0.15){
                front_left.setPower(0.5);
                back_right.setPower(0.5);

                front_right.setPower(-0.5);
                back_left.setPower(-0.5);

                runtime7.reset();
            }
*/


        }
    }
    public int convertInchesToCounts(double inches) {
        return (int) ((COUNTS_PER_INCH) * inches);
    }

    public double convertCountsToInches(int counts) {
        return (double) (counts / (COUNTS_PER_INCH));
    }
    public boolean isDriveBusy(){
        boolean motor = false;
        if(front_left.isBusy()||front_right.isBusy()||back_left.isBusy()||back_right.isBusy()){
            motor = true;
        }
        return motor;
    }

    public void setRunMode(DcMotor.RunMode RunMode) {
        front_left.setMode(RunMode);
        front_right.setMode(RunMode);
        back_left.setMode(RunMode);
        back_right.setMode(RunMode);
    }

    public void setDrive(double forward, double strafe, double rotate, double power) {
        //this function will combine wheel commands

        double[] wheelpowers = {
                (forward + strafe + rotate),
                (forward - strafe - rotate),
                (forward - strafe + rotate),
                (forward + strafe - rotate),
        };
        double max = Math.abs(wheelpowers[0]);
        for (int i = 0; i < wheelpowers.length; i++) {
            if (max < Math.abs(wheelpowers[i])) max = Math.abs(wheelpowers[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < wheelpowers.length; i++) wheelpowers[i] /= max;
        }
        front_left.setPower(wheelpowers[0]);
        front_right.setPower(wheelpowers[1]);
        back_left.setPower(wheelpowers[2]);
        back_right.setPower(wheelpowers[3]);
    }

    public void setDriveStop() {
        setDrive(0, 0, 0, 0);
    }

    public void setDriveDeltaPos(int deltaPos, double power) {

        for (DcMotor m : driveMotors) {
            int curPos = m.getCurrentPosition();
            int newPos = curPos + deltaPos;
            m.setTargetPosition(newPos);

            // Turn On RUN_TO_POSITION
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotor m : driveMotors) {
            m.setPower(Math.abs(power));
        }
    }

    public void driveDistance(double inches, double power, double timeoutSeconds){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        int movecounts = (int)(convertInchesToCounts(inches));
        if(opModeIsActive()&&runtime.seconds()<timeoutSeconds){
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int fr = front_right.getCurrentPosition();
            int fl = front_left.getCurrentPosition();
            int br = back_right.getCurrentPosition();
            int bl = back_left.getCurrentPosition();

            int newfr = fr + movecounts;
            int newfl = fl + movecounts;
            int newbr = br + movecounts;
            int newbl = bl + movecounts;

            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            front_left.setTargetPosition(newfl);
            front_right.setTargetPosition(newfr);
            back_left.setTargetPosition(newbl);
            back_right.setTargetPosition(newbr);

            front_left.setPower(power);
            front_right.setPower(power);
            back_left.setPower(power);
            back_right.setPower(power);
        }
        setDriveStop();
    }

    public void rotateToHeading(double heading, double power, double timeoutSeconds){

    }
    public void clawClose(){

            leftClaw.setPosition(leftClawClosePosition);
            rightClaw.setPosition(rightClawClosePosition);

    }
    public void clawOpen(){

            leftClaw.setPosition(leftClawOpenPosition);
            rightClaw.setPosition(rightClawOpenPosition);

    }
    public void armMaint(double timeout){
        ElapsedTime armtime = new ElapsedTime();
        while(armtime.seconds()<timeout){
            arm.setPower(armMaintPower);
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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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
}
