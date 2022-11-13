package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoRedAllainceRedTerminal",group="19380")
public class AutoRedRedTerminal extends LinearOpMode {
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
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


    public void runOpMode() throws InterruptedException {
        front_left   = hardwareMap.get(DcMotor.class, "fldrive");
        front_right  = hardwareMap.get(DcMotor.class, "frdrive");
        back_left    = hardwareMap.get(DcMotor.class, "bldrive");
        back_right   = hardwareMap.get(DcMotor.class, "brdrive");
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
        waitForStart();
        if(opModeIsActive()){
            ElapsedTime runtime2 = new ElapsedTime();

            while(runtime2.seconds()<1.5){

                front_right.setPower(0.5);
                front_left.setPower(-0.5);
                back_right.setPower(-0.5);
                back_left.setPower(0.5);

            }
            front_left.setPower(0);
            front_right.setPower(0);
            back_right.setPower(0);
            back_left.setPower(0);
        }
    }
}