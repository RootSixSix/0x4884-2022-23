package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RobotBrain {
    OpMode opmode;
    Robot robot;

    public RobotBrain(Robot therobot, OpMode theopmode) {
        robot = therobot;
        opmode = theopmode;
    }

    public boolean opModeIsActive() {
        if (opmode instanceof LinearOpMode)
            return ((LinearOpMode) opmode).opModeIsActive();
        else return false;
    }
}
