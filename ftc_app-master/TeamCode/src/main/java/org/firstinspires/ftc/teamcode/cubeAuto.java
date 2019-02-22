package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.teamcode.robotBase.midTraverseRight;


@Autonomous(name = "CubeAuto SAFE")
//@Disabled
public class cubeAuto extends LinearOpMode {
    robotBaseAuto robot = new robotBaseAuto();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.traverse.setPosition(midTraverseRight);
        robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.update();

        waitForStart();
        runtime.reset();

        //Lower Lift
        if (opModeIsActive()){
            robot.ADM.setTargetPosition((int) (robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV_rev) - 100); //tuner
            robot.ADM.setPower(.95);
            telemetry.addData("Lift Encoder Value", robot.ADM.getCurrentPosition());
        }

        sleep(3000);
        robot.ADM.setPower(.05); //To stop jittering
        robot.traverse.setPosition(robot.maxTraverse);
        sleep(2000);

        robot.turnByEncoder(0, .07, opModeIsActive(), 3.0, runtime);
        robot.encoderDriveStraight(51, 5.0, .24, opModeIsActive(), runtime);
        sleep(1000);
        robot.turnByEncoder(40, .07, opModeIsActive(), 3.0, runtime);

        if (opModeIsActive()) {
            robot.marker.setPosition(robot.markerOut);
            sleep(1000);
            robot.marker.setPosition(robot.markerMid);
        }

    }
}
