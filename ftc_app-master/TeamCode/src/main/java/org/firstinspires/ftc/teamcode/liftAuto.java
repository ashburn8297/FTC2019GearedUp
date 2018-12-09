package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.robotBase.midTraverseRight;

@Autonomous(name = "LiftAuto")
public class liftAuto extends LinearOpMode {
    robotBase robot = new robotBase();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.traverse.setPosition(midTraverseRight);
        robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        runtime.reset();

        //Lower Lift
        if (opModeIsActive()){
            robot.ADM.setTargetPosition((int) (robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV_rev) - 100); //tuner
            robot.ADM.setPower(.5);
            telemetry.addData("Lift Encoder Value", robot.ADM.getCurrentPosition());
        }

        sleep(5000);
        robot.ADM.setPower(.1); //To stop jittering

        //Slide over
        if (opModeIsActive()){
            robot.traverse.setPosition(robot.maxTraverse);
            robot.marker.setPosition(robot.markerMid);
            sleep(1000);
            robot.intakePitch.setPosition(robot.boxStowed);
            sleep(1000);
            robot.marker.setPosition(robot.markerIn);
        }
        sleep(3000);


    }
}
