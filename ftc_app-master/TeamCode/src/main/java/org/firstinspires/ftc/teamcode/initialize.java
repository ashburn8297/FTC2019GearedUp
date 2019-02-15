package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Initialize")
//@Disabled
public class initialize extends LinearOpMode {
    robotBaseInitialize robot = new robotBaseInitialize();
    boolean foundState = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.traverse.setPosition(robot.midTraverseRight);
        robot.intakeGate.setPosition(-1.0);


        waitForStart();
        while(opModeIsActive()) {
            if(robot.hall.getState()){
                robot.ADM.setPower(-.5);
                idle();
            }
            //If home is found, runs this only once
            else if(!robot.hall.getState()){
                foundState = true;
                robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ADM.setTargetPosition(0);
                idle();
            }
            //While in transit
            else if(robot.ADM.getTargetPosition() != robot.ADM.getCurrentPosition()){
                robot.ADM.setPower(.3);
                idle();
            }
            //If 0 is found, stop motion.
            else{
                robot.ADM.setPower(0);
                idle();
            }
            idle();
        }
    }
}
