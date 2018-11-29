package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Initialize")
//@Disabled
public class initialize extends LinearOpMode {
    robotBase robot = new robotBase();
    boolean foundState = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.traverse.setPosition(robot.midTraverseLeft);
        initialize();
        robot.inVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.inVertical.setTargetPosition(50);

        waitForStart();
        while(opModeIsActive())
            idle();
    }

    public void initialize(){
         //If home isn't found
        if(robot.hall.getState()){
            robot.ADM.setPower(-1);
            initialize();
        }
        //If home is found, runs this only once
        else if(!robot.hall.getState()){
            foundState = true;
            robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ADM.setTargetPosition(0);
            initialize();
        }
        //While in transit
        else if(robot.ADM.getTargetPosition() != robot.ADM.getCurrentPosition()){
            robot.ADM.setPower(.1);
            initialize();
        }
        //If 0 is found, stop motion.
        else{
            robot.ADM.setPower(0);
        }
    }
}
