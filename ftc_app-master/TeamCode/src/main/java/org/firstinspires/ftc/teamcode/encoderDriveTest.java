package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

@Autonomous(name = "Encoder")
public class encoderDriveTest extends TunableLinearOpMode {

    robotBase robot                     = new robotBase();
    private ElapsedTime runtime         = new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();
        robot.encoderDriveStraight(24, 4.0, opModeIsActive(), runtime);
    }
}
