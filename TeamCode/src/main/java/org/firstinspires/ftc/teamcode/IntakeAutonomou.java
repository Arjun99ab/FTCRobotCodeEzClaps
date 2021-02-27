package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="IntakeAutonomou", group="Linear Opmode")
//@Disabled
public class IntakeAutonomou extends AutonomousController {

    double firstDegrees;

    @Override
    public void runOpMode(){

        initRobot();
        applyBrake();

        //wobblePos = "A";


        while(opModeIsActive()) {
            intakeRampShooter();

        }

    }
}
