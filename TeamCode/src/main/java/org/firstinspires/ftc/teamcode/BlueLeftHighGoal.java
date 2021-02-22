package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BlueLeftHighGoal", group="Linear Opmode")
//@Disabled
public class BlueLeftHighGoal extends AutonomousController {

    double firstDegrees;

    @Override
    public void runOpMode(){

        initRobotWithoutVision();


        while(opModeIsActive()) {

            setShooterVelocity();

            forwardDistance(0.6, 160);
            //forwardDistance(0.3, 20);

            sidewaysDistanceRight(0.3, 34);

            intakeRampShooter();
            sleep(6000);

            //park at white line
            forwardDistance(0.4, 27);

            //change to move out of the way
            sidewaysDistanceLeft(0.5, 48.26);


        break;

        }

    }
}
