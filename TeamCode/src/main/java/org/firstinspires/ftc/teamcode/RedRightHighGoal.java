package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="RedRightHighGoal", group="Linear Opmode")
//@Disabled
public class RedRightHighGoal extends AutonomousController {

    double firstDegrees;

    @Override
    public void runOpMode(){

        initRobotWithoutVision();


        while(opModeIsActive()) {

            setShooterVelocity();

            forwardDistance(0.6, 159);
            //forwardDistance(0.3, 20);

            sidewaysDistanceLeft(0.3, 34);

            intakeRampShooter();
            sleep(6000);

            //park at white line
            forwardDistance(0.4, 27);

            //change to move out of the way
            sidewaysDistanceRight(0.5, 48.26);


        break;

        }

    }
}
