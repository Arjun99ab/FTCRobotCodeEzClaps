package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="EncoderDemo", group="Linear Opmode")
@Disabled
public class EncoderDemo extends AutonomousController {

    @Override
    public void runOpMode() throws InterruptedException {

        initRobotWithoutVision();

        while (opModeIsActive()) {
            applyBrake();
            forwardDistance(0.5,50);
            resetEncoders();
            GyroTurn(82, 0.5);
            if(1==1)
             break;
            //grip
           // powerCRServo(1);
           // sleep(1000);

            //lower
          ///  powerCRServo2(-0.8);
          ///  sleep(700);

            //grip
           // powerCRServo(1);
    ///        sleep(500);

            //raise
       //     powerCRServo2(0.8);
         ///   sleep(1000);

            //setShooterVelocity();

            forwardDistance(0.6, 190);
            //forwardDistance(0.3, 20);

            resetEncoders();

            GyroTurn(170, 0.6);

            //lower
        //    powerCRServo2(-0.8);
          //  sleep(1000);

            //ungrip
           // powerCRServo(-0.6);
           // sleep(1000);

            /*
            forwardDistance(0.4, 38);

            resetEncoders();

            GyroTurnTo0(0.6);

            //navigate to shooter position

            sidewaysDistanceRight(0.3, 29);

            //setShooterVelocity();
            //sleep(2000);

            intakeRampShooter();
            sleep(6000);

            //park at white line
            forwardDistance(0.4, 27);

            sidewaysDistanceRight(0.5, 80);

             */
        }
    }
}
