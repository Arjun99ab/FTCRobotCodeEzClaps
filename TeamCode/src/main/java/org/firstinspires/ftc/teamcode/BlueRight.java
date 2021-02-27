package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="BlueRight", group="Linear Opmode")
//@Disabled
public class BlueRight extends AutonomousController {

    double firstDegrees;

    @Override
    public void runOpMode(){

        initRobot();

        while(opModeIsActive()) {
            if(wobblePos == "A") {

                boolean isClamped = false;

                setShooterVelocity();

                //grip
                powerCRServo(1);
                sleep(1000);

                //lower
                powerCRServo2(-0.8);
                sleep(1500);

                //grip
                powerCRServo(1);
                sleep(500);

                //raise
                powerCRServo2(0.8);
                sleep(1000);

                forwardDistance(0.7, 190);

                sidewaysDistanceLeft(0.4, 50); //measure encoders

                resetEncoders();

                GyroTurn(170, 0.6);

                //lower
                powerCRServo2(-0.8);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);
                forwardDistance(0.4, 37);

                resetEncoders();

                GyroTurn(-170, 0.4);

                //navigate to shooter position

                sidewaysDistanceRight(0.3, 33);

                //setShooterVelocity();
                //sleep(2000);

                intakeRampShooter();
                sleep(6000);

                //park at white line
                forwardDistance(0.4, 27);

                sidewaysDistanceRight(0.5, 50);

            } else if(wobblePos == "B") {

                boolean isClamped = false;

                setShooterVelocity();

                //grip
                powerCRServo(1);
                sleep(1000);

                //lower
                powerCRServo2(-0.8);
                sleep(1500);

                //grip
                powerCRServo(1);
                sleep(500);

                //raise
                powerCRServo2(0.8);
                sleep(1000);

                forwardDistance(0.7, 246);

                sidewaysDistanceLeft(0.4, 50); //measure encoders

                //lower
                powerCRServo2(-0.8);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                backwardDistance(0.4, 53); //measure encoders

                //navigate to shooter position
                sidewaysDistanceRight(0.3, 33);

                intakeRampShooter();
                sleep(6000);

                //park at white line
                forwardDistance(0.4, 27);

                sidewaysDistanceRight(0.5, 50);

            } else {
                boolean isClamped = false;

                setShooterVelocity();

                //grip
                powerCRServo(1);
                sleep(1000);

                //lower
                powerCRServo2(-0.8);
                sleep(1500);

                //grip
                powerCRServo(1);
                sleep(500);

                //raise
                powerCRServo2(0.8);
                sleep(1000);

                forwardDistance(0.7, 312);

                sidewaysDistanceLeft(0.4, 50); //measure encoders

                resetEncoders();

                GyroTurn(170, 0.6);

                //lower
                powerCRServo2(-0.8);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                forwardDistance(0.4, 75); //measure encoders

                resetEncoders();

                GyroTurn(-170, 0.4);

                //navigate to shooter position

                sidewaysDistanceRight(0.3, 33);

                //setShooterVelocity();
                //sleep(2000);

                intakeRampShooter();
                sleep(6000);

                //park at white line
                forwardDistance(0.4, 27);

                sidewaysDistanceRight(0.5, 50);

            }

            break;

        }

    }
}
