package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="GyroTest", group="Linear Opmode")
//@Disabled
public class GyroTest extends AutonomousController {

    @Override
    public void runOpMode() throws InterruptedException {
        //has vision
        initRobot();

        while(opModeIsActive()) {
            // GyroTurn(170, 0.6);

            //powerCRServo2(-0.8);
            //sleep(1000);

            //ungrip
           // powerCRServo(-0.6);
            //sleep(1000);

            resetEncoders();

            sleep(500);

            GyroTurn(170, 0.5);

            GyroTurn(-170, 0.5);

            break;

        }
    }
}
