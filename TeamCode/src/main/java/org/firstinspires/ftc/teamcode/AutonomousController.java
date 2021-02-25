package org.firstinspires.ftc.teamcode;

/**
 @author Shriyana Grande,
 @author Arjoon
 @version 1.0
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Autonomous Controller", group="Linear Opmode")
@Disabled
public abstract class AutonomousController extends BaseController {

    private OpenCvCamera webcam;
    private EOCV.RingDetectionPipeline pipeline;

    public void initRobot(){
        super.initRobot();
        applyBrake();

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "HelixWebcam"));

        pipeline = new EOCV.RingDetectionPipeline();

        String eocvval = getRingCount();

        while(!opModeIsActive()) {


            if (gamepad2.dpad_up) {
                pipeline.FOUR_RING_THRESHOLD = pipeline.FOUR_RING_THRESHOLD + 1;
                printTelemetryUpdates();
                sleep(200);
            } else if (gamepad2.dpad_down) {
                pipeline.FOUR_RING_THRESHOLD = pipeline.FOUR_RING_THRESHOLD - 1;
                printTelemetryUpdates();
                sleep(200);
            } else if (gamepad2.dpad_right) {
                pipeline.ONE_RING_THRESHOLD = pipeline.ONE_RING_THRESHOLD + 1;
                printTelemetryUpdates();;
                sleep(200);
            } else if (gamepad2.dpad_left) {
                pipeline.ONE_RING_THRESHOLD = pipeline.ONE_RING_THRESHOLD - 1;
                printTelemetryUpdates();
                sleep(200);
            } else if (gamepad2.a) {
                while(pipeline.FOUR_RING_THRESHOLD > (pipeline.ONE_RING_THRESHOLD+3)){
                    pipeline.ONE_RING_THRESHOLD += 1;
                }
                printTelemetryUpdates();
                sleep(200);
            } else if (gamepad2.y) {
                while(pipeline.ONE_RING_THRESHOLD < (pipeline.FOUR_RING_THRESHOLD-3)){
                    pipeline.FOUR_RING_THRESHOLD -= 1;
                }
                printTelemetryUpdates();
                sleep(200);
            } else {
                eocvval = pipeline.position.name();
                if(eocvval.equals("NONE")){
                    wobblePos = "A";
                } else if(eocvval.equals("ONE")){
                    wobblePos = "B";
                } else{
                    wobblePos = "C";
                }
            }

        }

        waitForStart();

    }
    public void initRobotWithoutVision(){
        super.initRobot();

        waitForStart();

    }

    public String getRingCount(){

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }
        });
        sleep(3000);
        printTelemetryUpdates();

        return pipeline.position.name();

    }

    //Print the telemetry updates from the camera stream
    private void printTelemetryUpdates(){
        telemetry.addData("Analysis", pipeline.avg1);
        telemetry.addData("Four Ring Threshold", pipeline.FOUR_RING_THRESHOLD);
        telemetry.addData("One Ring Threshold", pipeline.ONE_RING_THRESHOLD);
        telemetry.addData("WobblePos", wobblePos);
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        // Don't burn CPU cycles busy-looping in this sample
        sleep(50);
    }
    //Move robot forward a certain number of centimeters
    public void forwardDistance(double pow, double distance) {

        boolean motorMoving = true;

        double rotationsNeeded = ((20)*distance)/wheelCircumference;
        int encoderTarget = (int) (rotationsNeeded*motorOutputCount);


        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setTargetPosition(encoderTarget);
        topRight.setTargetPosition(encoderTarget);
        botLeft.setTargetPosition(encoderTarget);
        botRight.setTargetPosition(encoderTarget);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        forward(pow);

        while(topLeft.isBusy()||topRight.isBusy()||botLeft.isBusy()||botRight.isBusy()){

            telemetry.addData("Angle:", getAngleZ() );
            telemetry.update();

        }

        forward(0);

        motorMoving = false;
    }
    public void resetEncoders() {
        //Changed ending of method so that it does not run on encoders after method
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        botLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        botRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //telemetry.addData("Resetting", "Encoders");
        //telemetry.update();
    }
    public void forwardPIDF(double pow, double distance) {

        frontLeft.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        frontLeft.setPositionPIDFCoefficients(5.0);
        frontRight.setVelocityPIDFCoefficients(1.037, 0.1037, 0, 10.37);
        frontRight.setPositionPIDFCoefficients(5.0);
        backLeft.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        backLeft.setPositionPIDFCoefficients(5.0);
        backRight.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        backRight.setPositionPIDFCoefficients(5.0);

        double rotationsNeeded = ((20)*distance)/wheelCircumference;
        int encoderTarget = (int) (rotationsNeeded*motorOutputCount);


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(encoderTarget);
        frontRight.setTargetPosition(encoderTarget);
        backLeft.setTargetPosition(encoderTarget);
        backRight.setTargetPosition(encoderTarget);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(pow*3100);
        frontRight.setVelocity(pow*3160);
        backLeft.setVelocity(pow*3100);
        backRight.setVelocity(pow*3100);

    }
    //Move robot backward a certain number of centimeters
    public void backwardDistance(double pow, double distance) {

        boolean motorMoving = true;

        double rotationsNeeded = (20*distance)/wheelCircumference;
        int encoderTarget = (int) (rotationsNeeded*motorOutputCount);


        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setTargetPosition(-encoderTarget);
        topRight.setTargetPosition(-encoderTarget);
        botLeft.setTargetPosition(-encoderTarget);
        botRight.setTargetPosition(-encoderTarget);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backward(pow);

        while(topLeft.isBusy()||topRight.isBusy()||botLeft.isBusy()||botRight.isBusy()){

            telemetry.addData("Going:"," "+distance+" centimeters." );
            telemetry.update();

        }

        forward(0);

        motorMoving = false;
    }
    //Move robot to the left a certain number of centimeters
    public void sidewaysDistanceLeft(double pow, double distance) {

        boolean motorMoving = true;

        double rotationsNeeded = (20*distance)/wheelCircumference;
        int encoderTarget = (int) (rotationsNeeded*motorOutputCount);


        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setTargetPosition(-encoderTarget);
        botLeft.setTargetPosition(encoderTarget);
        topRight.setTargetPosition(encoderTarget);
        botRight.setTargetPosition(-encoderTarget);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sidewaysLeft(pow);

        while(topLeft.isBusy()||topRight.isBusy()||botLeft.isBusy()||botRight.isBusy()){

            telemetry.addData("Going:"," "+distance+" centimeters." );
            telemetry.update();

        }

        forward(0);
        motorMoving = false;
    }

    //Move robot to the right a certain number of centimeters
    public void sidewaysDistanceRight(double pow, double distance) {

        boolean motorMoving = true;

        double rotationsNeeded = (20*distance)/wheelCircumference;
        int encoderTarget = (int) (rotationsNeeded*motorOutputCount);


        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setTargetPosition(encoderTarget);
        botLeft.setTargetPosition(-encoderTarget);
        topRight.setTargetPosition(-encoderTarget);
        botRight.setTargetPosition(encoderTarget);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sidewaysRight(pow);

        while(topLeft.isBusy()||topRight.isBusy()||botLeft.isBusy()||botRight.isBusy()){

            telemetry.addData("Going:"," "+distance+" centimeters." );
            telemetry.update();

        }

        forward(0);
        motorMoving = false;
    }

    public void turnLeftDistance(double pow, double distance) {

        boolean motorMoving = true;

        double rotationsNeeded = ((20)*distance)/wheelCircumference;
        int encoderTarget = (int) (rotationsNeeded*motorOutputCount);


        //topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //topLeft.setTargetPosition(-encoderTarget);
        //topRight.setTargetPosition(-encoderTarget);
        botLeft.setTargetPosition(encoderTarget);
        botRight.setTargetPosition(encoderTarget);

        //topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //botLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topRight.setPower(pow);
        botRight.setPower(pow);

        while(topLeft.isBusy()||topRight.isBusy()||botLeft.isBusy()||botRight.isBusy()){

            telemetry.addData("Angle:", getAngleZ() );
            telemetry.update();

        }

        forward(0);

        motorMoving = false;
    }
    public void turnRightDistance(double pow, double distance) {

        boolean motorMoving = true;

        double rotationsNeeded = ((20)*distance)/wheelCircumference;
        int encoderTarget = (int) (rotationsNeeded*motorOutputCount);


        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setTargetPosition(encoderTarget);
        topRight.setTargetPosition(encoderTarget);
        botLeft.setTargetPosition(-encoderTarget);
        botRight.setTargetPosition(-encoderTarget);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turnLeft(pow);

        while(topLeft.isBusy()||topRight.isBusy()||botLeft.isBusy()||botRight.isBusy()){

            telemetry.addData("Angle:", getAngleZ() );
            telemetry.update();

        }

        forward(0);

        motorMoving = false;
    }
    //Get the current angle of the robot respective to when it was last at 0
    public double getAngleZ() {
        Orientation angles;
        angles = myGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    //Turn a certain amount of degrees
    public void GyroTurn(double degrees, double power) {

        final double startAngle = getAngleZ();
        telemetry.addData("Start angle on gyroTurn", startAngle);
        telemetry.update();
        //sleep(1000);
        if(degrees < 0) {
            double targetDegrees = startAngle + degrees;

            while((getAngleZ()>(targetDegrees))){
                telemetry.addData("Inide Gyro",targetDegrees);
                telemetry.addData("Inide Gyro", getAngleZ() );
                telemetry.addData("Inide Gyro", (getAngleZ()>(targetDegrees)) );
                telemetry.update();
                turnRight(power);
                telemetry.addData("Inide Gyro", getAngleZ());
                telemetry.update();
            }
            forward(0);

        } else if(degrees > 0) {
            double targetDegrees = startAngle + degrees;
            telemetry.addData("target Degrees", targetDegrees);
            telemetry.addData("Angle Z inside degrees >0", getAngleZ());
            telemetry.update();

            while((getAngleZ()<(targetDegrees))){
                telemetry.addData("Out Gyro",targetDegrees);
                telemetry.addData("Out Gyro", getAngleZ() );
                telemetry.addData("Out Gyro", (getAngleZ()<(targetDegrees)) );
                telemetry.update();
                telemetry.addData("Gyro", getAngleZ());
                telemetry.update();
                turnLeft(power);
                telemetry.addData("Gyro", getAngleZ());
                telemetry.update();
            }
            forward(0);


        } else {
            telemetry.addData("Gyro", getAngleZ());
            telemetry.update();
            turnLeft(0);
        }
    }

    //turn to 0 degrees
    public void GyroTurnTo0(double pow) {

        final double startAngle = getAngleZ();

        if(startAngle>0){
            while(getAngleZ() > 0){
                turnRight(pow);
                telemetry.addData("Gyro", getAngleZ());
                telemetry.update();
            }
        } else if(startAngle<0){
            while(getAngleZ() < 0){
                turnLeft(pow);
                telemetry.addData("Gyro", getAngleZ());
                telemetry.update();
            }
        } else{
            forward(0);
            telemetry.addData("Gyro", getAngleZ());
            telemetry.update();
        }

    }

    public void GyroTurnTo180(double pow) {

        final double startAngle = getAngleZ();

        if(startAngle>0){
            while(getAngleZ() < 180 && getAngleZ() >= 0){
                turnLeft(pow);
                telemetry.addData("Gyro", getAngleZ());
                telemetry.update();
            }
        } else if(startAngle<0){
            while(getAngleZ() > -180 && getAngleZ() <= 0){
                turnRight(pow);
                telemetry.addData("Gyro", getAngleZ());
                telemetry.update();
            }
        } else{
            forward(0);
            telemetry.addData("Gyro", getAngleZ());
            telemetry.update();
        }

    }


    public void GyroTurnTest(double degrees, double power) {

        final double startAngle = getAngleZ();

        if(degrees < 0) {
            if(startAngle > degrees) {
                while(startAngle > degrees) {
                    turnLeft(power);
                }
            } else {

            }
        }


        if(degrees < 0) {
            double targetDegrees = startAngle + degrees;
            telemetry.addData("Gyro", getAngleZ());
            telemetry.update();
            while((getAngleZ()>(targetDegrees))&&(getAngleZ()<=0)){
                telemetry.addData("Gyro", getAngleZ());
                telemetry.update();
                turnLeft(-power);
                telemetry.addData("Gyro", getAngleZ());
                telemetry.update();
            }
            forward(0);
        } else if(degrees > 0) {
            double targetDegrees = startAngle + degrees;

            while((getAngleZ()<(targetDegrees))&&(getAngleZ()>=0)){

                telemetry.addData("Gyro", getAngleZ());
                telemetry.update();
                turnLeft(power);
                telemetry.addData("Gyro", getAngleZ());
                telemetry.update();
            }
            forward(0);
        } else {
            telemetry.addData("Gyro", getAngleZ());
            telemetry.update();
            turnLeft(0);
        }
    }
    //
    //Drive the robot straight with correction for a certain number of centimeters
    public void driveStraightForwardDistance(double power, double distance){

        double startAngle = getAngleZ();
        forwardDistance(power, distance);

        while(topLeft.getCurrentPosition()!=topLeft.getTargetPosition()){
            double drift = -0.01*(Math.abs(startAngle-getAngleZ()));

            topLeft.setPower(power-drift);
            botLeft.setPower(power-drift);
            topRight.setPower(power+drift);
            botRight.setPower(power+drift);
        }
    }
}
