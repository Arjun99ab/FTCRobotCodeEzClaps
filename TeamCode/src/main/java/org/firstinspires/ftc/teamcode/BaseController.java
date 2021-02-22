package org.firstinspires.ftc.teamcode;

/**
 @author Shriyana Grande,
 @author Arjoon
 @version 1.0
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="Base Controller", group="Linear Opmode")
@Disabled
public abstract class BaseController extends LinearOpMode {

    //Instance variable definitions
    /**
     @param topLeft DC Motor placed top left
     @param CRServo Continous rotation servo
     */
    protected DcMotor topLeft;
    protected DcMotor topRight;
    protected DcMotor botLeft;
    protected DcMotor botRight;
    protected DcMotor shooterLeft;
    protected DcMotor wobbleArm;
    protected DcMotorEx frontLeft;
    protected DcMotorEx frontRight;
    protected DcMotorEx backLeft;
    protected DcMotorEx backRight;
    protected DcMotorEx shooter;
    protected DcMotor rampMotor;
    protected BNO055IMU myGyro;
    protected CRServo crServo;
    protected CRServo crServo2;
    protected CRServo intakeServo;
    protected Servo angServo;
    protected DistanceSensor distanceSensorLeft;
    protected DistanceSensor distanceSensorRight;
    protected ModernRoboticsI2cRangeSensor distanceSensorFront;


    /*
    protected DistanceSensor distanceSensor;
    protected ModernRoboticsI2cRangeSensor ultraDistance;
     */

    //EOCV eocv = new EOCV();

    public double pow;

    //Instance variables for Tensor Flow
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false;
    public static final String VUFORIA_KEY =
            "Aag2kDT/////AAABmUQpJJbPIEhBikXKqBb5o5pABXTdOH9wWXYhvoQSnvI5ehKGFBH1Rv/JyPZE1WLVhid9hWqEHZIEHUcLU97MIgXlwxuqin56w4W63NJou8Usx/FYnbFb8SyyestvpnL92rFbk+FIfzbDWbPAEXjhXykeNnnfoRl1w/J6x0NrhqVlj7LtJYC/Pz/GqLCDKB2H2NFamFMSWDAjqZxpURMAU0zEI5byl4Cn4gul5IwKArWWZ4BuRjPCLZkAcMiovs5v1ksNJ7ey+kEI2HgBI3KVkDaCgCpHh0QjiOJxVa3L4DpLGFvuTloQxANtsx03yW1ia+u/Mp+5CDXTqBWQeJO55TA3O/44DwjQHe71WtilHQ87";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public String wobblePos = "Z";

    //Motor output counts
    /*The motor output count will vary from different torque motors as follows:

    SKU 5202-0002-0188, 188:1 Ratio - 5264
    SKU 5202-0002-0139, 139:1 Ratio - 3892
    SKU 5202-0002-0100, 99.5:1 Ratio - 2786
    SKU 5202-0002-0071, 71.2:1 Ratio - 1993.6
    SKU 5202-0002-0051, 50.9:1 Ratio - 1425.2
    SKU 5202-0002-0027, 26.9:1 Ratio - 753.2
    SKU 5202-0002-0019, 19.2:1 Ratio - 537.6
    SKU 5202-0002-0014, 13.7:1 Ratio - 383.6
    SKU 5202-0002-0005, 5.2:1 Ratio - 145.6
    SKU 5202-0002-0003, 3.7:1 Ratio - 103.6
     */

    //The motors currently being used for the Mecanum Drivetrain are the 13.7:1 Ratio Motors.
    public static final int motorOutputCount = (int) 383.6;
    public static final double wheelCircumference = (96* Math.PI);

    //Currently not used
    public double distance;
    public double rotationsNeeded;
    public Servo angServo2;
    public TouchSensor touchSensor;
    double zeroDistance;
    public double oneDistance;
    public double fourDistance;
    public double firstDistance;
    public double zeroDistance1;
    public double currentDistance;
    public double currentDistance2;



    //Will be called at the beginning of the TeleOp program, so we initiate everything needed and then call waitForStart()

    public void initRobot(){

        topLeft  = hardwareMap.dcMotor.get("topLeft");
        topRight  = hardwareMap.dcMotor.get("topRight");
        botLeft  = hardwareMap.dcMotor.get("botLeft");
        botRight  = hardwareMap.dcMotor.get("botRight");

        botLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterLeft  = hardwareMap.dcMotor.get("shooterLeft"); //port 1

        shooter = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleArm = hardwareMap.dcMotor.get("wobbleArm");

        frontLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "topRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "botLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "botRight");

        rampMotor = hardwareMap.dcMotor.get("rampMotor"); //port 0
        rampMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeServo = hardwareMap.crservo.get("intakeServo");
        crServo = hardwareMap.crservo.get("crServo");
        crServo2 = hardwareMap.crservo.get("crServo2");

        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        //distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        distanceSensorFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceSensorFront");

        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        myGyro = hardwareMap.get(BNO055IMU.class, "expansionImu");

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        myGyro.initialize(imuParameters);

        //waitForStart();
    }

    //Move robot forward
    public void forward(double pow) {

        topLeft.setPower(pow);
        topRight.setPower(pow);
        botLeft.setPower(pow);
        botRight.setPower(pow);

    }
    public void applyBrake() {
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void applyCoast() {
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        botLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        botRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //Move robot backward
    public void backward(double pow){

        topLeft.setPower(-pow);
        topRight.setPower(-pow);
        botLeft.setPower(-pow);
        botRight.setPower(-pow);

    }

    ////Move robot to the left
    public void sidewaysLeft(double pow) {


        topLeft.setPower(-pow);
        botLeft.setPower(pow);
        topRight.setPower(pow);
        botRight.setPower(-pow);

    }

    //Move robot to the right
    public void sidewaysRight(double pow) {


        topLeft.setPower(pow);
        botLeft.setPower(-pow);
        topRight.setPower(-pow);
        botRight.setPower(pow);

    }




    //Turn the robot left
    public void turnLeft(double pow){

        topLeft.setPower(-pow);
        botLeft.setPower(-pow);
        topRight.setPower(pow);
        botRight.setPower(pow);
        telemetry.addData("hasdasd", pow);
        telemetry.update();

    }

    //Turn the robot right
    public  void turnRight(double pow){
        topLeft.setPower(pow);
        botLeft.setPower(pow);
        topRight.setPower(-pow);
        botRight.setPower(-pow);
    }





    /*
    public void setPowerTeleOp(){

        topLeft.setPower(gamepad1.left_stick_y);
        botLeft.setPower(gamepad1.left_stick_y);

        topRight.setPower(gamepad1.right_stick_y);
        botRight.setPower(gamepad1.right_stick_y);

    }

     */

    //Set the position of an angular servo
    public void setServoPos(double pos) {
        angServo.setPosition(pos);
    }

    /*
    public void setServoPos2(double pos) {
        angServo2.setDirection(Servo.Direction.FORWARD);
        angServo2.setPosition(pos);
    }

     */
    //Set the power of a continuous rotation servo (gripper)
    public void powerCRServo(double pow) {
        crServo.setDirection(CRServo.Direction.FORWARD);
        crServo.setPower(pow);
    }
    //Set the power of a continuous rotation servo (elbow)
    public void powerCRServo2(double pow) {
        crServo2.setDirection(CRServo.Direction.FORWARD);
        crServo2.setPower(pow);
    }
    public void powerWobbleArm(double pow) {
        wobbleArm.setPower(pow);
    }


    /*
    public void shootRings(double pow, double seconds){
        while(getRuntime()<=seconds){
            shooterLeft.setPower(pow);
            shooterRight.setPower(pow);
            rampMotor.setPower(1);
        }

    }

     */
    /*
    public void testRed(){
        while(opModeIsActive()){
            telemetry.addData("Color Reading", ClrSensor.red());
            telemetry.update();
        }
    }
    public void stopAtRed(double pow) {
        int z = 0;
        while(z == 0){
            if(ClrSensor.red() > 70 & ClrSensor.red() < 100) {
                forward(0);
                z = 1;
                break;
            }
            else{
                forward(pow);
            }
            telemetry.addData("Color Reading", ClrSensor.red());
            telemetry.update();
        }
    }

    public void stopAtWhite(double pow) {
        int z = 0;
        while(z == 0){
            if(ClrSensor.red() > 150) {
                forward(0);
                z = 1;
                break;
            }
            else{
                forward(pow);
            }
            telemetry.addData("Color Reading", ClrSensor.red());
            telemetry.update();
        }
    }
    public void stopAtBlue(double pow) {
        forward(pow);
        if(ClrSensor.blue() == 10){
            forward(0);
        }
    }
    public void stopAtGreen(double pow) {
        forward(pow);
        if (ClrSensor.green() == 10) {
            forward(0);
        }
    }
    */

    //Run the intake, ramp, and shooter at the same time
    public void chargeMotor(){

        shooterLeft.setPower(0.56);
    }
    public void setShooterVelocity(){
        double shooterVelocity = 1344;

        shooter.setVelocity(shooterVelocity);
    }
    public void setShooterVelocityPowershot(){
        double shooterVelocity = 1224;

        shooter.setVelocity(shooterVelocity);
       while (shooter.isBusy()) {
           telemetry.addData("Current Velocity ", shooter.getVelocity());
           telemetry.update();
       }
    }


    public void intakeRampShooter(){

        rampMotor.setPower(1);
        setShooterVelocity();
        intakeServo.setDirection(CRServo.Direction.FORWARD);
        intakeServo.setPower(1);
    }

    public void intakeRampShooterPowershot(){

        rampMotor.setPower(1);
        //setShooterVelocityPowershot(3);
        intakeServo.setDirection(CRServo.Direction.FORWARD);
        intakeServo.setPower(1);
    }

    /*
    public void driveUntilTouched(double pow) {
        while (!(touchSensor.isPressed())) {
            forward(pow);
        }
        forward(0);

    }

     */


    public void driveUntilDistanceCMLEFT(double pow, double distance){
        while(distanceSensorLeft.getDistance(DistanceUnit.CM) > distance){
            sidewaysLeft(pow);
        }
        applyBrake();
        forward(0);
    }
    public void driveUntilDistanceCMRIGHT(double pow, double distance){
        while(distanceSensorLeft.getDistance(DistanceUnit.CM) < distance){
            sidewaysRight(pow);
        }
        applyBrake();
        forward(0);
    }
    public void driveUntilDistanceCMLEFT2(double pow, double distance){
        while(distanceSensorRight.getDistance(DistanceUnit.CM) < distance){
            sidewaysLeft(pow);
        }
        applyBrake();
        forward(0);
    }
    public void driveUntilDistanceCMRIGHT2(double pow, double distance){
        while(distanceSensorRight.getDistance(DistanceUnit.CM) > distance){
            sidewaysRight(pow);
        }
        applyBrake();
        forward(0);
    }
    public void driveUntilDistanceCMFORWARD(double pow, double distance){
        while(distanceSensorFront.getDistance(DistanceUnit.CM) > distance){
            forward(pow);
        }
        //applyBrake();
        forward(0);
    }
    public void driveUntilDistanceCMBACKWARD(double pow, double distance){
        while(distanceSensorFront.getDistance(DistanceUnit.CM) < distance){
            backward(pow);
        }
        //applyBrake();
        forward(0);
    }

    /*
    public void ultraDriveUntilDistanceSensedCentimeters(double pow, double distance) {

        while (ultraDistance.getDistance(DistanceUnit.CM) < distance) {
            forward(pow);
        }
        forward(0);
    }
    public void ultraDriveUntilDistanceSensedMeters(double pow, double distance) {
        while (ultraDistance.getDistance(DistanceUnit.METER) < distance) {
            forward(pow);
        }
        forward(0);
    }
    public void ultraDriveUntilDistanceSensedInches(double pow, double distance) {
        while (ultraDistance.getDistance(DistanceUnit.INCH) < distance) {
            forward(pow);
        }
        forward(0);
    }
    public void ultraDriveUntilDistanceSensedFeet(double pow, double distance) {
        while (12*(ultraDistance.getDistance(DistanceUnit.INCH)) < distance) {
            forward(pow);
        }
        forward(0);
    }

     */






    /*
    //Initialize the vuforia
    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "HelixWebcam");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

     */

    /*
    //Initialize the Tensorflow
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

     */


    /*
    public void detectWobblePos() {
        telemetry.addData("prior to checking tfod for not null", wobblePos);
        telemetry.update();

        while (wobblePos == "Z") {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.

            telemetry.addData("prior to getting recognitions...", wobblePos);
            telemetry.update();

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

     */
            /*
            int count = 1000;

            while (count > 0) {
                updatedRecognitions = tfod.getUpdatedRecognitions();

                telemetry.addData("getting recognitions...", wobblePos);
                telemetry.update();

                if (updatedRecognitions != null)
                    break;
                count--;
            }

             */
            /*
            while(getRuntime() <= 5.0){
                updatedRecognitions = tfod.getUpdatedRecognitions();

                telemetry.addData("getting recognitions...", wobblePos);
                telemetry.update();

                if (updatedRecognitions != null)
                    break;
            }
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step t-hrough the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    telemetry.update();

                    if (recognition.getLabel() == "Single") {
                        wobblePos = "B";
                        telemetry.addData("Wobble Position should be B", wobblePos);
                        telemetry.update();
                        continue;
                    } else {
                        wobblePos = "C";
                        telemetry.addData("Wobble Position should be C", wobblePos);
                        telemetry.update();
                        continue;
                    }
                }

            }else{
                wobblePos = "A";
                telemetry.addData("Wobble Position should be A", wobblePos);
                telemetry.update();
            }

        }
    }

             */

    /*
    //Test the tensorflow for one ring
    public void testVision(){
        if(wobblePos.equals("B")){
            forwardDistance(0.2, 20);
        }
    }

     */
}
