package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="EOCV", group="Linear OpMode")
@Disabled
public class EOCV extends BaseController
{
    //private OpenCvCamera webcam;
    //private RingDetectionPipeline pipeline;

    @Override
    public void runOpMode()
    {

    }
    /*
    @Override
    public void runOpMode()
    {

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "HelixWebcam"));

        pipeline = new RingDetectionPipeline();

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.avg1);
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
*/
/*
    public String getRingCount(HardwareMap inputHardwareMap){

        webcam = OpenCvCameraFactory.getInstance().createWebcam(inputHardwareMap.get(WebcamName.class, "HelixWebcam"));

        pipeline = new EOCV.RingDetectionPipeline();

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }
        });
        sleep(5000);
        //printTelemetryUpdates();

        return pipeline.position.name();

    }

    //Print the telemetry updates from the camera stream
    private void printTelemetryUpdates(){
        telemetry.addData("Analysis", pipeline.avg1);
        telemetry.addData("Position", pipeline.position);
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.update();

        // Don't burn CPU cycles busy-looping in this sample
        sleep(50);
    }
*/
    static class RingDetectionPipeline extends OpenCvPipeline
    {

        /*
         * An enum to define the ring position
         */
        enum RingPosition
        {
            ONE,
            FOUR,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */

        //static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,100);

        //static final int REGION_WIDTH = 181;
        //static final int REGION_HEIGHT = 100;






        int FOUR_RING_THRESHOLD = 142;
        int ONE_RING_THRESHOLD = 137;

        /*
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

         */

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        Point pointA;
        Point pointB;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingPosition position = RingPosition.NONE;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);

        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            /*
            pointA = new Point(
                    firstFrame.cols()*(1f/4f),
                    firstFrame.rows()/2);

            pointB = new Point(
                    firstFrame.cols()*(3f/4f),
                    firstFrame.rows()/1);

             */

            pointA = new Point(
                    firstFrame.cols()*(3f/8f),
                    firstFrame.rows()/2);

            pointB = new Point(
                    firstFrame.cols()*(5f/8f),
                    firstFrame.rows()*(3f/4f));


            /*
            pointA = new Point(
                    firstFrame.cols()*(2f/8f),
                    firstFrame.rows()*(7/16));

            pointB = new Point(
                    firstFrame.cols()*(4f/8f),
                    firstFrame.rows()*(3f/4f));

             */

            region1_Cb = Cb.submat(new Rect(pointA, pointB));


        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            //avg1 = (int) Core.mean(region1_Cb).val[0];

            /*
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    5); // Thickness of the rectangle lines
             */

            Imgproc.rectangle(
                    input,
                    pointA,
                    pointB,
                    BLUE, 10);

            avg1 = (int) Core.mean(region1_Cb).val[0];


            position = RingPosition.NONE; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

           /* Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -5); // Negative thickness means solid fill*/

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}