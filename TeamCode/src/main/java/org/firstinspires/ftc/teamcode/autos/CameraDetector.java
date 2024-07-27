package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;


public class CameraDetector
{
   public int cazus=3;
    public enum Result
    {
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }



    public OpenCvWebcam camera;
    public ctsStartDetection ctsStartDetectionPipeline;

    public CameraDetector(OpenCvWebcam camera)
    {
        this.camera = camera;

        ctsStartDetectionPipeline = new ctsStartDetection(dashboard.getTelemetry());

        camera.setPipeline(ctsStartDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
                dashboard.startCameraStream(camera, 40);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    FtcDashboard dashboard = FtcDashboard.getInstance();
//    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;

    // UNITS ARE METERS
//    double tagsize = 0.166;

    // IDs of sleves

//    int Left = 1, Middle = 2, Right = 3;

//    AprilTagDetection tagOfInterest = null;



    public Result detect()
    {

//         * The INIT-loop:
//         * This REPLACES waitForStart!
        ctsStartDetection.PixelPos currentDetections = ctsStartDetectionPipeline.getDetectedPos();

        if(currentDetections == ctsStartDetection.PixelPos.RIGHT) {
            cazus=3;
            return Result.LEFT;
        }
//        else if (currentDetections == ctsStartDetection.PixelPos.LEFT){
//            cazus=1;
//
//            return Result.LEFT;}
        else if (currentDetections == ctsStartDetection.PixelPos.CENTER){
            cazus=2;

            return Result.CENTER;}
        else
        {
            cazus=1;
            return Result.NONE;}

    }

    public void stop()
    {
        dashboard.stopCameraStream();
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}