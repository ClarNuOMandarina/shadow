//package org.firstinspires.ftc.teamcode.autos;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import android.annotation.SuppressLint;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.profile.VelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.autos.CameraDetector;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//
//import java.util.Arrays;
//
//@Autonomous(group = "testy_blue")
//
//public class blue_otto_detection_test extends LinearOpMode {
//
//    @SuppressLint("SuspiciousIndentation")
//    @Override
//    public void runOpMode() throws InterruptedException {
//        int caz_inter=3;
//        CameraDetector camera = new CameraDetector(OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1")));
//        CameraDetector.Result result = CameraDetector.Result.NONE;
//        while (opModeInInit()) {
//            result = camera.detect();
//            telemetry.addLine("Location" + result);
//            telemetry.update();
//        }
//        camera.stop();
//        waitForStart();
//        if (isStopRequested()) return;
//        while(opModeIsActive()&&!isStopRequested()){
//            telemetry.update();
//        }
//
//
//    }}