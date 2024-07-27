package org.firstinspires.ftc.teamcode.resources;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autos.CameraDetector;
import org.firstinspires.ftc.teamcode.autos.CameraDetector_blue;
import org.firstinspires.ftc.teamcode.autos.CameraDetector_red;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Arrays;

@Autonomous(name = "testy_blue")

public class blue_otto_detection_test extends LinearOpMode {

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        int caz_inter=3;

        CameraDetector_blue camera = new CameraDetector_blue(OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1")));
        CameraDetector_blue.Result result = CameraDetector_blue.Result.NONE;
        while (opModeInInit()) {
            result = camera.detect();
            telemetry.addLine("Location" + result);
            telemetry.addData("cazus",camera.cazus);

            telemetry.update();
        }
        int caz=3;
        caz=camera.cazus;
        camera.stop();
        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()&&!isStopRequested()){
            telemetry.addData("caz",caz);
            telemetry.update();
        }


    }}