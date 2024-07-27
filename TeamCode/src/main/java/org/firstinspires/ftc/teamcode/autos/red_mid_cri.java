package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pepeop.colectarev2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="red_mid_cri")
public class red_mid_cri extends LinearOpMode {
    public  oto oto = new oto(hardwareMap);
    ElapsedTime timez = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        colectarev2 colectarev2 = new colectarev2(hardwareMap);
        ElapsedTime exti = new ElapsedTime();
        boolean ext=false;
        ElapsedTime cronos = new ElapsedTime();
        double transfer =1;
        TrajectoryAccelerationConstraint accel = new ProfileAccelerationConstraint(40);

        //        detection_red detection = new detection_red(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(oto.start_red_mid);
        colectarev2.auto_init_top();
        TrajectorySequence unu_d = drive.trajectorySequenceBuilder(oto.start_red_mid)

                .lineToSplineHeading(oto.back_trav_red_intermediar)

                .turn(Math.toRadians(50))
                .back(2.3)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
                })
                .turn(Math.toRadians(-50))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2.transfer(colectarev2.dreapta_inter);
                })

                .build();

        TrajectorySequence unu_d1 = drive.trajectorySequenceBuilder(oto.preload_red_mid_dreapta)
                .lineToSplineHeading(oto.back_trav_perete_intermediu)
                .turn(Math.toRadians(90))
                .lineToSplineHeading(oto.back_trav_dreapta)
//                .lineToSplineHeading(oto.back_trav_dreapta_schema)
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    colectarev2.score();
                    colectarev2.culisanta_pozitie=120;


                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_orizont_invers);
                    colectarev2.culisante();
                })
                .lineToSplineHeading(oto.table_align_red_dreapta)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.back_trav_dreapta)
                .lineToSplineHeading(oto.table_red_park_2)
                .build();


        TrajectorySequence unu_m = drive.trajectorySequenceBuilder(oto.start_red_mid)

                .lineToSplineHeading(oto.back_trav_red_intermediar)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    colectarev2.transfer(colectarev2.dreapta_inter);
                })
                .build();
        TrajectorySequence unu_m1 = drive.trajectorySequenceBuilder(oto.preload_red_mid_mid)

                .lineToSplineHeading(oto.back_trav_perete_intermediu)
                .turn(Math.toRadians(90))
                .lineToSplineHeading(oto.back_trav_dreapta)
//                .lineToSplineHeading(oto.back_trav_dreapta_schema)
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    colectarev2.score();
                    colectarev2.culisanta_pozitie=120;


                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_orizont_invers);
                    colectarev2.culisante();
                })
                .lineToSplineHeading(oto.table_align_red_mid)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.back_trav_dreapta)
                .lineToSplineHeading(oto.table_red_park_2)
                .build();



        TrajectorySequence unu_s = drive.trajectorySequenceBuilder(oto.start_red_mid)

                .lineToSplineHeading(oto.back_trav_red_intermediar)
                .turn(Math.toRadians(65))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
                })
                .turn(Math.toRadians(-65))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2.transfer(colectarev2.dreapta_inter);
                    colectarev2.rotite.setPosition(colectarev2.rotite_colect);
                })


                .build();

        TrajectorySequence unu_s1 = drive.trajectorySequenceBuilder(oto.preload_red_mid_stanga)

                .lineToSplineHeading(oto.back_trav_perete_intermediu)
                .turn(Math.toRadians(90))
                .lineToSplineHeading(oto.back_trav_dreapta)
//                .lineToSplineHeading(oto.back_trav_dreapta_schema)
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    colectarev2.score();
                    colectarev2.culisanta_pozitie=120;


                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_orizont_invers);
                    colectarev2.culisante();
                })
                .lineToSplineHeading(oto.table_align_red_stanga)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.back_trav_dreapta)
                .lineToSplineHeading(oto.table_red_park_2)
                .build();




        CameraDetector_red camera = new CameraDetector_red(OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1")));
        CameraDetector_red.Result result = CameraDetector_red.Result.NONE;;
        while (opModeInInit()) {
            result = camera.detect();
            telemetry.addLine("Location" + result);
            telemetry.addData("cazus",camera.cazus);

            telemetry.update();
        }
        int caz=1;
        caz=camera.cazus;
        camera.stop();
        waitForStart();
        if(isStopRequested()) return;
        //3= stanga

//caz=3;
        colectarev2.transfer(colectarev2.transfer_preload);
        colectarev2.rotite.setPosition(colectarev2.rotite_preload);
        if(caz==1) {
            drive.followTrajectorySequence(unu_d);
            drive.followTrajectorySequence(unu_d1);

//            ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);
//
//        ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);

        }
        if(caz==2){
            drive.followTrajectorySequence(unu_m);
            drive.followTrajectorySequence(unu_m1);

        }
        if(caz==3){
            drive.followTrajectorySequence(unu_s);
            drive.followTrajectorySequence(unu_s1);

        }
        while(!isStopRequested() && opModeIsActive()){
            colectarev2.state_reset();
        }
    }}
