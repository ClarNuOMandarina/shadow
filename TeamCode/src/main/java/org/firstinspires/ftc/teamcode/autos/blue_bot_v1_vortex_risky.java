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

@Autonomous(name="blue_bot_v1_vortex_risky")
public class blue_bot_v1_vortex_risky extends LinearOpMode {
    public  oto_blue oto = new oto_blue(hardwareMap);
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
        drive.setPoseEstimate(oto.start_red_bot);
        colectarev2.auto_init_top();
        TrajectorySequence unu_d = drive.trajectorySequenceBuilder(oto.start_red_bot)

                .lineToSplineHeading(oto.preload_red_bot_dreapta)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2.griper_dreapta.setPosition(colectarev2.griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.red_inner_stack)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{
                    colectarev2.perie_pozitie.setPosition(colectarev2.stack5);
                    colectarev2.perie.setPower(-1);                })
                .build();

        TrajectorySequence unu_d1 = drive.trajectorySequenceBuilder(oto.red_inner_stack)

                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_hopa);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_default);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_open);
                    colectarev2.perie.setPower(1);

                })
                .UNSTABLE_addTemporalMarkerOffset(1.6, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_inter);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()-> {
                    colectarev2.perie.setPower(0);
                })
                .lineToSplineHeading(oto.back_trav_red_intermediar)
                .lineToSplineHeading(oto.front_trav_red)
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    colectarev2.score();
                    colectarev2.culisanta_pozitie=120;


                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_orizont_invers);
                    colectarev2.rotite.setPosition(colectarev2.rotite_score_preload);
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

                .lineToSplineHeading(oto.front_trav_red)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.extindere_state=1;
                    colectarev2.extindere();
                    colectarev2.perie_pozitie.setPosition(colectarev2.stack4);
                    colectarev2.perie.setPower(-1);
                })
                .lineToSplineHeading(oto.back_trav_red)
                .build();
        TrajectorySequence unu_s2 = drive.trajectorySequenceBuilder(oto.back_trav_red)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_hopa);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_default);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_open);
                    colectarev2.perie.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.35, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_inter);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()-> {
                    colectarev2.perie.setPower(0);
                })
                .lineToSplineHeading(oto.back_trav_red_intermediar)
                .lineToSplineHeading(oto.front_trav_red)
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    colectarev2.score();
                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_drept_invers);
//                    colectarev2.rotite.setPosition(colectarev2.rotite_score_preload);
                    colectarev2.culisanta_pozitie=1800;
                    colectarev2.culisante();
                })
                .lineToSplineHeading(oto.table2_align_red_mid)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    colectarev2.state_reset();
                })

                .lineToSplineHeading(oto.front_trav_red)
                .lineToSplineHeading(oto.back_trav_red_intermediar)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.extindere_state=1;
                    colectarev2.extindere();
                    colectarev2.perie_pozitie.setPosition(colectarev2.stack4);
                    colectarev2.perie.setPower(-1);
                })
                .lineToSplineHeading(oto.back_trav_red)

                .build();
        TrajectorySequence unu_s3 = drive.trajectorySequenceBuilder(oto.back_trav_red)
                .UNSTABLE_addTemporalMarkerOffset(0+transfer, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_hopa);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2+transfer, ()-> {
                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6+transfer, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_default);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1+transfer, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_open);
                    colectarev2.perie.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.35+transfer, ()-> {
                    colectarev2.perie.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()-> {
                    colectarev2.perie.setPower(0);
                })
                .lineToSplineHeading(oto.back_trav_red_intermediar)
                .lineToSplineHeading(oto.front_trav_red)
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    colectarev2.score();
                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_drept);
                    colectarev2.culisanta_pozitie=1800;
                    colectarev2.culisante();                })
                .lineToSplineHeading(oto.table2_align_red_mid)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.perie.setPower(0);
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.red_stack_score_stanga_park)

                .build();
        TrajectorySequence unu_m = drive.trajectorySequenceBuilder(oto.start_red_bot)
                .lineToSplineHeading(oto.preload_red_bot_mid)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2.griper_dreapta.setPosition(colectarev2.griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.red_inner_stack)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{
                    colectarev2.perie_pozitie.setPosition(colectarev2.stack5);
                    colectarev2.perie.setPower(-1);                })
                .build();
        TrajectorySequence unu_m1 = drive.trajectorySequenceBuilder(oto.red_inner_stack)

                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_hopa);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_default);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_open);
                    colectarev2.perie.setPower(1);

                })
                .UNSTABLE_addTemporalMarkerOffset(1.6, ()-> {
                    colectarev2.perie.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()-> {
                    colectarev2.perie.setPower(0);
                })
                .lineToSplineHeading(oto.back_trav_red_intermediar)
                .lineToSplineHeading(oto.front_trav_red)
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    colectarev2.score();
                    colectarev2.culisanta_pozitie=120;

                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_drept);
                    colectarev2.culisante();
                })
                .lineToSplineHeading(oto.table_align_red_mid)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.perie.setPower(0);

                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.front_trav_red)
                .lineToSplineHeading(oto.back_trav_red_intermediar)

                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.extindere_state=1;
                    colectarev2.extindere();

                    colectarev2.perie_pozitie.setPosition(colectarev2.stack4);
                    colectarev2.perie.setPower(-1);
                })
                .lineToSplineHeading(oto.back_trav_red)
                .build();


        TrajectorySequence unu_s = drive.trajectorySequenceBuilder(oto.start_red_bot)

                .lineToSplineHeading(oto.preload_red_bot_stanga)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2.griper_dreapta.setPosition(colectarev2.griper_dreapta_close);
                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.red_inner_stack)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{
                    colectarev2.perie_pozitie.setPosition(colectarev2.stack5);
                    colectarev2.perie.setPower(-1);                })
                .build();

        TrajectorySequence unu_s1 = drive.trajectorySequenceBuilder(oto.red_inner_stack)

                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_hopa);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_default);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_open);
                    colectarev2.perie.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.35, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_inter);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()-> {
                    colectarev2.perie.setPower(0);
                })
                .lineToSplineHeading(oto.back_trav_red_intermediar)
                .lineToSplineHeading(oto.front_trav_red)
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    colectarev2.score();
                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_drept_invers);
//                    colectarev2.rotite.setPosition(colectarev2.rotite_score_preload);
                    colectarev2.culisanta_pozitie=1800;
                    colectarev2.culisante();
                })
                .lineToSplineHeading(oto.table2_align_red_mid)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    colectarev2.state_reset();
                })

                .lineToSplineHeading(oto.front_trav_red)
                .lineToSplineHeading(oto.back_trav_red_intermediar)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.extindere_state=1;
                    colectarev2.extindere();
                    colectarev2.perie_pozitie.setPosition(colectarev2.stack4);
                    colectarev2.perie.setPower(-1);
                })
                .lineToSplineHeading(oto.back_trav_red)

                .build();



        // dreapta
        //        if(!isStarted())
        //        {
        //            while(!isStarted()){
        //                detection.wallhack();
        //                telemetry.addData("case",detection.d);
        //                telemetry.update();sleep(20);}
        //        }
        CameraDetector_blue camera = new CameraDetector_blue(OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1")));
        CameraDetector_blue.Result result = CameraDetector_blue.Result.NONE;;
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


        colectarev2.transfer(colectarev2.transfer_preload);
        colectarev2.rotite.setPosition(colectarev2.rotite_preload);
        if(caz==3) {
            drive.followTrajectorySequence(unu_d);
            colectarev2.perie_pozitie.setPosition(colectarev2.stack5);
            colectarev2.perie.setPower(-1);
            cronos.reset();
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<2)
            {   }
            colectarev2.perie.setPower(0);
//        ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);

            drive.followTrajectorySequence(unu_s2);
            colectarev2.perie_pozitie.setPosition(colectarev2.stack4);
            colectarev2.perie.setPower(-1);
            cronos.reset();
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<1)
                colectarev2.perie_pozitie.setPosition(colectarev2.stack3);
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<1.7)
            {   }
            colectarev2.perie.setPower(0);
            colectarev2.extindere_state=0;
            colectarev2.extindere();
//            ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);
            drive.followTrajectorySequence(unu_s3);
//
//        ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);

        }
        if(caz==2){
            drive.followTrajectorySequence(unu_m);
            colectarev2.perie_pozitie.setPosition(colectarev2.stack5);
            colectarev2.perie.setPower(-1);
            cronos.reset();
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<2)
            {   }
            colectarev2.perie.setPower(0);
//        ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);

            drive.followTrajectorySequence(unu_s2);
            colectarev2.perie_pozitie.setPosition(colectarev2.stack4);
            colectarev2.perie.setPower(-1);
            cronos.reset();
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<1)
                colectarev2.perie_pozitie.setPosition(colectarev2.stack3);
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<1.7)
            {   }
            colectarev2.perie.setPower(0);
            colectarev2.extindere_state=0;
            colectarev2.extindere();
//            ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);
            drive.followTrajectorySequence(unu_s3);

//
//        ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);

        }
        if(caz==1){
            drive.followTrajectorySequence(unu_s);
            colectarev2.perie_pozitie.setPosition(colectarev2.stack5);
            colectarev2.perie.setPower(-1);
            cronos.reset();
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<2)
            {   }
            colectarev2.perie.setPower(0);
//        ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);

            drive.followTrajectorySequence(unu_s2);
            colectarev2.perie_pozitie.setPosition(colectarev2.stack4);
            colectarev2.perie.setPower(-1);
            cronos.reset();
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<1)
                colectarev2.perie_pozitie.setPosition(colectarev2.stack3);
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<2.2)
            {   }
            colectarev2.perie.setPower(0);
            colectarev2.extindere_state=0;
            colectarev2.extindere();
//            ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);
            drive.followTrajectorySequence(unu_s3);

        }
        while(!isStopRequested() && opModeIsActive()){
            colectarev2.state_reset();
        }
    }}
