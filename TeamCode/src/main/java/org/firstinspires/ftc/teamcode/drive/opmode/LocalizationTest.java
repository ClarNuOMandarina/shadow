package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.autos.oto;

import org.firstinspires.ftc.teamcode.autos.oto_blue;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pepeop.colectarev2;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    public oto_blue oto = new oto_blue(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    colectarev2 colectare = new colectarev2(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(oto.start_red_mid);
        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("culi",colectare.culisanta.getCurrentPosition());
            telemetry.addData("capac",colectare.capac.getPosition());
            telemetry.addData("extindere",colectare.extindere.getCurrentPosition());
            telemetry.addData("grip_stng",colectare.griper_stanga.getPosition());
            telemetry.addData("grip_drpt",colectare.griper_dreapta.getPosition());
            telemetry.addData("perie_pozitie",colectare.perie_pozitie.getPosition());
            telemetry.addData("trands_dreapta",colectare.transfer_dreapta.getPosition());
            telemetry.addData("trands_stanga",colectare.transfer_stanga.getPosition());
            telemetry.addData("rotatie",colectare.rotite.getPosition());
            telemetry.addData("rotatie_grip",colectare.griper_rotatie.getPosition());
            telemetry.update();
        }
    }
}
