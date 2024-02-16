package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.macros.MacroSequence;
import org.firstinspires.ftc.teamcode.macros.arm.IntakePoseMacro;
import org.firstinspires.ftc.teamcode.macros.arm.down.LowerArmMacro;
import org.firstinspires.ftc.teamcode.macros.arm.down.TuckWristDownMacro;
import org.firstinspires.ftc.teamcode.macros.arm.up.ArmToDumpPointMacro;
import org.firstinspires.ftc.teamcode.macros.arm.up.TuckWristForRiseMacro;
import org.firstinspires.ftc.teamcode.opmodes.auto.Webcam.PrimaryDetectionPipeline;
import org.firstinspires.ftc.teamcode.opmodes.auto.Webcam.Webcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.function.Supplier;

@Autonomous(name="RedEndzone Shortside SENSING Auto", group="aCompete")
public class CloseRedSENSING extends LinearOpMode {


    public Webcam webcam = new Webcam();
    public static final double PIXEL_RELEASE_POSITION = 0.5;
    public static final double PIXEL_HOLD_POSITION = 1.0;

    public static final Supplier<MacroSequence> TOWER_UP_SEQUENCE =
            () -> MacroSequence.compose(
                    "Lift And Dump Sequence",
                    //new IntakePoseMacro(),
                    new TuckWristForRiseMacro(),
                    new ArmToDumpPointMacro(
                            //new DumpBucketMacro(),
                    ));

    public static final Supplier<MacroSequence> TOWER_DOWN_SEQUENCE =
            () -> MacroSequence.compose(
                    "Lower and Tuck Sequence",
                    new LowerArmMacro(),
                    new TuckWristDownMacro(),
                    new IntakePoseMacro()
            );




    public void runOpMode() throws InterruptedException {
        RobotComponents.init(hardwareMap);


        RobotComponents.left_pixel_hold_servo.setPosition(PIXEL_HOLD_POSITION);
        RobotComponents.right_pixel_hold_servo.setPosition( PIXEL_HOLD_POSITION);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        webcam.initCamera(hardwareMap, PrimaryDetectionPipeline.Color.RED);

        int ElementLocation = 0;

        while (opModeInInit()) {
            telemetry.addData("Location: ", webcam.getLocation());
            telemetry.update();
            if (webcam.getLocation() == PrimaryDetectionPipeline.ItemLocation.CENTER) {
                telemetry.addLine("IS CENTER");
                ElementLocation = 1;
            } else if (webcam.getLocation() == PrimaryDetectionPipeline.ItemLocation.RIGHT) {
                telemetry.addLine("IS RIGHT");
                ElementLocation = 2;
            } else if (webcam.getLocation() == PrimaryDetectionPipeline.ItemLocation.LEFT) {
                telemetry.addLine("IS LEFT");

            }
        }

        waitForStart();
        if(isStopRequested()) {
            return;
        }



        Pose2d startPose = new Pose2d(11, -61, Math.toRadians(0));


        // LEFT


        TrajectorySequence closeLeftAuto = drive.trajectorySequenceBuilder(startPose)
                .back(28)
                .turn(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.front_intake_motor.setPower(1)) // Spit out
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.front_intake_motor.setPower(0)) // Stop outtake
                .back(38.5)
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> TOWER_UP_SEQUENCE.get().start())
                .waitSeconds(.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.left_pixel_hold_servo.setPosition(PIXEL_RELEASE_POSITION)) //Drops Pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.right_pixel_hold_servo.setPosition(PIXEL_RELEASE_POSITION)) //Drops Pixel
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> TOWER_DOWN_SEQUENCE.get().start())
                .waitSeconds(.25)
                .forward(2)
                .strafeLeft(25)
                .back(8)
                .build();


        //CENTER


        TrajectorySequence closeCenterAuto = drive.trajectorySequenceBuilder(startPose)
                .back(28)
                .turn(Math.toRadians(180 + .00000000000000001))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.front_intake_motor.setPower(1)) // Spit out
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.front_intake_motor.setPower(0)) // Stop outtake
                .turn(Math.toRadians(90))
                .back(38.5)
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> TOWER_UP_SEQUENCE.get().start())
                .waitSeconds(.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.left_pixel_hold_servo.setPosition(PIXEL_RELEASE_POSITION)) //Drops Pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.right_pixel_hold_servo.setPosition(PIXEL_RELEASE_POSITION)) //Drops Pixel
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> TOWER_DOWN_SEQUENCE.get().start())
                .waitSeconds(.25)
                .forward(2)
                .strafeLeft(25)
                .back(8)
                .build();


        // RIGHT

        TrajectorySequence closeRightAuto = drive.trajectorySequenceBuilder(startPose)
                .back(28)
                .turn(Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.front_intake_motor.setPower(1)) // Spit out
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.front_intake_motor.setPower(0)) // Stop outtake
                .turn(Math.toRadians(180+.000000000000000001))
                .back(38.5)
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> TOWER_UP_SEQUENCE.get().start())
                .waitSeconds(.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.left_pixel_hold_servo.setPosition(PIXEL_RELEASE_POSITION)) //Drops Pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> RobotComponents.right_pixel_hold_servo.setPosition(PIXEL_RELEASE_POSITION)) //Drops Pixel
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> TOWER_DOWN_SEQUENCE.get().start())
                .waitSeconds(.25)
                .forward(2)
                .strafeLeft(25)
                .back(8)
                .build();


                if (ElementLocation == 0) {

                    drive.followTrajectorySequence(closeLeftAuto);
                } else if (ElementLocation == 1) {
                    drive.followTrajectorySequence(closeCenterAuto);
                } else {
                    drive.followTrajectorySequence(closeRightAuto);
                }

                }




}


