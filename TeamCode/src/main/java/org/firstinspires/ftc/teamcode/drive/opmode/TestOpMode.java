package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class TestOpMode extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
//                .forward(DISTANCE)
//                .build();

        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-4, 31, Math.toRadians(90)))
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-43, 40, Math.toRadians(242)))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-9.8, 59.9, Math.toRadians(333)))
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(37.9, -16.8, Math.toRadians(238)))
                .build();

//        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
//                .back(DISTANCE)
//                .build();

        waitForStart();

        drive.followTrajectory(trajectory1);
        drive.followTrajectory(trajectory2);
        drive.followTrajectory(trajectory3);
        drive.followTrajectory(trajectory4);

//        while (opModeIsActive() && !isStopRequested()) {
//            drive.followTrajectory(trajectoryForward);
//            drive.followTrajectory(trajectoryBackward);
//        }
    }
}