package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Limelight {

    private final Limelight3A limelight;
    private IMU imu;
    private double initialYaw;
    public Set<Integer> observedAprilTags = new HashSet<>();

    public static class TagDetection {
        public final int id;        // AprilTag ID
        public final double txDeg;  // horizontal offset (deg) from crosshair
        public final double tyDeg;  // vertical offset (deg) from crosshair
        public final double xInches;  // forward distance
        public final double yInches;  // lateral distance
        public final double distInches;

        public TagDetection(int id, double txDeg, double tyDeg, Position pos) {
            this.id = id;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.xInches = pos.x * 39.37;  // position is in meters, convert to inches
            this.yInches = pos.y * 39.37;
            this.distInches = Math.hypot(this.xInches, this.yInches);
        }

        @NonNull
        @SuppressLint("DefaultLocale")
        @Override
        public String toString() {
            return String.format("Tag %d: tx=%.2f°, ty=%.2f°, td=%.3f",
                    id, txDeg, tyDeg, distInches);
        }
    }

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public void initializeIMU(IMU imu, Pose2d pose) {
        this.imu = imu;
        this.imu.resetYaw();
        // keep track of absolute heading
        this.initialYaw = pose.heading.toDouble();
    }

    public List<TagDetection> getCurrentDetections() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return Collections.emptyList();
        }

        // get apriltag detections
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return Collections.emptyList();
        }

        // convert to TagDetection objects with the data we want
        List<TagDetection> detections = new ArrayList<>(fiducials.size());
        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();              // tag ID
            double tx = f.getTargetXDegrees();       // horizontal offset (deg)
            double ty = f.getTargetYDegrees();       // vertical offset (deg)
            Position pos = f.getTargetPoseRobotSpace().getPosition();
            detections.add(new TagDetection(id, tx, ty, pos));
        }

        return detections;
    }

    public double distanceToTag(int tagId) {
        List<TagDetection> detections = getCurrentDetections();
        for (TagDetection detection : detections) {
            if (detection.id == tagId) {
                return detection.distInches;
            }
        }
        // arbitrary large distance if tag not detected
        return 1000000;
    }

    public void updateTags() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducials : result.getFiducialResults()) {
                int id = fiducials.getFiducialId();
                this.observedAprilTags.add(id);
            }
        }
    }

    public boolean hasSeenTag(int tagId) {
        return this.observedAprilTags.contains(tagId);
    }

    @SuppressLint("DefaultLocale")
    public void updatePose(MecanumDrive drive, Telemetry telemetry, boolean updateLocalizer, int updateTries) {
        // tell limelight the orientation on every frame
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(angles.getYaw(AngleUnit.RADIANS) + initialYaw);

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducials : result.getFiducialResults()) {
                int id = fiducials.getFiducialId();
                this.observedAprilTags.add(id);
            }
            double [] stddev = result.getStddevMt2();
            Pose3D pose3d = result.getBotpose_MT2();
            Pose2d pose2d = new Pose2d(
                    pose3d.getPosition().x * 39.3701,  // meters to inches
                    pose3d.getPosition().y * 39.3701, // meters to inches
                    pose3d.getOrientation().getYaw()
            );

            telemetry.addData("LL pose", String.format("x=%.2f, y=%.2f, yaw=%.2f",
                    pose2d.position.x, pose2d.position.y, Math.toDegrees(pose2d.heading.toDouble())));
            telemetry.addData("LL stddev", String.format("x=%.1f, y%.1f, yaw=%.1f", stddev[0], stddev[1], Math.toDegrees(stddev[5])));
            Pose2d rrPose = drive.localizer.getPose();
            telemetry.addData("RR pose", String.format("x=%.2f, y=%.2f, yaw=%.2f",
                    rrPose.position.x, rrPose.position.y, Math.toDegrees(rrPose.heading.toDouble())));
            telemetry.addData("Detected tags", this.observedAprilTags.toString());
            if (updateLocalizer) {
                drive.localizer.setPose(pose2d);
            }
        } else if (updateLocalizer && updateTries > 0) {
            // try again
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
            }
            this.updatePose(drive, telemetry, true, updateTries - 1);
        }
    }
}
