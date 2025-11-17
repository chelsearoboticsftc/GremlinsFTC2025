package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Limelight {

    private final Limelight3A limelight;

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
//        limelight.pipelineSwitch(0);
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
}
