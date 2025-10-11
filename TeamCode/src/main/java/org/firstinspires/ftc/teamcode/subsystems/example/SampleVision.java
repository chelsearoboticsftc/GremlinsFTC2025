package org.firstinspires.ftc.teamcode.subsystems.example;
// This sample vision subsystem only includes the reading of the main apriltag for goals
// it does not include obelisk detection or sorting at this time

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class SampleVision {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private double distance; //
    private VisionPortal visionPortal;
    private int team;   // 24 for red, 20 for blue
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    public SampleVision(HardwareMap hardwareMap) {
        //Constructor for the SampleVision class.  This code is called everytime you create
        //an object of this class type.  Rename to match your class name.

        //'this' keyword is to eliminate the confusion between objects/attributes which are part of and parameters
        // this class and with the same name.  It refers to objects in the SampleVision class in this case

        //Device names in hardwareMap.get(class,deviceName) must match names from Control Hub
        //configuration exactly.  This is the connection with the Control Hub Config

        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                // Calibration below is for Logitech C920 at 1920x1280 resolution
                // as captured by Rob Hoffman
                .setLensIntrinsics(1371.47, 1371.47, 1162.12, 417.84)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCamera(BuiltinCameraDirection.BACK);

        // Choose a camera resolution. Not al cameras support all resolutions.
        builder.setCameraResolution(new Size(1920,1080));
        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }

    /* Standard functions.  All Chelsea Robotics subsystems shall have init() and update() these  *
     * methods defined. Leave empty if not needed!                                                */
    public void init() {
        /* Call this method at the start of your opmode logic once to execute any logic you       *
         * want to be called on initialization. If none, leave empty!                             */
    }

    public void update() {
        //Call this method each time your opmode logic loops (i.e. inside while(opModeIsActive()){}
        //to execute any logic you want to be called periodically. If none, leave empty!

    }

    public double getDistanceToGoal(int team) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections.size() > 0) {
            // Detections found, cycle through them to get the one that is needed
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {

                    if (!detection.metadata.name.contains("Obelisk") && detection.id == team) {
                        //Don't include obelisk detections
                        //20 for blue, 24 for red!
                        distance = detection.ftcPose.range;
                    }
                }
            }


        } else {

        }
        return distance;
    }
}

