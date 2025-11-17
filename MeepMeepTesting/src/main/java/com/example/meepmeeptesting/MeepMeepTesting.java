package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class MeepMeepTesting {

    public static TrajectorySequence builder(DriveShim drive, double rowY) {
        final double gridSize = 23.5;
        final Pose2d startingPos = new Pose2d(-2.1 * gridSize, 2.1 * gridSize, Math.toRadians(-35));
        return drive.trajectorySequenceBuilder(startingPos)
            .splineToLinearHeading(
                    new Pose2d(-2. * gridSize, rowY + 2, Math.toRadians(-160)),
                    Math.toRadians(-160)
            )
            // reverse back to starting pose
            .setReversed(true)
            .splineTo(startingPos.vec(), startingPos.getHeading() + Math.PI)
            .build();
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        final double gridSize = 23.5;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16, 16)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> MeepMeepTesting.builder(drive, .5 * gridSize));
        // row positions: 0.5, -0.5, and -1.5 * gridSize


        Image img = null;
        try {
            img = ImageIO.read(new File("./MeepMeepTesting/src/main/resources/decode.bmp"));
            System.out.println(img);
        }
        catch(IOException ignored) {}
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}