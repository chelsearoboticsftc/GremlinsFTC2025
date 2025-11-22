package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        final double gridSize = 23.5;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16, 16)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        final Pose2d startingPos = new Pose2d(2.1 * gridSize, 2.3 * gridSize, Math.toRadians(225));
        final double posY = .5 * gridSize;
        myBot.runAction(myBot.getDrive().actionBuilder(startingPos)
                        .lineToX(.5 * gridSize)
                        .turn(Math.toRadians(35))
                        .lineToY(posY)
                        .turn(Math.toRadians(80))
                        .lineToX(2.3 * gridSize)
                        .setReversed(true)
                        .lineToX(0.5 * gridSize)
                        .turn(Math.toRadians(-90))
                        .lineToY(0.5 * gridSize)
                        .turn(Math.toRadians(-35))
                        .lineToX(startingPos.position.x)
                        .build()
//                .splineTo(
//                        new Vector2d(-2.1 * gridSize, rowY + 2),
//                        Math.toRadians(-180)
//                )
//                // reverse back to starting pose
//                .setReversed(true)
//                .splineTo(startingPos.position, startingPos.heading.plus(Math.toRadians(180)))
//                .build()
        );

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