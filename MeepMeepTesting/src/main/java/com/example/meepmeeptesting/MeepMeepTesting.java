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
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        final Pose2d startingPos = new Pose2d(2 * gridSize, 2.2 * gridSize, Math.toRadians(217));
        final double posY = 0.5 * gridSize;

        double wallX = (2.1 * gridSize);
        if (posY < 0.2 * gridSize){
        wallX = (2.3 * gridSize);
        }
        myBot.runAction(myBot.getDrive().actionBuilder(startingPos)
                        .splineToLinearHeading(
                                new Pose2d(1 * gridSize, posY, Math.toRadians(0)),
                                Math.toRadians(-90)
                        )
                .setTangent(Math.toRadians(180))
                .lineToX(wallX)
                .setTangent(Math.toRadians(0))
                .lineToX((startingPos.position.x) - 3)
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(startingPos.position.y, startingPos.heading)
                .setTangent(Math.toRadians(180))
                .lineToX(startingPos.position.x)
                .build()
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