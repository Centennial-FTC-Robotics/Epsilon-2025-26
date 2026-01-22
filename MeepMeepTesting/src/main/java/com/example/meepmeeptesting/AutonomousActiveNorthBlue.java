package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.*;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutonomousActiveNorthBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        double t = 23.5;

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(t * -2, t * -2, Math.toRadians(0)))
                .waitSeconds(3.0)
                .splineToConstantHeading(new Vector2d(t * -2, t * -0.5), Math.toRadians(-180))
                .turnTo(Math.toRadians(-115))
                .waitSeconds(3.0) // shooting
                .splineToLinearHeading(new Pose2d(t * -0.5, t * -1.5, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(t * -0.5, t * -2.25), Math.toRadians(-90))
                .waitSeconds(2.0)
                .lineToYLinearHeading(t * -2, Math.toRadians(-90))
                .lineToYLinearHeading(t * -1.5, Math.toRadians(-270))
                .splineToLinearHeading(new Pose2d(t * -2, t * -0.5, Math.toRadians(-115)), Math.toRadians(-180))
                .waitSeconds(2.0)
                .splineToLinearHeading(new Pose2d(t * 0.5, t * -1.5, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(t * 0.5, t * -2.25), Math.toRadians(-90))
                .waitSeconds(2.0)
                .lineToYLinearHeading(t * -2, Math.toRadians(-90))
                .lineToYLinearHeading(t * -1.5, Math.toRadians(-270))
                .splineToLinearHeading(new Pose2d(t * -2, t * -0.5, Math.toRadians(-115)), Math.toRadians(-180))
                .waitSeconds(2.0)
                .splineToLinearHeading(new Pose2d(t * 0, t * -1, Math.toRadians(-180)), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}