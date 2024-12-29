package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        double t = 23.5;

        // (NEW) 3 Specimen Auto
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(t*-0.5, t*2.5, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-9.73, 35.77), Math.toRadians(270))
                .waitSeconds(3.0)

                //perform preloaded specimen hang




                .setTangent(Math.toRadians(180))

                // Move to side
                .splineToConstantHeading(new Vector2d(-35, 36.16), Math.toRadians(270))



                // Move down
                .splineToConstantHeading(new Vector2d(-35, 12), Math.toRadians(270))



                // Move right
                .splineToConstantHeading(new Vector2d(-41, 12), Math.toRadians(90))
                // Move up
                .splineToConstantHeading(new Vector2d(-41, 50), Math.toRadians(90))
                // Move right
                .splineToConstantHeading(new Vector2d(-45, 50), Math.toRadians(270))
                // Move down
                .splineToConstantHeading(new Vector2d(-45, 12), Math.toRadians(270))
                // Move right
                .splineToConstantHeading(new Vector2d(-51, 12), Math.toRadians(90))
                // Move up
                .splineToConstantHeading(new Vector2d(-50, 50), Math.toRadians(90))
                // Move right
                .splineToConstantHeading(new Vector2d(-55, 50), Math.toRadians(270))
                // Move down
                .splineToConstantHeading(new Vector2d(-53, 12), Math.toRadians(270))
                // Move right
                .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(90))
                // Move up
                .splineToConstantHeading(new Vector2d(-60, 50), Math.toRadians(90))
                // Move back
                .splineToConstantHeading(new Vector2d(-55, 47), Math.toRadians(270))


                .splineToConstantHeading(new Vector2d(-42,60), Math.toRadians(180))


                .waitSeconds(5.0)






                /*
                // Prepare for specimens
                .splineTo(new Vector2d(-38, 42.5), Math.toRadians(90))
                .waitSeconds(1)
                // Head into Observation Zone
                .splineTo(new Vector2d(-38, 58), Math.toRadians(90))
                */


                /*  not for us
                // Pick up Specimen
                .waitSeconds(1)
                .setTangent(Math.toRadians(270))
                // Go to rung
                .splineToConstantHeading(new Vector2d(-9.73, 35.77), Math.toRadians(270))
                // Hang specimen
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                // Go Back to Observation Zone
                .splineToConstantHeading(new Vector2d(-38, 58), Math.toRadians(90))

                // Pick up Specimen
                .waitSeconds(1)
                .setTangent(Math.toRadians(270))
                // Go to rung
                .splineToConstantHeading(new Vector2d(-9.73, 35.77), Math.toRadians(270))
                // Hang specimen
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                // Go Back to Observation Zone
                .splineToConstantHeading(new Vector2d(-38, 58), Math.toRadians(90))

                // Pick up Specimen
                .waitSeconds(1)
                .setTangent(Math.toRadians(270))
                // Go to rung
                .splineToConstantHeading(new Vector2d(-9.73, 35.77), Math.toRadians(270))
                // Hang specimen
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                // Go Back to Observation Zone
                .splineTo(new Vector2d(-38, 58), Math.toRadians(90))
                */

                .build());








        // 4 Sample Auto
        /*
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(t*1.5, t*2.5, Math.toRadians(180)))
                .strafeTo(new Vector2d(t*1.5, t*2))
                .strafeToLinearHeading(new Vector2d(t*2, t*2), Math.toRadians(220))
                .strafeTo(new Vector2d(t*2.2, t*2.2))
                .strafeToLinearHeading(new Vector2d(t*2, t*2), Math.toRadians(220))
                .turnTo(Math.toRadians(270))
                // Cycle
                .turnTo(Math.toRadians(220))
                .strafeTo(new Vector2d(t*2.2, t*2.2))
                .strafeToLinearHeading(new Vector2d(t*2, t*2), Math.toRadians(220))
                .turnTo(Math.toRadians(295))
                // Cycle
                .turnTo(Math.toRadians(220))
                .strafeTo(new Vector2d(t*2.2, t*2.2))
                .strafeToLinearHeading(new Vector2d(t*2, t*2), Math.toRadians(220))
                .turnTo(Math.toRadians(315))
                // Cycle
                .turnTo(Math.toRadians(220))
                .strafeTo(new Vector2d(t*2.2, t*2.2))
                .strafeToLinearHeading(new Vector2d(t*2, t*2), Math.toRadians(220))

                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(t*2, t*0.5))
                .strafeTo(new Vector2d(t*1.2, t*0.5))

                .build());

         */
        /*
        // (OLD) 4 Specimen Auto
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(t*-0.5, t*2.5, Math.toRadians(270)))
                .lineToY(t*1.7)
                //.waitSeconds(1)
                .turn(Math.toRadians(-70))
                .lineToY(t*1.55)
                //.waitSeconds(1)
                .setTangent(Math.toRadians(180))
                // 140 is turning to Observation Zone, 200 is turning to samples
                .lineToXSplineHeading(t*-1.2, Math.toRadians(140))
                .lineToXSplineHeading(t*-1.5,Math.toRadians(200))
                .lineToXSplineHeading(t*-1.8, Math.toRadians(120))
                .lineToXSplineHeading(t*-2.1, Math.toRadians(200))
                .lineToXSplineHeading(t*-2.0, Math.toRadians(110))
                .turnTo(Math.toRadians(270))
                // Grabbing specimen pos
                .strafeTo(new Vector2d(t * -1.5, t * 2.5))
                // Scoring specimen pos
                .strafeTo(new Vector2d(t * -0.5, t * 1.7))
                // Grabbing specimen pos
                .strafeTo(new Vector2d(t * -1.5, t * 2.5))
                // Scoring specimen pos
                .strafeTo(new Vector2d(t * -0.5, t * 1.7))
                // Grabbing specimen pos
                .strafeTo(new Vector2d(t * -1.5, t * 2.5))
                // Scoring specimen pos
                .strafeTo(new Vector2d(t * -0.5, t * 1.7))

                // Park
                .strafeTo(new Vector2d(t * -2.0, t * 2.5))
                .build());
*/
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}