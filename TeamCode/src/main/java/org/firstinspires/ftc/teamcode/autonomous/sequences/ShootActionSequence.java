package org.firstinspires.ftc.teamcode.autonomous.sequences;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.action.EnableShooterAction;
import org.firstinspires.ftc.teamcode.action.ExecuteSequenceAction;
import org.firstinspires.ftc.teamcode.action.SetMotorPowerAction;
import org.firstinspires.ftc.teamcode.action.WaitAction;
import org.firstinspires.ftc.teamcode.action.WaitUntilCanShootAction;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;

@Config
public class ShootActionSequence extends ActionSequence {

    public static double DELAY_AT_START = 1000;
    public static double DELAY_BETWEEN_SHOTS = 500;
    public static double DELAY_AFTER_SHOT = 500;
    public static double TIME_TO_SHOOT = 3000;

    public ShootActionSequence(int numOfRingsToShoot) {
        addAction(new EnableShooterAction(true));
        addAction(new WaitAction(DELAY_AT_START));
        addAction(new WaitUntilCanShootAction());
        addAction(new SetMotorPowerAction("escalator" ,1));
        addAction(new WaitAction(TIME_TO_SHOOT));
        addAction(new SetMotorPowerAction("escalator" ,0));
        addAction(new EnableShooterAction(false));
    }

    private static class ShootRingSequence extends ActionSequence {
        public ShootRingSequence () {
            addAction(new WaitUntilCanShootAction());
            addAction(new SetMotorPowerAction("escalator" ,1));
            addAction(new WaitAction(DELAY_BETWEEN_SHOTS));
            addAction(new SetMotorPowerAction("escalator" ,0));
            addAction(new WaitAction(DELAY_AFTER_SHOT));

        }
    }

}
