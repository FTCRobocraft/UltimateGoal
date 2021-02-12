package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.teamcode.action.EnableShooterAction;
import org.firstinspires.ftc.teamcode.action.ExecuteSequenceAction;
import org.firstinspires.ftc.teamcode.action.SetMotorPowerAction;
import org.firstinspires.ftc.teamcode.action.WaitAction;
import org.firstinspires.ftc.teamcode.action.WaitUntilCanShootAction;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;

public class ShootActionSequence extends ActionSequence {

    public ShootActionSequence(int numOfRingsToShoot) {
        addAction(new EnableShooterAction(true));
        addAction(new WaitUntilCanShootAction());
        addAction(new WaitAction(1500));
        addAction(new ExecuteSequenceAction(new ShootRingSequence(), 3));
        addAction(new EnableShooterAction(false));
    }

    private static class ShootRingSequence extends ActionSequence {
        public ShootRingSequence () {
            addAction(new WaitUntilCanShootAction());
            addAction(new SetMotorPowerAction("escalator" ,1));
            addAction(new WaitAction(1200));
            addAction(new SetMotorPowerAction("escalator" ,0));
            addAction(new WaitAction(1500));

        }
    }

}
