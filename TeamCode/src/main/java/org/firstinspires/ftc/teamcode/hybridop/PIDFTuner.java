package org.firstinspires.ftc.teamcode.hybridop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions.GamepadButtons;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions.GamepadType;
import org.firstinspires.ftc.teamcode.playmaker.HybridOp;

@TeleOp(name="PIDF Tuner")
public class PIDFTuner extends UltimateGoalHardware implements HybridOp {

    enum PIDFModifier {
        P,
        I,
        D
    }

    PIDFModifier currentModifier;
    final double INCREMENT = 10;

    @Override
    public void autonomous_loop() {

    }

    @Override
    public void teleop_loop() {
        if (gamepadActions.isFirstPress(GamepadType.ONE, GamepadButtons.a)) {
            currentModifier = PIDFModifier.P;
        } else if (gamepadActions.isFirstPress(GamepadType.ONE, GamepadButtons.b)) {
            currentModifier = PIDFModifier.I;
        } else if (gamepadActions.isFirstPress(GamepadType.ONE, GamepadButtons.x)) {
            currentModifier = PIDFModifier.D;
        }

        this.setShooterEnabled(gamepadActions.isToggled(GamepadType.ONE, GamepadButtons.y));
        this.escalator.setPower(gamepad1.right_bumper ? 1 : 0);

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_up)) {
            switch (currentModifier) {
                case P:
                    P += INCREMENT;
                    break;
                case I:
                    I += INCREMENT;
                    break;
                case D:
                    D += INCREMENT;
                    break;
            }
        } else if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_down)) {
            switch (currentModifier) {
                case P:
                    P -= INCREMENT;
                    break;
                case I:
                    I -= INCREMENT;
                    break;
                case D:
                    D -= INCREMENT;
                    break;
            }
        }

        telemetry.addData("[PIDFTuner] Current Modifier", currentModifier.toString());
    }

    @Override
    public void hybrid_loop() {

    }

    @Override
    public void run_loop() {

    }
}
