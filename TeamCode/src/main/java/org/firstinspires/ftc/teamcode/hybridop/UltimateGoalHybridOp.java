package org.firstinspires.ftc.teamcode.hybridop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.autonomous.sequences.NavigateAndShootPowerTargetsSequence;
import org.firstinspires.ftc.teamcode.autonomous.sequences.NavigateAndShootRingsSequence;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;
import org.firstinspires.ftc.teamcode.playmaker.HybridOp;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public abstract class UltimateGoalHybridOp extends UltimateGoalHardware implements HybridOp {

    @TeleOp(name = "Ultimate Goal Hybrid Op - RED")
    public static class UltimateGoalHybridOpRed extends UltimateGoalHybridOp {
        @Override
        Team getTeam() {
            return Team.RED;
        }
    }

    @TeleOp(name = "Ultimate Goal Hybrid Op - BLUE")
    public static class UltimateGoalHybridOpBlue extends UltimateGoalHybridOp {
        @Override
        Team getTeam() {
            return Team.BLUE;
        }
    }

    float omniDrivePower = 1f;
    boolean slowMode = false;
    float slowModeMultiplier = 0.5f;

    abstract RobotHardware.Team getTeam();

    @Override
    public void autonomous_loop() {

    }

    @Override
    public void teleop_loop() {

        // region slowmode {...}

        // Slowmode when shooter running or bumper_left toggled
        slowMode = gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.bumper_left) || shooter.getPower() > 0;

        // endregion

        omniDrive.dpadMove(gamepad1, slowMode ? omniDrivePower * slowModeMultiplier : omniDrivePower, false);
        // endregion



        if (gamepad1.back) {
            collector.setPower(0);
            if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.y)) {
                localizer.syncEncodersWithVuforia();
            }
        } else {
            if (gamepad1.start) {
                collector.setPower(-1);
            } else if (gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.a)) {
                collector.setPower(1);
            } else {
                collector.setPower(0);
            }

            if (gamepad1.b  && this.canShoot()) {
                escalator.setPower(1);
            } else {
                escalator.setPower(0);
            }

            extendWobbleGoal = gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.x);
            if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.x) && extendWobbleGoal) {
                gamepadActions.setToggleStateFor(false, GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.a);
            }

            this.setShooterEnabled(gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.y));
        }




        // region wobbleGoal {...}

        if (extendWobbleGoal) {
            if (gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.bumper_right)) {
                wobbleServo.setPosition(1);
            } else {
                wobbleServo.setPosition(0);
            }
        } else {
            gamepadActions.setToggleStateFor(false, GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.bumper_right);
        }

        // endregion

        // Collector should not run if shooter is running
        if (shooter.getPower() > 0) {
            collector.setPower(0);
        }

    }

    @Override
    public void hybrid_loop() {
        // Navigate and shoot rings
        gamepadActions.setDisableToggle(false);
        if (gamepad1.back) {
            gamepadActions.setDisableToggle(true);
            if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.a)) {
                hybridOpExecutor.executeActionSequence(new NavigateAndShootRingsSequence(getTeam(), UltimateGoalHardware.SHOOTING_POSITION_RED_NEAR_CENTER), true, true);
            }

            if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.b)) {
                hybridOpExecutor.executeActionSequence(new NavigateAndShootPowerTargetsSequence(getTeam()), true, true);
            }
        }
    }

    @Override
    public void init() {
        super.init();
        this.initializeForHybridOp(this);
    }

    @Override
    public void run_loop() {
        telemetry.addData("[UGHO] Slow Mode", slowMode);
    }
}
