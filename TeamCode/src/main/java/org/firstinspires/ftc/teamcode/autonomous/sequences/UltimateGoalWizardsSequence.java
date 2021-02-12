package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.action.DetectRingsAction;
import org.firstinspires.ftc.teamcode.action.DisableTFODAction;
import org.firstinspires.ftc.teamcode.action.EnableCollectorAction;
import org.firstinspires.ftc.teamcode.action.ExecuteSequenceAction;
import org.firstinspires.ftc.teamcode.action.IfActionResult;
import org.firstinspires.ftc.teamcode.action.LocalizerMoveAction;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.Localizer.RobotTransform;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class UltimateGoalWizardsSequence extends ActionSequence {

    public static RobotTransform[] SHOOTING_TRANSFORMS_NEAR_CENTER = {
            //new RobotTransform(DistanceUnit.INCH, 1, -18, 90),
            new RobotTransform(DistanceUnit.INCH, -12, -3, 71)
    };

    static final LocalizerMoveAction.FollowPathMethod FOLLOW_PATH_METHOD = LocalizerMoveAction.FollowPathMethod.FAST;
    static final RobotTransform SHOOTING_POSITION_NEAR_WALL = new RobotTransform(DistanceUnit.INCH, -3, -36, 90);
    static final RobotTransform PARKING_POSITION_NEAR_CENTER = new RobotTransform(DistanceUnit.INCH, 12, -12, 90);
    static final RobotTransform PARKING_POSITION_NEAR_WALL = new RobotTransform(DistanceUnit.INCH, 12, -36, 90);

    public UltimateGoalWizardsSequence(RobotHardware.Team team, UltimateGoalHardware.UltimateGoalStartingPosition startingPosition) {
        // Setup transforms
        RobotTransform[] shootingPosition;
        RobotTransform[] parkingPosition;

        if (team == RobotHardware.Team.RED && startingPosition == UltimateGoalHardware.UltimateGoalStartingPosition.LEFT) {
            // RED LEFT
            shootingPosition = SHOOTING_TRANSFORMS_NEAR_CENTER.clone();
            parkingPosition = new RobotTransform[] { PARKING_POSITION_NEAR_CENTER };
        } else if (team == RobotHardware.Team.RED && startingPosition == UltimateGoalHardware.UltimateGoalStartingPosition.RIGHT) {
            // RED RIGHT
            shootingPosition = new RobotTransform[] { SHOOTING_POSITION_NEAR_WALL };
            parkingPosition = new RobotTransform[] { PARKING_POSITION_NEAR_WALL };
        } else if (team == RobotHardware.Team.BLUE && startingPosition == UltimateGoalHardware.UltimateGoalStartingPosition.LEFT) {
            // BLUE LEFT;
            shootingPosition = new RobotTransform[] { Localizer.mirrorTransformOverTeamLine(SHOOTING_POSITION_NEAR_WALL) };
            parkingPosition = new RobotTransform[] { Localizer.mirrorTransformOverTeamLine(PARKING_POSITION_NEAR_WALL)};
        } else {
            // BLUE RIGHT
            shootingPosition = Localizer.mirrorTransformsOverTeamLine(SHOOTING_TRANSFORMS_NEAR_CENTER.clone());
            parkingPosition = new RobotTransform[] { Localizer.mirrorTransformOverTeamLine(PARKING_POSITION_NEAR_CENTER) };
        }

        // Add shooting offset
        RobotTransform finalShootingPosition = shootingPosition[shootingPosition.length - 1].copy();
        finalShootingPosition.heading += UltimateGoalHardware.SHOOTER_HEADING_OFFSET;
        shootingPosition[shootingPosition.length - 1] = finalShootingPosition;

        // Move to shooting position
        addAction(new LocalizerMoveAction(shootingPosition, UltimateGoalHardware.defaultLocalizerMoveParameters));

        // Shoot three rings into high goal
        addAction(new ExecuteSequenceAction(new ShootActionSequence(3)));

        // Park on center line
        addAction(new LocalizerMoveAction(parkingPosition, UltimateGoalHardware.defaultLocalizerMoveParameters));
    }
}
