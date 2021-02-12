package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.action.ExecuteSequenceAction;
import org.firstinspires.ftc.teamcode.action.LocalizerMoveAction;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.Localizer.RobotTransform;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class NavigateAndShootRingsSequence extends ActionSequence {

    public static final Position RED_GOAL_ON_FIELD = new Position(DistanceUnit.INCH, 72, -36, 0, 0);

    public NavigateAndShootRingsSequence(RobotHardware.Team team, Position shootingPositionOnRedSide) {
        Position shootingPosition;
        Position goalPosition;
        if (team == RobotHardware.Team.RED) {
            shootingPosition = shootingPositionOnRedSide.toUnit(DistanceUnit.INCH);
            goalPosition = RED_GOAL_ON_FIELD.toUnit(DistanceUnit.INCH);
        } else {
            shootingPosition = Localizer.mirrorPositionOverTeamLine(shootingPositionOnRedSide.toUnit(DistanceUnit.INCH));
            goalPosition = Localizer.mirrorPositionOverTeamLine(RED_GOAL_ON_FIELD.toUnit(DistanceUnit.INCH));
        }
        double shootingAngle = Localizer.atan2InDegrees(shootingPosition, goalPosition);
        RobotTransform shootingTransform = new RobotTransform(shootingPosition, shootingAngle + UltimateGoalHardware.SHOOTER_HEADING_OFFSET);

        addAction(new LocalizerMoveAction(shootingTransform, UltimateGoalHardware.defaultLocalizerMoveParameters));
        addAction(new ExecuteSequenceAction(new ShootActionSequence(3)));
    }
}
