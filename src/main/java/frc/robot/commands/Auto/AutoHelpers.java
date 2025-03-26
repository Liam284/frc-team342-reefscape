// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.WristConstants.WristPositions;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Claw.Outtake;
import frc.robot.commands.Elevator.MoveElevatorToPosition;
import frc.robot.commands.Limelight.AutoAlign;
import frc.robot.commands.Wrist.WristToPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Vision.Limelight;

import static frc.robot.Constants.ElevatorConstants.L4_HEIGHT;

import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public class AutoHelpers {
    public static Command moveElevatorUpL4(Elevator elevator, Wrist wrist){
        return Commands.sequence(
            new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
            new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
            new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION),
            new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4, true).withTimeout(2)
        );
    }

    public static Command moveElevatorUpL3(Elevator elevator, Wrist wrist){
        return Commands.sequence(
            new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
            new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_MIDDLE_POSITION_L3),
            new WristToPosition(wrist, WristPositions.L3_WRIST_POSITION),
            new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_MIDDLE_POSITION_L3, true).withTimeout(2)
        );
    }

    public static Command moveElevatorUpL2(Elevator elevator, Wrist wrist){
        return Commands.sequence(
            new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
            new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_MIDDLE_POSITION_L2),
            new WristToPosition(wrist, WristPositions.MIDDLE_WRIST_POSITION),
            new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_MIDDLE_POSITION_L2, true).withTimeout(2)
        );
    }

    public static Command moveElevatorDownIntake(Elevator elevator, Wrist wrist){
        return Commands.sequence(
            new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
            new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1),
            new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION),
            new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1, true).withTimeout(2)
        );
    }
}
