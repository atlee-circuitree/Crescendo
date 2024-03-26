// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AlwaysRunningIntake;
import frc.robot.commands.AutoArm;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualShoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightHelpers;

public class BackfeedBlueCenterShoot4 extends SequentialCommandGroup {
  
  Drivetrain m_drivetrain;
  RobotContainer m_RobotContainer;

  public BackfeedBlueCenterShoot4(Drivetrain Drivetrain, RobotContainer RobotContainer, Arm Arm, Intake Intake) {
  
    m_drivetrain = Drivetrain;
    m_RobotContainer = RobotContainer;

    addCommands(

      // First Shot

      new ManualShoot(Arm, 120), // Shoot your preload
      InitialPose("CSBackfeed1", false), // Set your inital field position

      // Second Shot

      new ParallelCommandGroup( // Go to the speaker to shoot our note
        new ManualIntake(Intake, Arm, 120), 
        ChoreoPathing("CSBackfeed1", false)
      ),
      new ManualIntake(Intake, Arm, 120).withTimeout(1.5), 
      new ManualShoot(Arm, 120),

      // Third Shot

      new ParallelCommandGroup( // Go to the speaker to shoot our note
      new ManualIntake(Intake, Arm, 120), 
      ChoreoPathing("CSBackfeed2", false)
      ),
      new ManualIntake(Intake, Arm, 120).withTimeout(1.5), 
      new ManualShoot(Arm, 120),

      // Fourth Shot

      new ParallelCommandGroup( // Go to the speaker to shoot our note
      new ManualIntake(Intake, Arm, 120), 
      ChoreoPathing("CSBackfeed3", false)
      ),
      new ManualIntake(Intake, Arm, 120).withTimeout(1.5), 
      new ManualShoot(Arm, 120),

      // Use what time is left to get a note using limelight

      ChoreoPathing("CSBackfeed4", false),

      new ParallelCommandGroup( // Drive toward the center close note until the limelight loses detection
        new ManualIntake(Intake, Arm, 120), // Enables intake
        m_drivetrain.applyRequest(() -> RobotContainer.driveRobotCentric // Robot centric drive command that adjusts off of the lightlight
          .withVelocityX(1.5)  
          .withVelocityY(-LimelightHelpers.getTX("limelight-ri") / 30) 
          .withRotationalRate(0))
      ).until(() -> -LimelightHelpers.getTX("limelight-ri") / 30 == 0),

      new ParallelCommandGroup( // Go to the speaker to shoot our note
      new ManualIntake(Intake, Arm, 120), 
      ChoreoPathing("CSBackfeed5", false)
      ).withTimeout(3.3)
 
 
 
    );

  }

  private Command ChoreoPathing(String Trajectory, boolean IsRed) {

    return Choreo.choreoSwerveCommand(

      Choreo.getTrajectory(Trajectory), 
      () -> (m_drivetrain.getState().Pose), 
      Constants.AutoDrivePID, Constants.AutoDrivePID, Constants.AutoTurnPID, 
      (ChassisSpeeds speeds) -> m_drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> IsRed, 
      m_drivetrain

    ).withTimeout(Choreo.getTrajectory(Trajectory).getTotalTime() * 1.2);

  }
 
  private Command InitialPose(String Trajectory, boolean IsRed) {

    return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(Choreo.getTrajectory(Trajectory).getInitialPose()));

  }

}
