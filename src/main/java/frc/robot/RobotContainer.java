// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
 
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.BlueCenterShoot1;
import frc.robot.autos.BlueCenterShootMiddle4;
import frc.robot.autos.BlueCenterStealMiddle;
import frc.robot.autos.BlueCloseLeft;
import frc.robot.autos.BlueCloseLeftStealLeft;
import frc.robot.autos.BlueCloseLeftStealLeftLineupLeft;
import frc.robot.autos.BlueLeftJustMove;
import frc.robot.autos.BlueRightJustMove;
import frc.robot.autos.BlueRightMoveAndSteal;
import frc.robot.autos.ExampleAuto;
import frc.robot.autos.RedCenterShoot1;
import frc.robot.autos.RedCenterShootMiddle4;
import frc.robot.autos.RedCenterStealMiddle;
import frc.robot.autos.RedCloseLeft;
import frc.robot.autos.RedCloseLeftStealLeft;
import frc.robot.autos.RedCloseLeftStealLeftLineupLeft;
import frc.robot.autos.RedLeftJustMove;
import frc.robot.autos.RedRightJustMove;
import frc.robot.commands.AutoArm;
import frc.robot.commands.ManualArm;
import frc.robot.commands.ManualHooks;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualLeftHook;
import frc.robot.commands.ManualRightHook;
import frc.robot.commands.ManualShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hooks;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightHelpers;

public class RobotContainer {
 
  // Variables
  private static final double MaxSpeed = 5; // 6 meters per second desired top speed
  private static final double MaxAngularRate = 4 * Math.PI; // Half a rotation per second max angular velocity
  SendableChooser<Command> AutoSelect = new SendableChooser<Command>();
 
  // Controllers
  private final CommandXboxController Player1 = new CommandXboxController(0);
  private final CommandXboxController Player2 = new CommandXboxController(1);
 
  // Subsystems
  public final Drivetrain drivetrain = TunerConstants.DriveTrain;  
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final Hooks hooks = new Hooks();

  // Objects for Tele-op Drive
  public SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  public SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I DON'T want field-centric
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    // Basic Drive Command
    drivetrain.setDefaultCommand( 
        drivetrain.applyRequest(() -> driveFieldCentric
        .withVelocityX(Player1.getLeftY() * MaxSpeed * .85)  
        .withVelocityY(Player1.getLeftX()* MaxSpeed * .85) 
        .withRotationalRate((-Player1.getRightX() * MaxAngularRate)) 
    ));

    // Buttons
    Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), new Rotation2d(135)))));//James changed from Leftbumper 2/24/2024
 
    // Intake and Shoot
    Player1.leftTrigger().whileTrue(new SequentialCommandGroup(
    new AutoArm(arm, 66),
    new ManualIntake(intake, arm, 45)
    ));
    Player1.y().whileTrue(new ManualIntake(intake, arm, -45)); //Rotations per second
    Player1.rightTrigger().whileTrue(new ManualShoot(arm, 300));//120 velocity

    // Limelight Intake
    Player1.leftBumper().whileTrue( 
        new ParallelCommandGroup(
        new ManualIntake(intake, arm, 45),
        drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(1.5)  
        .withVelocityY(0) 
        .withRotationalRate(-LimelightHelpers.getTX("limelight-ri") / 14))));

    // Arm
    Player1.a().onTrue(new AutoArm(arm, 66));
    Player1.x().onTrue(new AutoArm(arm, 0));

    Player1.povUp().whileTrue(new ManualArm(arm, .2));
    Player1.povDown().whileTrue(new ManualArm(arm, -.2));
    
    // Hooks
    Player2.a().whileTrue(new ManualHooks(hooks, 1));
    Player2.y().whileTrue(new ManualHooks(hooks, -1));

    Player2.leftBumper().whileTrue(new ManualLeftHook(hooks, -.80));
    Player2.leftTrigger().whileTrue(new ManualLeftHook(hooks, .80));
    Player2.rightBumper().whileTrue(new ManualRightHook(hooks, -.80));
    Player2.rightTrigger().whileTrue(new ManualRightHook(hooks, .80));
 
    // Registers the Telemetry
    drivetrain.registerTelemetry(logger::telemeterize);

    // Auto Options
    AutoSelect.setDefaultOption("Blue Center Steal", new BlueCenterStealMiddle(drivetrain, arm, intake));
    AutoSelect.addOption("Red Center Steal", new RedCenterStealMiddle(drivetrain, arm, intake));
    AutoSelect.addOption("Blue Center Shoot 4", new BlueCenterShootMiddle4(drivetrain, this, arm, intake));
    AutoSelect.addOption("Red Center Shoot 4", new RedCenterShootMiddle4(drivetrain, this, arm, intake));
    AutoSelect.addOption("Blue Close Left, Steal Left, and Lineup Left", new BlueCloseLeftStealLeftLineupLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Red Close Left, Steal Left, and Lineup Left", new RedCloseLeftStealLeftLineupLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Blue Close Left", new BlueCloseLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Red Close Left", new RedCloseLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Blue Close Left and Steal Left", new BlueCloseLeftStealLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Red Close Left and Steal Left", new RedCloseLeftStealLeft(drivetrain,arm,intake));
    AutoSelect.addOption("Blue Center Shoot 1", new BlueCenterShoot1(drivetrain,arm,intake));
    AutoSelect.addOption("Red Center Shoot 1", new RedCenterShoot1(drivetrain,arm,intake));
    AutoSelect.addOption("Blue Right Just Move", new BlueRightJustMove(drivetrain,arm,intake));
    AutoSelect.addOption("Red Right Just Move", new RedRightJustMove(drivetrain,arm,intake));
    AutoSelect.addOption("Red Left Just Move", new RedLeftJustMove(drivetrain,arm,intake));
    AutoSelect.addOption("Blue Left Just Move", new BlueLeftJustMove(drivetrain,arm,intake));
      AutoSelect.addOption("Blue Right Move and Steal", new BlueRightMoveAndSteal(drivetrain,arm,intake));
    SmartDashboard.putData("Select Auto", AutoSelect);

  }

  public RobotContainer() {
    
    // Set the button bindings
    configureBindings();
 
  }
 
  public Command getAutonomousCommand() {

    return AutoSelect.getSelected();
    
  }

  public Command LimelightIntake() {

    return new ParallelCommandGroup(

      new ManualIntake(intake, arm, 45),
        drivetrain.applyRequest(() -> driveFieldCentric
        .withVelocityX(1.5)  
        .withVelocityY(0) 
        .withRotationalRate(-LimelightHelpers.getTX("limelight-ri") / 14))

    );

  }

}
