// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ironriders.core;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.vision.VisionSubsystem;
import org.photonvision.PhotonCamera;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    public Robot() {
        robotContainer = new RobotContainer();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {

            autonomousCommand.cancel();
        }
    
        RobotContainer.init();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationInit() {
        PhotonCamera.setVersionCheckEnabled(false); // Silence camera not found warnings.

        // teleport to the center of the field on startup.
        DriveSubsystem.getSwerveDrive().getMapleSimDrive().orElseThrow().setSimulationWorldPose(FieldPositions.Field.CENTER);
        DriveSubsystem.getSwerveDrive().resetOdometry(FieldPositions.Field.CENTER);

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    }

    @Override
    public void simulationPeriodic() {
        VisionSubsystem.visionSim.update(DriveSubsystem.getSwerveDrive().field.getRobotPose());
    }
}
