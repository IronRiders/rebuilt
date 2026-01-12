// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ironriders.core;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

    private final RobotContainer robotContainer;

    private Command autonomousCommand;

    /**
     * Runs when the robot starts.
     *
     * <ol>
     * <li>Initializes Robot Container
     * <li>Starts web server at port 5800
     * <li>Starts automatic capture on camera server
     * </ol>
     */
    public Robot() {
        robotContainer = new RobotContainer();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        CameraServer.startAutomaticCapture();
    }

    /**
     * Runs periodically, <em>only</em> calls {@link CommandScheduler#getInstance()} with the .run()
     * method.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putData("Command Schedule", CommandScheduler.getInstance());
    }

    @Override
    public void teleopPeriodic() {} // Make it stop yelling at me

    /**
     * If you start in autonomous mode, this is the first method that runs. Calls
     * {@link Robot#generalInit()} for startup functions, has some basic autonomous logic that calls
     * {@link RobotContainer#schedule()} on {@link RobotContainer#getAutonomousCommand()}.
     */
    @Override
    public void autonomousInit() {
        generalInit();

        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** Only calls {@link Robot#generalInit()}, no other logic. */
    @Override
    public void teleopInit() {
        generalInit();
    }

    @Override
    public void simulationInit() {
        generalInit();
    }

    @Override
    public void teleopExit() {
        generalInit();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * Initialization that applies to autonomous and teleop. Contains startup methods for various
     * parts of the robot, not part of WPILib. Created because we were tired of trying to keep all
     * the startup functions updated for both startup methods.
     */
    private void generalInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
        System.out.println("Starting Robot...");
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }
}
