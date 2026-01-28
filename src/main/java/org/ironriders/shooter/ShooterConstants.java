package org.ironriders.shooter;

public class ShooterConstants {

  public static final double G = 9.8; // m/s
  public static final double HEIGHT_DIFFERENCE_HUB_TO_SHOOTER = 0; // meters //TODO

  public static final double FLYWHEEL_MAX_VEL = 1000000; // ONE MILLION DOLLARS //TODO
  public static final double FLYWHEEL_MAX_ACC = 1000000; // ONE MILLION DOLLARS //TODO

  public static final double SHOOTER_HOOD_MAX_VEL = 1000000; // ONE MILLION DOLLARS //TODO
  public static final double SHOOTER_HOOD_MAX_ACC = 1000000; // ONE MILLION DOLLARS //TODO

  public static final double FLYWHEEL_P = 0.5;
  public static final double FLYWHEEL_I = 0.0;
  public static final double FLYWHEEL_D = 0.0;

  public static final double SHOOTER_P = 0.5;
  public static final double SHOOTER_I = 0.0;
  public static final double SHOOTER_D = 0.0;

  public static final double SHOOTER_STOW_POSITION = 0.0;

  public static final double FLYWHEEL_RADIUS = 2; // inches

  public enum State {
    AIMED(),
    READY(),
    STOW();
  }
}
