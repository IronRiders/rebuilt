package org.ironriders.manipulation.shooter;

public class ShooterConstants {

  public static final double G = 9.8; // m/s
  public static final double HEIGHT_DIFFERENCE_HUB_TO_SHOOTER = 1.3208; // m
  public static final double TARGET_BALL_VELOCITY = 12.6; //m/s (see https://www.reca.lc/flywheel)

  public static final double FLYWHEEL_MAX_VEL = 6065; // (see https://www.reca.lc/flywheel)
  public static final double FLYWHEEL_MAX_ACC = FLYWHEEL_MAX_VEL / 2;

  public static final double SHOOTER_HOOD_MAX_VEL = 180; // TODO
  public static final double SHOOTER_HOOD_MAX_ACC = SHOOTER_HOOD_MAX_VEL / 2; // TODO

  public static final double FLYWHEEL_P = 0.5;
  public static final double FLYWHEEL_I = 0.0;
  public static final double FLYWHEEL_D = 0.0;

  public static final double FLYWHEEL_TOLERANCE = 40; // rpm

  public static final double SHOOTER_P = 0.5;
  public static final double SHOOTER_I = 0.0;
  public static final double SHOOTER_D = 0.0;

  public static final double SHOOTER_TOLERANCE = 0.1;

  public static final double SHOOTER_STOW_POSITION = 0.0;

  public static final double SHOOTER_MAX_RANGE = 14.5; // meters

  public static final double MIN_ROTATION = 15;
  public static final double MAX_ROTATION = 45;

  public static final double SPINDOWN_TIME = 2;  // time to automatically go into idle mode after, currently TODO

  public enum State {
    READY(),
    IDLE(),
    STOW();
  }
}
