package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import pabeles.concurrency.ConcurrencyOps.Reset;
import java.lang.Math;

public class NavXGyro extends SubsystemBase {
  private final AHRS navX = new AHRS();

  public NavXGyro() {
    navX.calibrate();
    reset();
  }

  public boolean isConnected() {
    return navX.isConnected();
  }

  public Rotation2d getAngle() {
    return navX.getRotation2d();
  }

  public void reset() {
    navX.reset();
  }

  public Rotation2d getRate() {
    return Rotation2d.fromDegrees(navX.getRate());
  }

  public float getPitchAngleDegrees() {
    return navX.getPitch();
  }

  public float getRollAngleDegrees() {
    return navX.getRoll();
  }

  public float getYawAngleDegrees() {
    return navX.getYaw();
  }


    
}
