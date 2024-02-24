package frc.robot.Utilities;

public class Tracking {
  public static double hypotenuseFromLegs(double height, double distance) {
    // distance^2 + height^2 = hypotenuse^2
    return Math.sqrt(distance * distance + height * height);
  }
}
