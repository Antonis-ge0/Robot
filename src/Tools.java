

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import simbad.sim.Agent;
import simbad.sim.RangeSensorBelt;

/**
 *Class implementing useful code to be used for robot behavior.
 * @author dvrakas
 */


public class Tools {
    /**
     * Method that returns a Point detected by the sonars.
     * @param rob The robot.
     * @param sonars Sonars on the robot.
     * @param sonar Sonar that made the detection
     * @return A 3d Point representing the point detected by the sonar.
     */
    public static Point3d getSensedPoint(Agent rob, RangeSensorBelt sonars, int sonar) {
        double v;
        if (sonars.hasHit(sonar))
            v = rob.getRadius() + sonars.getMeasurement(sonar);
        else
            v = rob.getRadius() + sonars.getMaxRange();
        double x = v * Math.cos(sonars.getSensorAngle(sonar));
        double z = v * Math.sin(sonars.getSensorAngle(sonar));
        return new Point3d(x, 0, z);
    }


    /**
     * Method that returns the angle of the robot.
     * @param rob The robot.
     * @return The angle of the robot.
     */
    public static double getAngle(Agent rob) {
        double angle = 0;
        double msin;
        double mcos;
        Transform3D m_Transform3D = new Transform3D();
        rob.getRotationTransform(m_Transform3D);
        Matrix3d m1 = new Matrix3d();
        m_Transform3D.get(m1);
        msin = m1.getElement(2, 0);
        mcos = m1.getElement(0, 0);
        if (msin < 0) {
            angle = Math.acos(mcos);
        } else {
            if (mcos < 0) {
                angle = 2 * Math.PI - Math.acos(mcos);
            } else {
                angle = -Math.asin(msin);
            }
        }
        while (angle < 0)
            angle += Math.PI * 2;
        return angle;
    }


    public static double wrapToPi(double a) {
        if (a > Math.PI)
            return a - Math.PI * 2;
        if (a <= -Math.PI)
            return a + Math.PI * 2;
        return a;
    }
}

