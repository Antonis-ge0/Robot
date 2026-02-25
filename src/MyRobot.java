import simbad.sim.*; //simbad-1.7
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.Random;

/**
 * @author Antonis Georgosopoulos
 *  Class MyRobot that implements the Robot and its behavior. It initialized all the variables needed and defines
 *  robot behavior.
 */
public class MyRobot extends Agent {
    public enum alState{
        one,two;
    }
    alState alstate;
    int counter2;
    RangeSensorBelt bumpers;
    double iL,iH;
    RangeSensorBelt sonars;
    LightSensor LR, LL;
    LineSensor line;
    boolean done,lineFollow,beginning,turning,fwd,collision,backing;
    int counter;
    static double K1 = 5;
    static double K2 = 0.8;
    static double K3 = 1;
    static double SAFETY =0.5;
    boolean CLOCKWISE;
    double light;
    Point3d robCod = new Point3d();
    
    /*
     * Constructor for class MyRobot. Initiates position, name as well as its sensors.
     * @param position Starting position of the robot.
     * @param name Name of the robot.
     * @param CLOCKWISE Boolean that indicates if the robot turns clockwise or counterclockwise.
     */
    public MyRobot(Vector3d position, String name,boolean CLOCKWISE) {
        super(position, name);
        bumpers = RobotFactory.addBumperBeltSensor(this, 8);
        sonars = RobotFactory.addSonarBeltSensor(this, 12);
        LR = RobotFactory.addLightSensorRight(this);
        LL = RobotFactory.addLightSensorLeft(this);
        line = RobotFactory.addLineSensor(this, 11);
        this.CLOCKWISE = CLOCKWISE;
    }

    /*
     * Initializing behavior, namely all the variables that are to be used in the main loop. This method is called once upon
     * beginning of the simulation.
     */
    public void initBehavior() {
        beginning = true;
        done = false;
        lineFollow = false;
        counter =0;
        fwd = false;
        collision = false;
        turning = true;
        alstate = alState.one;
        backing = false;
        counter2 = 0;
    }

    /*
     * Main loop. This method is called once every interval of the simulation. First, the robot turns to check if there
     * are lines under its position in any direction. Once it checks, the main loop begins.
     * First, it decides if the robot is to move clockwise or counterclockwise
     * based on its angle. Then it proceeds to state one, which is turning to the goal and moving towards it,
     * if it encounters a line, it follows it, otherwise if it encounters an obstacle, it circumnavigates it.
     * Then, if it detects a local maximum of the average of its two light sensors, it moves towards the goal.
     * If it encounters an obstacle, it moves backwards for 3 ticks.
     * If weighted average of its two light sensor inputs is above a certain threshold, it stops.
     */
    public void performBehavior(){
        
        double neutral = Math.PI/6;
        
        if(Tools.getAngle(this) >=(Math.PI/2 + neutral)/2*Math.PI && Tools.getAngle(this) <=(3*Math.PI/2 - neutral)/2*Math.PI){
            CLOCKWISE = true;
        }else if (Tools.getAngle(this) <= Math.PI/2- neutral && Tools.getAngle(this)>= 3*Math.PI/2 + neutral) {
            CLOCKWISE = false;
        }
        
        if(!beginning) {
            if (!done) {
                if (backing) {
                    counter++;
                    if (counter > 3) {
                        backing = false;
                        counter = 0;
                    }
                    return;
                }

                if(counter2>10000){
                    alstate = alState.one;
                    counter2 = 0;
                }else{
                    counter++;
                }
                
                if (alstate == alState.one) {
                    light = (LL.getLux() + LR.getLux());
                    if (!checkLineFollow()) {

                        if (!UOri()) {
                            return;
                        } else {
                            UFwd();
                        }
                    }else{
                        initLineFollow();
                    }
                    if (LL.getLux() > LR.getLux()) {
                        if ((LL.getLux() + LR.getLux() + 0.005) / 2 > 0.068) {
                            setRotationalVelocity(0);
                            setTranslationalVelocity(0);
                            done = true;
                        }

                    } else if (LL.getLux() < LR.getLux()) {
                        if ((LL.getLux() + 0.005 + LR.getLux()) / 2 > 0.068) {
                            setRotationalVelocity(0);
                            setTranslationalVelocity(0);
                            done = true;
                        }

                    } else {
                        if ((LL.getLux() + 0.003 + LR.getLux() + 0.003) / 2 > 0.068) {
                            setRotationalVelocity(0);
                            setTranslationalVelocity(0);
                            done = true;
                        }
                    }

                    if ((Math.abs(light - iL) >= 0.001)) {
                        iH = light;
                    }

                    if (!obstacleBumper()) {
                        alstate = alState.one;
                    } else {
                        backing = true;
                        alstate = alState.two;
                    }

                } else {
                        if ((LL.getLux() + LR.getLux()) > iH + 0.0009) {
                            alstate = alState.one;
                        } else {
                            avoidObstacle();
                        }
                }
            }
        }else{
            if(Tools.getAngle(this) <= 3*Math.PI/2) {
                setRotationalVelocity(1);
            }else{
                beginning= false;
            }
            if(checkLineFollow()){
                beginning = false;
            }
        }
    }

    /*
     * Method that checks if the robot is on a line and is to follow it.
     * @return A boolean that is true if the line sensors detected a line and false otherwise.
     */
    private boolean checkLineFollow(){
        int left =0, right =0;
        int k=0;
        for (int i = 0; i < line.getNumSensors() / 2; i++) {
            left += line.hasHit(i) ? 1 : 0;
            right += line.hasHit(line.getNumSensors() - i - 1) ? 1 : 0;
            k++;
        }
        return left != 0 || right != 0;
    }

    /*
     * Method that initiates line-following.
     */
    private void initLineFollow(){
        int left =0, right =0;
        float k=0;
        for (int i = 0; i < line.getNumSensors() / 2; i++) {
            left += line.hasHit(i) ? 1 : 0;
            right += line.hasHit(line.getNumSensors() - i - 1) ? 1 : 0;
            k++;
        }
        setTranslationalVelocity(0.5);
        this.setRotationalVelocity((left - right) / k * 5);
    }

    /*
     * Method that circumnavigates an obstacle. Based on the minimum reading of the sonars it calculates the closest
     * point to the robot and follows the perimeter of the obstacle at a safe distance defined above.
     */
    public void avoidObstacle() {
        int min;
        min=0;
        for (int i=1;i<sonars.getNumSensors();i++)
            if (sonars.getMeasurement(i)<sonars.getMeasurement(min))
                min=i;
        Point3d p = Tools.getSensedPoint(this,this.sonars,min);
        double d = p.distance(new Point3d(0,0,0));
        Vector3d v;
        if (CLOCKWISE)
            v = new Vector3d(-p.z,0,p.x);
        else
            v = new Vector3d(p.z,0,-p.x);
        double phLin = Math.atan2(v.z,v.x);
        double phRot = Math.atan(K3*(d-SAFETY));

        if (CLOCKWISE)
            phRot=-phRot;
        double phRef = Tools.wrapToPi(phLin+phRot);
        setRotationalVelocity(K1*phRef);
        setTranslationalVelocity(K2*Math.cos(phRef));
       // setTranslationalVelocity(0.5);
    }

    /*
     * Method that moves the robot backwards if the front bumper sensors are activated and forwards if the backwards if
     * the front sensors are activated. Returns if the sensors hit at all.
     * @return A boolean that is true if the sensors detected a collision and false otherwise.
     */

    private boolean obstacleBumper(){
        if(bumpers.getFrontQuadrantHits() > 0){
            setTranslationalVelocity(-0.5);
        }else if(bumpers.getBackQuadrantHits()>0){
            setTranslationalVelocity(0.5);
        }

        return bumpers.getFrontQuadrantHits() > 0 || bumpers.getBackQuadrantHits() >0;
    }

    /*
     * Method that turns the robot toward the goal (light source). Returns a boolean that is true if the robot is
     * turned towards the goal and false otherwise.
     * @return A boolean that is true if the robot is turned towards the goal and false otherwise.
     */
    private boolean UOri(){
        setTranslationalVelocity(0);
        if(LL.getLux() - LR.getLux() >= 0.001){
            setRotationalVelocity(Math.PI/4);
            return false;
        }else if (LL.getLux() - LR.getLux() <= -0.001){
            setRotationalVelocity(-Math.PI/4);
            return false;
        }else{
            return true;
        }
    }

    /*
     * Method that moves the robot in a straight line.
     */
    private void UFwd(){
        setRotationalVelocity(0);
        setTranslationalVelocity(0.5);
    }
}
