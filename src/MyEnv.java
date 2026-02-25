
import simbad.sim.*;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

/**
 * Class that implements the environment the robot operates on.
 * @author Spyridon Drakakis 3582
 * @author Antonis Georgosopoulos 3760
 */
public class MyEnv extends EnvironmentDescription {
    /**
     * Definition of the environment, adding objects and the robot.
     */
    public MyEnv(){
     //   add(new Arch(new Vector3d(3,0,-3),this));
        light1IsOn = true;
        light2IsOn = false;
        light1SetPosition(6,2,2);
        add(new MyRobot(new Vector3d(0, 0, 0),"my robot",false));




        Wall wall1 = new Wall(new Vector3d(5,0,1),5,1,this);
        add(wall1);


        Wall wall2 = new Wall(new Vector3d(5,0,-1),5,1,this);
        add(wall2);

        Wall wall3 = new Wall(new Vector3d(7,0,0),1,1,this);
        wall3.rotate90(1);
        add(wall3);

        Wall wall4 = new Wall(new Vector3d(3,0,-3),4,1,this);
        wall4.rotate90(1);
        add(wall4);

        Wall wall5 = new Wall(new Vector3d(0,0,-5),6,1,this);
        add(wall5);

        Wall wall6 = new Wall(new Vector3d(-3,0,0),10,1,this);
        wall6.rotate90(1);
        add(wall6);

        Wall wall7 = new Wall(new Vector3d(3,0,5),6,1,this);
        //wall7.rotate90(1);
        add(wall7);

        Box box = new Box(new Vector3d(-1.5,0,5),new Vector3f(1,1,1),this);
        add(box);

        Line line = new Line(new Vector3d(0,0,0),5,this);
       add(line);

        Line line2 = new Line(new Vector3d(0,0,5),5,this);
        line2.rotate90(1);
        add(line2);


        Line line3 = new Line(new Vector3d(-5,0,5),5,this);
        line3.rotate90(1);
        add(line3);

        Line line4 = new Line(new Vector3d(0,0,0),7,this);
        line4.rotate90(1);
        add(line4);

        Line line5 = new Line(new Vector3d(5,0,-6),5,this);
        add(line5);

        Line line6 = new Line(new Vector3d(0,0,-5),5,this);
        line6.rotate90(1);
        add(line6);

        Line line7 = new Line(new Vector3d(0,0,-5),5,this);
        add(line7);


/*
        Wall wall = new Wall(new Vector3d(1,0,1),5,1,this);
        wall.rotate90(1);
        add(wall);

        Wall wall2 = new Wall(new Vector3d(5,0,1),4,1,this);
        add(wall2);
        Wall wall3 = new Wall(new Vector3d(7,0,4),6,1,this);
        wall3.rotate90(1);
        add(wall3);

        Wall wall4 = new Wall(new Vector3d(5,0,3),4,1,this);
        add(wall4);


 */









    }
}
