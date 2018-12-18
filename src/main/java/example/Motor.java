package example;

import ev3dev.actuators.lego.motors.EV3LargeRegulatedMotor;
import ev3dev.sensors.Battery;

import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Motor 
{
	final static EV3LargeRegulatedMotor mA = new EV3LargeRegulatedMotor(MotorPort.A);
    final static EV3LargeRegulatedMotor mB = new EV3LargeRegulatedMotor(MotorPort.B);
    final static EV3LargeRegulatedMotor mC = new EV3LargeRegulatedMotor(MotorPort.C);
    public static int multiplicateur = 3;
    public static float forces[][] = {{0.58F, -0.33F, 0.33F},{-0.58F, -0.33F, 0.33F},{0, 0.67F, 0.33F}}; 
    public static float direction[] = {0, 0, 0};
    public static float valeursInterMoteurs[] = {0, 0, 0};
    public static int valeursMoteurs[] = {0, 0, 0} ;
    public static int iteration_threshold = 10;
    
    public static void main(String[] args) 
    {

    		while(true)
    		{
    			deplacement90fluide(forces, direction, valeursInterMoteurs, valeursMoteurs, mA, mB, mC);
                mA.stop();
                mC.stop();
                mB.stop();
                Delay.msDelay(2000);

    		}
    }
    
    public static void directionDroit(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
    {

        direction[0] = 1;
        direction[1] = 0;
        direction[2] = 0;
        multiplicationMatrice(forces, direction, valeursInterMoteurs, valeursMoteurs);
        lancement(valeursMoteurs, mA, mB, mC);
    }
    
    public static void deplacementDroite(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
    {

        direction[0] = 0;
        direction[1] = 1;
        direction[2] = 0;
        multiplicationMatrice(forces, direction, valeursInterMoteurs, valeursMoteurs);
        lancement(valeursMoteurs, mA, mB, mC);
    }
    
    public static void deplacementGauche(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
    {

        direction[0] = 0;
        direction[1] = 1;
        direction[2] = 0;
        multiplicationMatrice(forces, direction, valeursInterMoteurs, valeursMoteurs);
        valeursMoteurs[0] = (-1) * valeursMoteurs[0];
        valeursMoteurs[1] = (-1) * valeursMoteurs[1];
        valeursMoteurs[2] = (-1) * valeursMoteurs[2];
        lancement(valeursMoteurs, mA, mB, mC);
    }
    
    public static void deplacementArriere(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
    {

        direction[0] = 1;
        direction[1] = 0;
        direction[2] = 0;
        multiplicationMatrice(forces, direction, valeursInterMoteurs, valeursMoteurs);
        valeursMoteurs[0] = (-1) * valeursMoteurs[0];
        valeursMoteurs[1] = (-1) * valeursMoteurs[1];
        valeursMoteurs[2] = (-1) * valeursMoteurs[2];
        lancement(valeursMoteurs, mA, mB, mC);
    }
    
    public static void deplacement90fluide(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
    {

        /*direction[0] = 1;
        direction[1] = 2;
        direction[2] = 2;*/
        direction[0] = 0;
        direction[1] = 1;
        direction[2] = 1;
        multiplicationMatrice(forces, direction, valeursInterMoteurs, valeursMoteurs);
        lancement(valeursMoteurs, mA, mB, mC);
        Delay.msDelay(1500);
    }

    public static void deplacement90(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
    {

        direction[0] = 0;
        direction[1] = 0;
        direction[2] = 2;
        multiplicationMatrice(forces, direction, valeursInterMoteurs, valeursMoteurs);
        lancement(valeursMoteurs, mA, mB, mC);
        Delay.msDelay(3000);
    }
    public static void lancement(int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
    {
        System.out.println("Starting motors");

        if(valeursMoteurs[0] < 0)
        {
            mA.setSpeed(-multiplicateur * valeursMoteurs[0]);
            mA.forward();
        }
        else
        {
            mA.setSpeed(multiplicateur * valeursMoteurs[0]);
            mA.backward();
        }

        if(valeursMoteurs[1] < 0)
        {
            mB.setSpeed(-multiplicateur * valeursMoteurs[1]);
            mB.forward();
        }
        else
        {
            mB.setSpeed(multiplicateur * valeursMoteurs[1]);
            mB.backward();
        }
        if(valeursMoteurs[2] < 0)
        {
            mC.setSpeed(-multiplicateur * valeursMoteurs[2]);
            mC.forward();
        }
        else
        {
            mC.setSpeed(multiplicateur * valeursMoteurs[2]);
            mC.backward();
        }
        System.out.println(String.format("Large Motor is moving: %s at speed %d", mA.isMoving(), mA.getSpeed()));
        
    }

    public static void multiplicationMatrice(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[])
    {
        valeursMoteurs[0] = 0;
        valeursMoteurs[1] = 0;
        valeursMoteurs[2] = 0;
        valeursInterMoteurs[0] = 0;
        valeursInterMoteurs[1] = 0;
        valeursInterMoteurs[2] = 0;
        for(int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                valeursInterMoteurs[i] = valeursInterMoteurs[i] + forces[i][j] * direction[j];
            }
        }
        for(int i = 0; i < 3; i++)
        {
            valeursMoteurs[i] = (int)(valeursInterMoteurs[i] * 100);
        }
    }

}







