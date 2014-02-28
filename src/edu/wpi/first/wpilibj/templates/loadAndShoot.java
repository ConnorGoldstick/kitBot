/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class loadAndShoot extends Thread {

    boolean running = false;
    Solenoid sol4, sol5, sol7, sol8;
    Victor victor;
    AnalogChannel encoder;
    DigitalInput digi14, digi13, digi3;
    Joystick xBox;
    Shoot shooter;
    boolean shooting = false; //starts the catapult
    int ShootCount = 0;
    Load Load;
    boolean loadingWithBall = false; //loads with more force, to compensate for the ball
    boolean loadingWithoutBall = false; //loads with less force
    int endLoadCount = 0;
    Unload Unload;
    boolean unloading = false; //extends the suction cup
    SuckUpBall SuckUpBall;
    boolean okToSuck = false; //toggles on/off the auto suction
    boolean sucking = false; //begins the autosuction
    int suckingCount = 0;
    boolean doNotSuck = false; //stops the autosuction from screwing up
    SmartDashboard smart;

    public loadAndShoot(AnalogChannel e, Victor v, Solenoid s4, Solenoid s5, Solenoid s7, Solenoid s8, Joystick x, DigitalInput d14, DigitalInput d13, DigitalInput d3, SmartDashboard sd) {
        victor = v;
        encoder = e;

        sol4 = s4;
        sol5 = s5;

        sol7 = s7;
        sol8 = s8;

        xBox = x;

        digi14 = d14;
        digi13 = d13;
        digi3 = d3;

        smart = sd;

        shooter = new Shoot(sol4, sol5, sol7, sol8);
        Load = new Load(victor, encoder);
        Unload = new Unload(victor, sol4, sol5);
        SuckUpBall = new SuckUpBall(victor, sol4, sol5);
    }

    public void setRun(boolean run) {
        running = run;
    }

    public void setBooleansToZero() {
        shooting = false;
        loadingWithBall = false;
        loadingWithoutBall = false;
        unloading = false;
        okToSuck = true;
        sucking = false;
        doNotSuck = false;
        sol4.set(false);
        sol5.set(true);
        sol7.set(true);
        sol8.set(false);
    }

    public void run() {
        while (true) {
            sol7.set(true);
            sol8.set(false);
            sol4.set(false);
            sol5.set(true);
            victor.set(0);
            shooting = false;
            loadingWithBall = false;
            loadingWithoutBall = false;
            endLoadCount = 0;
            unloading = false;
            ShootCount = 0;
            okToSuck = true;
            sucking = false;
            suckingCount = 0;
            doNotSuck = false;
            while (running) {
                smart.putBoolean("auto suction enabled", !doNotSuck && okToSuck);
                if (xBox.getRawAxis(6) == -1) {
                    System.out.println("reset everything");
                    loadingWithBall = false;
                    loadingWithoutBall = false;
                    unloading = false;
                    shooting = false;
                    sucking = false;
                    okToSuck = false;
                    doNotSuck = false;
                }

                //start end load
                if (digi3.get() && !unloading) {//stops the loading
                    System.out.println("load stop");
                    victor.set(0);
                    sol4.set(false); //disables the suction
                    sol5.set(true);
                    loadingWithBall = false;
                    loadingWithoutBall = false;
                }
                // end end load

                //begin autosuction and manual suction
                if (xBox.getRawButton(3)) { //toggle off
                    System.out.println("toggle suction off");
                    sol4.set(false); //turns the suction off
                    sol5.set(true);
                    okToSuck = false;
                }
                if (xBox.getRawButton(4)) { //toggle on
                    System.out.println("toggle suction on");
                    okToSuck = true;
                    doNotSuck = false; //resets the autosuction stop
                }
                if ((digi14.get()||digi13.get()) && !loadingWithBall && !loadingWithoutBall && !unloading && !shooting && !sucking && okToSuck && !doNotSuck) {
                    System.out.println("start suction");
                    sucking = true; //checks to see if ball is touching limit switch
                }
                if (sucking) { //autosuction occurs as long as this is true
                    System.out.println("suctioning");
                    suckingCount++;
                    SuckUpBall.suckItUp(); //suction command
                }
                if (suckingCount > 100) { //ends autosuction
                    System.out.println("end suction");
                    suckingCount = 0;
                    victor.set(0); //stops the victor movement
                    sucking = false;
                    doNotSuck = true; //prevents the autosuction from activating again
                }
                //end autosuction and manual suction

                //begin loading
                if (xBox.getRawButton(6) && !loadingWithBall && !loadingWithoutBall && !unloading && !shooting) {
                    System.out.println("start loading");
                    //checks to see if suction is on or not
                    if (sol4.get() == true && sol5.get() == false) { //suctioning
                        System.out.println("start load w/ ball");
                        loadingWithBall = true;
                    }
                    if (sol4.get() == false && sol5.get() == true) { //not suctioning
                        System.out.println("start load w/out ball");
                        loadingWithoutBall = true;
                    }
                }
                if (loadingWithBall) { //runs load with ball
                    System.out.println("load w/ ball");
                    Load.loadWithBall(); //loadWithBall command
                }
                if (loadingWithoutBall) { //runs load without ball
                    System.out.println("load w/out ball");
                    Load.loadWithoutBall(); //loadWithoutBall command
                }
                //end loading

                //begin shooter
                if (xBox.getRawAxis(3) < -.95 && !loadingWithBall && !loadingWithoutBall && !unloading && !shooting) {
                    System.out.println("start shooting");
                    shooting = true;
                    shooter.setCountToZero(); //resets the count in the class to zero
                }
                if (shooting) { //currently shooting
                    System.out.println("shooting");
                    ShootCount++;
                    shooter.Shoot(); //shoot command
                }
                if (ShootCount > 205) { //turns off the shoot
                    System.out.println("end shoot");
                    shooting = false;
                    ShootCount = 0;
                }
                //end shooter

                //begin unloading
                if (xBox.getRawButton(5) && !loadingWithBall && !loadingWithoutBall && !unloading && !shooting) {
                    unloading = true;
                    System.out.println("start unload");
                }
                if (unloading) { //suction arm extends
                    System.out.println("unloading");
                    Unload.unload(); //unload command
                }
                if (encoder.getVoltage() >= 2.5 && encoder.getVoltage() <= 3.8 && (unloading)) { //ends unloading
                    System.out.println("end unload");
                    unloading = false;
                    victor.set(0); //stops the victor
                    doNotSuck = false; //resets the autosuction stop
                }
                //end Unload
            }
        }
    }
}
