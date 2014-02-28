/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class Drive extends Thread {

    Jaguar jagleft1, jagright3, jagleft2, jagright4;
    Solenoid sol1, sol2;
    Joystick xBox;
    boolean running = false;
    double speed, turn, leftspeed, rightspeed;

    public Drive(Jaguar j1, Jaguar j2, Jaguar j3, Jaguar j4, Solenoid s1, Solenoid s2, Joystick x) {
        jagleft1 = j1;
        jagleft2 = j2;
        jagright3 = j3;
        jagright4 = j4;
        sol1 = s1;
        sol2 = s2;
        xBox = x;
    }

    public void setRun(boolean run) {
        running = run;
    }

    public void run() {
        while (true) {
            sol1.set(false);
            sol2.set(true);
            while (running) {
                if (xBox.getRawAxis(2) >= 0) {
                    speed = Math.sqrt(xBox.getRawAxis(2) * xBox.getRawAxis(2) * xBox.getRawAxis(2));
                } else {
                    speed = -Math.sqrt(-xBox.getRawAxis(2) * (xBox.getRawAxis(2) * xBox.getRawAxis(2)));
                }
                turn = 0.5 * xBox.getRawAxis(4); //may need to adjust this
                if (turn < 0.1 && turn > -0.1) { //maybe need to adjust this
                    turn = 0;
                }
                /*
                if (xBox.getRawButton(1)) {
                    sol1.set(false);
                    sol2.set(true);
                }
                if (xBox.getRawButton(2)) {
                    sol1.set(true);
                    sol2.set(false);
                }
                */
                if (Math.abs((speed - turn) - leftspeed) < .075/*
                         * .075
                         */) { //don't skip target
                    leftspeed = speed - turn;
                }
                if (leftspeed < speed - turn) { //ramp up
                    leftspeed = leftspeed + .1;
                }
                if (leftspeed > speed - turn) { //ramp down
                    leftspeed = leftspeed - .1;
                }

                if (Math.abs((speed + turn) - leftspeed) < .075/*
                         * 0.075
                         */) { //don't skip target
                    rightspeed = speed + turn;
                }
                if (rightspeed < speed + turn) { //ramp up
                    rightspeed = rightspeed + .1;
                }
                if (rightspeed > speed + turn) {//ramp down
                    rightspeed = rightspeed - .1;
                }

                if (Math.abs(rightspeed) < .05) { //dead zone
                    rightspeed = 0;
                }
                if (Math.abs(leftspeed) < .05) { //dead zone
                    leftspeed = 0;
                }
                jagleft1.set(leftspeed);
                // jagleft2.set(leftspeed);
                jagright3.set(-(rightspeed));
                // jagright4.set(-(rightspeed));
            }
        }
    }
}