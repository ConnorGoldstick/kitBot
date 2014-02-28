/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;

public class Shoot extends Thread {

    Victor victor;
    Solenoid sol4, sol5, sol7, sol8;
    int count;
    Unload Unload;

    public Shoot(Solenoid s4, Solenoid s5, Solenoid s7, Solenoid s8) {
        sol4 = s4;
        sol5 = s5;
        sol7 = s7;
        sol8 = s8;

        count = 0;
    }

    public void setCountToZero() {
        count = 0;
    }

    public void Shoot() {
        if (count <= 5) {
            count++;
            sol4.set(false);
            sol5.set(true);
        }
        if (count > 5 && count < 200) {
            count++;
            sol7.set(false);
            sol8.set(true);
        }
        if (count >= 200) {
            sol7.set(true);
            sol8.set(false);
        }
    }
}
