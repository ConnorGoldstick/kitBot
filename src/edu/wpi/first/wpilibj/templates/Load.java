/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Victor;

public class Load extends Thread {

    Victor victor;
    AnalogChannel encoder;

    public Load(Victor v, AnalogChannel e) {
        victor = v;
        encoder = e;
    }

    public void loadWithBall() {
        if (encoder.getVoltage() > 3 || encoder.getVoltage() < 0.5) {
            victor.set(0.6);
        } else {
            victor.set(0.1);
        }
    }
    public void loadWithoutBall(){
        System.out.println("LOAD2");
        if (encoder.getVoltage() > 3 || encoder.getVoltage() < 0.5) {
            victor.set(0.5);
            System.out.println("load3");
        } else {
            victor.set(0.2);
            System.out.println("load4");
        }
    }
}
