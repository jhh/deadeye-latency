/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.squareup.moshi.Moshi;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import org.strykeforce.deadeye.Camera;
import org.strykeforce.deadeye.Deadeye;
import org.strykeforce.deadeye.MinAreaRectTargetData;
import org.strykeforce.deadeye.MinAreaRectTargetDataJsonAdapter;


public class ExternalMeasurementRobot extends TimedRobot {

    private final static boolean TRACE = true;

    private final static Deadeye DEADEYE = Deadeye.INSTANCE;

    private Camera<MinAreaRectTargetData> camera;
    private DigitalOutput output = new DigitalOutput(0);
    private boolean lastValid;

    @Override
    public void robotInit() {
        camera = DEADEYE.getCamera("C0");
        camera.setJsonAdapter(new MinAreaRectTargetDataJsonAdapter(new Moshi.Builder().build()));
        camera.setTargetDataListener(targetData -> {
            boolean valid = targetData.getValid();
            if (lastValid != valid) {
                output.set(valid);
                lastValid = valid;
                System.out.println(lastValid);
            }
        });
        camera.setEnabled(true);
    }

    @Override
    public void robotPeriodic() {
//        output.set(valid);
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

}
