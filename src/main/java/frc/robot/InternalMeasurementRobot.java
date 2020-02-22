/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.squareup.moshi.Moshi;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import org.strykeforce.deadeye.Camera;
import org.strykeforce.deadeye.Deadeye;
import org.strykeforce.deadeye.MinAreaRectTargetData;
import org.strykeforce.deadeye.MinAreaRectTargetDataJsonAdapter;

import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;

import static frc.robot.InternalMeasurementRobot.State.*;


public class InternalMeasurementRobot extends TimedRobot {

    enum State {
        CAMERA_INIT, CAMERA_WAIT, LIGHT_INIT, LIGHT_WAIT, TEST_INIT, TEST_WAIT, TEST_DONE, ERROR, STOPPED
    }

    private final static boolean TRACE = false;

    private final static Deadeye DEADEYE = Deadeye.INSTANCE;

    private State state;
    private long startTime;
    private Camera<MinAreaRectTargetData> camera;
    private volatile boolean targetValid;
    private boolean lastValid;
    private boolean running = true;
    private double sum = 0.0;
    private int count = 0;

    private ExecutorService executorService = Executors.newSingleThreadExecutor();
    private ScheduledExecutorService scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();

    private DigitalOutput output = new DigitalOutput(1);

    Future<Long> lightFuture;
    AtomicLong targetValidTime = new AtomicLong(0);

    @Override
    public void robotInit() {
        camera = DEADEYE.getCamera("C0");
        camera.setJsonAdapter(new MinAreaRectTargetDataJsonAdapter(new Moshi.Builder().build()));
        camera.setTargetDataListener(targetData -> {
            targetValid = targetData.getValid();
            if (targetValid) targetValidTime.compareAndSet(0, System.nanoTime());
            if (lastValid != targetValid) {
                output.set(targetValid);
                lastValid = targetValid;
                System.out.println(lastValid);
            }

        });
        camera.setEnabled(true);
        output.set(true);
        setState(STOPPED);
    }

    @Override
    public void robotPeriodic() {
        switch (state) {
            case ERROR:
                setState(STOPPED);
                break;

            case STOPPED:
                if (RobotController.getUserButton()) {
                    running = false;
                    camera.setEnabled(false);
                    System.out.printf("Running = %b\n", running);
                }
                if (running) {
                    setState(CAMERA_INIT);
                }
                break;

            case CAMERA_INIT:
                startTime = System.currentTimeMillis();
                setState(CAMERA_WAIT);
                break;

            case CAMERA_WAIT:
                if (targetValid) {
                    setState(LIGHT_INIT);
                    break;
                }

                if (System.currentTimeMillis() - startTime >= 5000) {
                    System.out.println("Timed out waiting for camera init target valid");
                    setState(ERROR);
                }
                break;

            case LIGHT_INIT:
                camera.setLightEnabled(false);
                NetworkTableInstance.getDefault().flush();
                startTime = System.currentTimeMillis();
                setState(LIGHT_WAIT);
                break;

            case LIGHT_WAIT:
                if (!targetValid) {
                    setState(TEST_INIT);
                    break;
                }

                if (System.currentTimeMillis() - startTime >= 1000) {
                    if (TRACE)
                        System.out.println("Timed out waiting for lights out");
                    setState(LIGHT_INIT);
                }
                break;

            case TEST_INIT:
                targetValidTime.set(0);
                lightFuture = executorService.submit(() -> {
                    try (DigitalInput input = new DigitalInput(0)) {
                        if (!input.get()) {
                            System.out.println("ERROR: digital input should be high at start of test");
                        }
                        while (input.get()) {
                            Thread.sleep(1);
                        }
                    }
                    return System.nanoTime();
                });

                scheduledExecutorService.schedule(() -> {
                    camera.setLightEnabled(true);
                }, 1, TimeUnit.SECONDS);

                if (targetValid) {
                    System.out.println("ERROR: target should not be valid at test start");
                }

                startTime = System.currentTimeMillis();
                setState(TEST_WAIT);
                break;

            case TEST_WAIT:
                if (targetValidTime.get() != 0) {
                    setState(TEST_DONE);
                    break;
                }

                if (System.currentTimeMillis() - startTime >= 10000) {
                    System.out.println("Timed out waiting for test to finish");
                    setState(ERROR);
                }
                break;

            case TEST_DONE:
                if (!lightFuture.isDone()) {
                    System.out.println("Light shut-off was not detected");
                    lightFuture.cancel(true);
                    setState(ERROR);
                    break;
                }

                try {
                    long start = lightFuture.get();
                    double latency = (targetValidTime.get() - start) / 1e6;
                    count++;
                    sum += latency;
                    System.out.printf("Latency: %f msec, %f msec average\n", latency, sum / count);
                } catch (InterruptedException | ExecutionException e) {
                    e.printStackTrace();
                }
                setState(STOPPED);

                break;


        }
    }

    private void setState(State state) {
        if (TRACE)
            System.out.printf("State: %s\n", state);
        this.state = state;
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
