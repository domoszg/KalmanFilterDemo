package hr.fer.spus.kalmanfilterdemo;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.TextView;

import java.net.UnknownServiceException;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class OrientationDemo extends AppCompatActivity implements SensorEventListener {

    private SensorManager sm;
    private Sensor accMag, gyro;
    private float[] accMagValues = new float[3];
    private float[] gyroValues = new float[3];

    private ScheduledThreadPoolExecutor scheduler = new ScheduledThreadPoolExecutor(1);

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_orientation_demo);

        // Initialize sensors
        sm = (SensorManager) getSystemService(SENSOR_SERVICE);
        accMag = sm.getDefaultSensor(Sensor.TYPE_ORIENTATION);
        gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        sm.registerListener(this, accMag, SensorManager.SENSOR_DELAY_FASTEST);
        sm.registerListener(this, gyro, SensorManager.SENSOR_DELAY_FASTEST);


        // Schedule UI updates
        scheduler.scheduleAtFixedRate(new Runnable() {
            @Override
            public void run() {
                runOnUiThread(showMeasurements);
            }
        }, 2000, 100, TimeUnit.MILLISECONDS);
    }

    public void onPause() {
        super.onPause();
        this.finish();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        sm.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.equals(accMag)) {
            accMagValues = event.values;
            for (int i = 0; i < 3; i++)
                accMagValues[i] = (float) Math.toRadians(accMagValues[i]);
        } else if (event.sensor.equals(gyro))
            gyroValues = event.values;
        else
            throw new RuntimeException(new UnknownServiceException("Unknown sensor"));
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }



    //////////// UI PART

    Runnable showMeasurements = new Runnable() {
        @Override
        public void run() {
            float[] UImeasurementData = accMagValues.clone();

            TextView[] UImeasurementTV = new TextView[3];
            UImeasurementTV[0] = (TextView) findViewById(R.id.azimuth);
            UImeasurementTV[1] = (TextView) findViewById(R.id.pitch);
            UImeasurementTV[2] = (TextView) findViewById(R.id.roll);

            for (int i = 0; i < 3; i++)
                UImeasurementTV[i].setText(String.format("%1.3f", UImeasurementData[i]));
        }
    };
}
