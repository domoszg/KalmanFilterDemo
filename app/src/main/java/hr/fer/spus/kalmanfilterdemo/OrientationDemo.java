package hr.fer.spus.kalmanfilterdemo;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

import java.net.UnknownServiceException;

public class OrientationDemo extends AppCompatActivity implements SensorEventListener {

    private SensorManager sm;
    private Sensor accMag, gyro;
    private float[] accMagValues = new float[3];
    private float[] gyroValues = new float[3];

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
        if (event.sensor.equals(accMag))
            accMagValues = event.values;
        else if (event.sensor.equals(gyro))
            gyroValues = event.values;
        else
            throw new RuntimeException(new UnknownServiceException("Unknown sensor"));
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}
