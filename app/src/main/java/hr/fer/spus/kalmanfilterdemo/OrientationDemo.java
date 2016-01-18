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
    private float[] gyroEarth = new float[3];

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
        } else if (event.sensor.equals(gyro)) {
            gyroValues = event.values;

            float[] gyroExchangedSystem = new float[3];
            /*gyroExchangedSystem[0] = - gyroValues[2];
            gyroExchangedSystem[1] = - gyroValues[0];
            gyroExchangedSystem[2] = - gyroValues[1];
            */
            gyroExchangedSystem[0] = -gyroValues[2];
            gyroExchangedSystem[1] = -gyroValues[0];
            gyroExchangedSystem[2] = -gyroValues[1];

            gyroEarth = (new CoordinateSystems(accMagValues)).rotateSystem(gyroExchangedSystem);
        } else
            throw new RuntimeException(new UnknownServiceException("Unknown sensor"));
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }



    //////////// UI PART

    Runnable showMeasurements = new Runnable() {
        @Override
        public void run() {
            float[] UIStateMeasurementData = accMagValues.clone();
            float[] UIStateDerData = gyroEarth.clone();

            TextView[] UIStateMeasurementTV = new TextView[3];
            UIStateMeasurementTV[0] = (TextView) findViewById(R.id.azimuth);
            UIStateMeasurementTV[1] = (TextView) findViewById(R.id.pitch);
            UIStateMeasurementTV[2] = (TextView) findViewById(R.id.roll);

            for (int i = 0; i < 3; i++)
                UIStateMeasurementTV[i].setText(String.format("%d", (int) Math.toDegrees(UIStateMeasurementData[i])));

            TextView[] UIStateDerTV = new TextView[3];
            UIStateDerTV[0] = (TextView) findViewById(R.id.gyroAzimuth);
            UIStateDerTV[1] = (TextView) findViewById(R.id.gyroPitch);
            UIStateDerTV[2] = (TextView) findViewById(R.id.gyroRoll);

            for (int i = 0; i < 3; i++)
                UIStateDerTV[i].setText(String.format("%d", (int) Math.toDegrees(UIStateDerData[i])));
        }
    };
}
