package hr.fer.spus.kalmanfilterdemo;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.SystemClock;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.UnknownServiceException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class OrientationDemo extends AppCompatActivity implements SensorEventListener {

    private SensorManager sm;
    private Sensor accMag, gyro;
    private float[] accMagValues = new float[3];
    private float[] gyroValues = new float[3];
    private float[] gyroEarth = new float[3];
    private long prevTime, dT;

    private static final int period = 50;
    private static final int UIperiod = 100;
    private static final float Q_angle = (float) 0.001;
    private static final float Q_dAdT = (float) 0.003;
    private static final float R_angle = (float) 0.03;

    private boolean filtersInitialized = false;
    private KalmanFilter azimuthFilter, pitchFilter, rollFilter;
    private float[] filteredStateValues = new float[3];
    private float[] filteredDerValues = new float[3];

    private boolean fileOpen = false;
    private FileOutputStream file;
    private static final int BUF_SIZE = 1024;
    private float[][][] fileBuffer = new float[BUF_SIZE][5][3];
    private int bufIndex = 0;

    private ScheduledThreadPoolExecutor scheduler = new ScheduledThreadPoolExecutor(2);

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
        }, 2000, UIperiod, TimeUnit.MILLISECONDS);


        scheduler.scheduleAtFixedRate(new Runnable() {
            @Override
            public synchronized void run() {
                float dT_tmp = (float) (new Long(dT)) / 1000;
                prevTime = SystemClock.elapsedRealtime();

                if (!filtersInitialized){
                    azimuthFilter = new KalmanFilter(new float[]{accMagValues[0], gyroEarth[0]}, new float[][]{{Q_angle, 0}, {0, Q_dAdT}}, R_angle);
                    pitchFilter = new KalmanFilter(new float[]{accMagValues[1], gyroEarth[1]}, new float[][]{{Q_angle, 0}, {0, Q_dAdT}}, R_angle);
                    rollFilter = new KalmanFilter(new float[]{accMagValues[2], gyroEarth[2]}, new float[][]{{Q_angle, 0}, {0, Q_dAdT}}, R_angle);

                    filtersInitialized = true;
                }

                azimuthFilter.newMeasurement(new float[]{accMagValues[0], gyroEarth[0]}, dT_tmp);
                pitchFilter.newMeasurement(new float[]{accMagValues[1], gyroEarth[1]}, dT_tmp);
                rollFilter.newMeasurement(new float[]{accMagValues[2], gyroEarth[2]}, dT_tmp);

                if (filtersInitialized){

                    float[] azFil = azimuthFilter.getState();
                    float[] piFil = pitchFilter.getState();
                    float[] roFil = rollFilter.getState();

                    filteredStateValues = new float[]{azFil[0], piFil[0], roFil[0]};
                    filteredDerValues = new float[]{azFil[1], piFil[1], roFil[1]};

                    try {
                        if (fileOpen)
                            if (bufIndex < BUF_SIZE) {
                                fileBuffer[++bufIndex] = new float[][]{new float[]{(float) prevTime / 1000}, accMagValues, gyroEarth, filteredStateValues, filteredDerValues};
                            } else {
                                writeFile();
                            }
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }, 5000, period, TimeUnit.MILLISECONDS);


        // Handle Record button
        final Button recButton = (Button) findViewById(R.id.fileRecord);
        final TextView fileNameTV = (TextView) findViewById(R.id.fileNameField);
        fileNameTV.setText("DataGrabber-"+new SimpleDateFormat("yyyy-MM-dd-HH.mm.ss").format(new Date())+".txt");

        recButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (!fileOpen) {
                    File fileObject = new File(android.os.Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + fileNameTV.getText().toString());

                    try {
                        file = new FileOutputStream(fileObject);
                        fileOpen = true;
                        recButton.setText("Stop");
                    } catch (Exception e){
                        throw new RuntimeException(e);
                    }
                } else {
                    try {
                        writeFile();
                        file.flush();
                        file.close();
                        fileOpen = false;
                        recButton.setText("Record");
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        });
    }

    private void writeFile() throws IOException{
        if (!fileOpen)
            throw new RuntimeException("writeFile while file not open");

        for (int i = 0; i < bufIndex; i++){
            String line = String.format("%.3f#%1.2f;%1.2f;%1.2f#%1.2f;%1.2f;%1.2f#%1.2f;%1.2f;%1.2f#%1.2f;%1.2f;%1.2f",
                    fileBuffer[i][0][0],
                    fileBuffer[i][1][0], fileBuffer[i][1][1], fileBuffer[i][1][2],
                    fileBuffer[i][2][0], fileBuffer[i][2][1], fileBuffer[i][2][2],
                    fileBuffer[i][3][0], fileBuffer[i][3][1], fileBuffer[i][3][2],
                    fileBuffer[i][4][0], fileBuffer[i][4][1], fileBuffer[i][4][2] );

            file.write(line.getBytes());
            file.write('\n');
        }

        fileBuffer = new float[BUF_SIZE][4][3];
        bufIndex = 0;

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
    public synchronized void onSensorChanged(SensorEvent event) {
        if (event.sensor.equals(accMag)) {

            accMagValues = event.values.clone();
            dT = SystemClock.elapsedRealtime() - prevTime;

            for (int i = 0; i < 3; i++)
                accMagValues[i] = (float) Math.toRadians(accMagValues[i]);

        } else if (event.sensor.equals(gyro)) {

            gyroValues = event.values.clone();
            dT = SystemClock.elapsedRealtime() - prevTime;

            float[] gyroExchangedSystem = new float[3];
            gyroExchangedSystem[2] = -gyroValues[2];
            gyroExchangedSystem[1] = -gyroValues[0];
            gyroExchangedSystem[0] = -gyroValues[1];

            float[] gyroEarthTmp = (new CoordinateSystems(accMagValues)).rotateSystem(gyroExchangedSystem);
            gyroEarth[2] = gyroEarthTmp[0];
            gyroEarth[1] = gyroEarthTmp[1];
            gyroEarth[0] = gyroEarthTmp[2];

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

            TextView[] UIKalmanTV = new TextView[3];
            UIKalmanTV[0] = (TextView) findViewById(R.id.kalman_yaw);
            UIKalmanTV[1] = (TextView) findViewById(R.id.kalman_pitch);
            UIKalmanTV[2] = (TextView) findViewById(R.id.kalman_roll);

            float[] UIKalmanState = filteredStateValues.clone();
            for (int i = 0; i < 3; i++)
                UIKalmanTV[i].setText(String.format("%d", (int) Math.toDegrees(UIKalmanState[i])));
        }
    };
}
