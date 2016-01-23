package hr.fer.spus.kalmanfilterdemo;

/**
 * Simplified implementation with two state components:
 *   [ x dx/dt ]
 */
public class KalmanFilter {
    float[] x_tp = new float[2];
    float[] x_t = new float[2];

    float[] z_t = new float[2];
    float dT = 0;

    float[][] P_tp = new float[2][2];
    float[][] P_t = new float[2][2];

    float[] y_t = new float[2];
    float S_t = 0;
    float[] K_t = new float[2];

    float[][] Q = new float[2][2];
    float R = 0;


    /**
     * Initializes a Kalman filter
     * @param x_t Initial state
     * @param Q Process noise covariance
     * @param R Measurement noise covariance
     */
    public KalmanFilter(float[] x_t, float[][] Q, float R){
        this.x_t = x_t.clone();
        this.z_t = this.x_t.clone();

        predict();
    }

    public void newMeasurement(float[] z_t, float dT){
        this.z_t = z_t;
        this.dT = dT;

        measure();
        predict();
    }

    /*
      Prediction equations

        x_tp = F_t * x_t + B_t * [ z_t[1] 0 ]'
            F_t = [ 1 -dT ; 0 1 ]
            B_t = [ dT 0 ]
            z_t[1] == measured dx/dt

        P_tp = F_t * P_t * transp(F_t) + Q
            Q in form: [ Qx 0 ; 0 Qdx/dt ]
     */
    private synchronized void predict(){
        x_tp[0] = x_t[0] + dT * (z_t[1] - x_t[1]);
        x_tp[1] = x_t[1];
        toFirstPeriod(x_tp);

        P_tp[0][0] = P_t[0][0] + dT * (dT * P_t[1][1] - P_t[0][1] - P_t[1][0] + Q[0][0]);
        P_tp[0][1] = P_t[0][1] - dT * P_t[1][1];
        P_tp[1][0] = P_t[1][0] - dT * P_t[1][1];
        P_tp[1][1] = P_t[1][1] - Q[1][1] * dT;
    }

    /*
      Measurement equations

        y_t = z_t - H_t * x_tp
            H_t = [ 1 0 ]
          |
          -->  y_t = z_t - x_tp[0]

        S_t = H_t * P_tp * transp(H_t) + R
            H_t = [ 1 0 ]
          |
          --> S_t = P_tp[0][0] + R

        K_t = P_tp * transp(H_t) * inv(S_t)
          |
          --> K_t = [ P_tp[0][0] P_tp[1][0] ] / S_t

        x_t = x_tp + K_t * y_t

        P_t = (I - K_t * H_t) * P_tp
          |
          --> P_t = P_tp - [ K_t[i] * P_tp[i][j] ]
     */
    private synchronized void measure(){
        y_t[0] = z_t[0] - x_tp[0];
        y_t[1] = z_t[1];

        S_t = P_tp[0][0] + R;

        if (S_t == 0)
            S_t = 1;

        K_t[0] = P_tp[0][0] / S_t;
        K_t[1] = P_tp[1][0] / S_t;

        x_t[0] = x_tp[0] + K_t[0] * y_t[0];
        x_t[1] = x_tp[1] + K_t[1] * y_t[1];
        toFirstPeriod(x_t);

        P_t[0][0] = P_tp[0][0] * (1 - K_t[0]);
        P_t[0][1] = P_tp[0][1] * (1 - K_t[0]);
        P_t[1][0] = P_tp[1][0] * (1 - K_t[1]);
        P_t[1][1] = P_tp[1][1] * (1 - K_t[1]);
    }

    private void toFirstPeriod(float[] a){
        a[0] %= (float) 2*Math.PI;
        a[1] %= (float) 2*Math.PI;
    }

    public synchronized float[] getState(){
        return x_t.clone();
    }
}
