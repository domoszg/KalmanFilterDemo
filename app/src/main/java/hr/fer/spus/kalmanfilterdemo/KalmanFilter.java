package hr.fer.spus.kalmanfilterdemo;

/**
 * Implementation requires two component state vectors.
 */
public class KalmanFilter {
    private float[] x_t;      // Predicted state
    private float[] x_tp;     // Previous state
    private float[] u_t;      // Control input
    private float[] w_t;      // Process noise

    private float[][] F_t;      // State transition matrix
    private float[][] B_t;      // Control input matrix

    private float[] z_t;            // Measurement
    private float[][] H_t;          // Measurement matrix
    private float[] R_t;            // Measurement noise covariance

    private float[][] P_t;          // Covariance matrix predicted
    private float[][] P_tp;         // Covariance matrix previous
    private float[][] Q_t;          // Covariance noise

    private float[][] K_t;          // Kalman gain

    /*
    Prediction equations:
        X_t = F_t * X_t-1 + B_t * u_t + w_t
        P_t = F_t * P_t-1 * transp(F_t) + Q_t
     */
    private synchronized void predictCalculation(){
        x_tp = x_t.clone();
        P_tp = P_t.clone();

        x_t = Matrices.SumVectors( Matrices.Mul2byVector(F_t, x_tp), Matrices.Mul2byVector(B_t, u_t) );
        x_t = Matrices.SumVectors( x_t, w_t );

        float[][] tF_t = Matrices.Transp2by2(F_t);
        float[][] PMulTF = Matrices.Mul2by2(P_t, tF_t);
        P_t = Matrices.Mul2by2(F_t, PMulTF);
        P_t = Matrices.Add2by2(P_t, Q_t);
    }

    /*
    Kalman gain
        K_t = P_t * transp(H_t) * inv( H_t * P_t * trasnp(H_T) + R_t )
     */
    private synchronized void KalmanGain(){
        float[][] HtPtHtTrans = Matrices.Mul2by2(H_t, Matrices.Mul2by2(P_t, Matrices.Transp2by2(H_t)));
        float[][] InvertedMember = Matrices.Inv2by2(Matrices.Add2by2(HtPtHtTrans, Q_t));

        K_t = Matrices.Mul2by2(P_t, Matrices.Mul2by2(Matrices.Transp2by2(H_t), InvertedMember));
    }

    /*
    Measurement equations
        x_t = x_t + K_t * ( z_t - H_t * x_t )
        P_t = P_t - K_t * H_t * P_t
     */
    private synchronized void measurementCalculation(){
        float[] firstMember = Matrices.Mul2byVector(H_t, x_t);
        firstMember = Matrices.SubVectors(z_t, firstMember);
        firstMember = Matrices.Mul2byVector(K_t, firstMember);

        x_t = Matrices.SumVectors(x_t, firstMember);


        float[][] secondMember = Matrices.Mul2by2( K_t, Matrices.Mul2by2( H_t, P_t ) );

        P_t = Matrices.Sub2by2( P_t, secondMember );
    }


}
