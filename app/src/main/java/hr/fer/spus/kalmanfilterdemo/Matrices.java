package hr.fer.spus.kalmanfilterdemo;

/**
 * Helper matrix functions
 */
public class Matrices {

    protected static float[] SumVectors(float[] a, float[] b){
        float[] res = a.clone();
        res[0] += b[0]; res[1] += b[1];

        return res;
    }

    protected static float[] SubVectors(float[] a, float[] b){
        float[] res = a.clone();
        res[0] -= b[0]; res[1] -= b[1];

        return res;
    }

    protected static float[][] Transp2by2(float[][] a){
        float[][] res = new float[2][2];
        for(int i = 0; i < 2; i++)
            for(int j = 0; j < 2; j++)
                res[j][i] = a[i][j];

        return res;
    }

    protected static float[][] Add2by2(float[][] a, float[][] b){
        float[][] res = a.clone();
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                res[i][j] += b[i][j];

        return res;
    }

    protected static float[][] Sub2by2(float[][] a, float[][] b) {
        float[][] res = a.clone();
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                res[i][j] -= b[i][j];

        return res;
    }

        protected static float[][] Mul2by2(float[][] a, float[][] b){
        float[][] res = new float[2][2];
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                for (int k = 0; k < 2; k++)
                    res[i][j] += a[i][k] * b[k][j];

        return res;
    }

    protected static float[] Mul2byVector(float[][] a, float[] b){
        float[] res = new float[2];
        res[0] = a[0][0] * b[0] + a[0][1] * b[1];
        res[1] = a[1][0] * b[0] + a[1][1] * b[1];

        return res;
    }

    protected  static float[][] Inv2by2(float[][] a){
        float det = a[0][0] * a[1][1] - a[0][1] * a[1][0];

        if (det == 0)
            throw new RuntimeException("Matrix inverse non-existent");

        float[][] res = new float[][]{{a[1][1], -a[0][1]}, {-a[1][0], a[0][0]}};
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                res[i][j] /= det;

        return res;
    }

    protected static float[][] Transp3by3(float[][] a){
        float[][] res = new float[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                res[j][i] = a[i][j];

        return res;
    }

    protected static float[][] Add3by3(float[][] a, float[][]b){
        float[][] res = a.clone();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                res[i][j] += b[i][j];
        return res;
    }

    protected static float[][] Mul3by3(float[][] a, float[][]b){
        float[][] res = new float[3][3];
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                for (int k = 0; k < 3; k++){
                    res[i][j] += a[i][k] * b[k][j];
                }
            }
        }

        return res;
    }

    protected static float[][] Inv3by3(float[][] a){
        float det = a[0][0] * a[1][1] * a[2][2] + a[0][1] * a[1][2] * a[2][0] + a[0][2] * a[1][0] * a[2][1]
                - a[0][0] * a[1][2] * a[2][1] - a[0][1] * a[1][0] * a[2][2] - a[0][2] * a[1][1] * a[2][0];

        if (det == 0)
            throw new RuntimeException("Matrix inverse non-existent");

        float[][] b = new float[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                b[i][j] = Matrices.subDet3by3(a,i,j) / det;

        return b;
    }

    protected static float subDet3by3(float[][] a, int r, int c){
        float[][] subM = new float[2][2];
        int rr = 0; int cc = 0;

        for (int i = 0; i < 3; i++){
            if (i != r){
                for (int j = 0; j < 3; j++){
                    if (j != c){
                        subM[rr][cc] = a[i][j];
                        cc++;
                    }
                }
                rr++;
            }
        }

        float det = subM[0][0] * subM[1][1] - subM[0][1] * subM[1][0];
        return det;
    }
}
