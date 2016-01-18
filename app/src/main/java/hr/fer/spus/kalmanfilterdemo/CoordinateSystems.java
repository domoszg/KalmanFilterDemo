package hr.fer.spus.kalmanfilterdemo;

/**
 * Helper class for coordinate system manipulation
 * Euler angles are defined as:
 *   [ yaw, pitch, roll ]
 */
public class CoordinateSystems {
    private float[] eulerAngles;

    public CoordinateSystems(float[] eulerAngles) {
        this.eulerAngles = eulerAngles;
    }
    public CoordinateSystems(float yaw, float pitch, float roll){
        this(new float[]{yaw, pitch, roll});
    }

    public static float[][] getRotationMatrix(float[] eulerAngles){ return (new CoordinateSystems(eulerAngles)).getRotationMatrix(); }
    public float[][] getRotationMatrix(){
        float[][] R = new float[3][3];
        float yaw = eulerAngles[0]; float pitch = eulerAngles[1]; float roll = eulerAngles[2];

        float cosy = (float) Math.cos(yaw); float siny = (float) Math.sin(yaw);
        float cosp = (float) Math.cos(pitch); float sinp = (float) Math.sin(pitch);
        float cosr = (float) Math.cos(roll); float sinr = (float) Math.sin(roll);

        R[0][0] = cosr * cosy + sinr * siny * sinp;
        R[0][1] = cosy * sinr * sinp - cosr * siny;
        R[0][2] = cosp * sinr;

        R[1][0] = cosp * siny;
        R[1][1] = cosy * cosp;
        R[1][2] = - sinp;

        R[2][0] = cosr * siny * sinp - cosy * sinr;
        R[2][1] = sinr * siny + cosr * cosy * sinp;
        R[2][2] = cosr * cosp;

        return R;
    }

    public float[] rotateSystem(float[] xyz){
        float[] newXYZ = new float[3];
        float[][] R = this.getRotationMatrix();

        newXYZ[0] = xyz[0] * R[0][0] + xyz[1] * R[0][1] + xyz[2] * R[0][2];
        newXYZ[1] = xyz[0] * R[1][0] + xyz[1] * R[1][1] + xyz[2] * R[1][2];
        newXYZ[2] = xyz[0] * R[2][0] + xyz[1] * R[2][1] + xyz[2] * R[2][2];

        return newXYZ;
    }
}
