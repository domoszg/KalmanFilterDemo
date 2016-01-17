package hr.fer.spus.kalmanfilterdemo;

/**
 * Helper class for coordinate system manipulation
 */
public class CoordinateSystems {
    private float[] eulerAngles;

    public CoordinateSystems(float[] eulerAngles) {
        this.eulerAngles = eulerAngles;
    }
    public CoordinateSystems(float yaw, float pitch, float roll){
        this(new float[]{yaw, pitch, roll});
    }

    public static float[] getRotationMatrix(float[] eulerAngles){ return (new CoordinateSystems(eulerAngles)).getRotationMatrix(); }
    public float[] getRotationMatrix(){
        float[] R = new float[9];
        float yaw = eulerAngles[0]; float pitch = eulerAngles[1]; float roll = eulerAngles[2];

        float cosy = (float) Math.cos(yaw); float siny = (float) Math.sin(yaw);
        float cosp = (float) Math.cos(pitch); float sinp = (float) Math.sin(pitch);
        float cosr = (float) Math.cos(roll); float sinr = (float) Math.sin(roll);

        R[0] = cosr * cosy - sinr * siny * sinp;
        R[1] = - cosp * siny;
        R[2] = cosy * sinr + cosr * siny * sinp;

        R[3] = cosr * siny + cosy * sinr * sinp;
        R[4] = cosy * cosp;
        R[5] = sinr * siny - sinp * cosr * cosy;

        R[6] = - cosp * sinr;
        R[7] = sinp;
        R[8] = cosr * cosp;

        return R;
    }

    public float[] rotateSystem(float[] xyz){
        float[] newXYZ = new float[3];
        float[] R = this.getRotationMatrix();

        newXYZ[0] = xyz[0] * R[0] + xyz[1] * R[1] + xyz[2] * R[2];
        newXYZ[1] = xyz[0] * R[3] + xyz[1] * R[4] + xyz[2] * R[5];
        newXYZ[2] = xyz[0] * R[6] + xyz[1] * R[7] + xyz[2] * R[8];

        return newXYZ;
    }
}
