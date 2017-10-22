package cmx.crestmuse.jp.data_collection_kinect_v2;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.util.Log;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by korona on 2016/09/20.
 */

    public class OrientationEstimater  {
    /*スマートフォンセンサーから得た値を処理するプログラム
    * センサー処理手順x:という感じで書いておくので検索してください
    * 基本的に各センサーの値が更新されるごとに自動処理される(5ms秒間隔くらい？)*/
        private float currentPosition;
        private printclass printclass = new printclass();
        public float Gx;
        public float Gy;
        public float Gz;
        public float v2;

        public float[] rotateM = new float[9];
        public float[] rotateM2 = new float[9];
        private float[] accMagOrientation = new float[3];

        // configurations
        //private boolean landscape = true; // swapXY
    /*
        private boolean zeroSnap = true;
        private boolean applyPressureHeight = false;
*/

        private float[] mag = new float[3];
        private float[] accel = new float[3];
        private float[] rotation_vec = new float[3];
        private long lastAccelTime = 0;
        private long resetTime = 0;
        public final Vector3f vVec2 = new Vector3f();
        public final Vector3f accVec = new Vector3f();
        public final Vector3f vVec = new Vector3f();
        public final Vector3f posVec = new Vector3f();
        private final Vector3f gyroVec = new Vector3f();
        private final Vector3f gravity = new Vector3f();

        private float[] outputPosition = new float[3];
        public long startTime = 0;
        public long currentTime = 0;
        private long secondTime = 0;
        /*
        private float pressureHeightErrorHigh = 250f; // mm
        public float pressureHeightCurrent = 0; // mm (from HeightBase)
        private float pressureHeightErrorLow = 250f; // mm
        private float pressureHeightErrorFactor = 0.000f; // m/s
        private float pressureHeightBase = 1013; // hPa
        private long pressureHeightErrorBaseTime = 0; // ns
        private final float[] pressHistory = new float[16];
        private int pressHistoryCount = 0;
*/
        private final float[] accHistory = new float[8];
        private int accHistoryCount = 0;

        public int eventCount = 0;
        public int flag = 0;
        public int flag2 = 0;

        /*追加変数*/
        public float[] fusedOrientation = new float[3];
        public static final int TIME_CONSTANT = 30;
        public static final float FILTER_COEFFICIENT = 0.98f;
        private Timer fuseTimer = new Timer();
        private float[] gyroMatrix = new float[9];
        private float[] gyroOrientation = new float[3];
        private float[] gyro = new float[3];
        private static final float NS2S = 1.0f / 1000000000.0f;
        private float timestamp;
        private boolean initState = true;
        public OrientationEstimater() {
            reset();
            fuseTimer.scheduleAtFixedRate(new calculateFusedOrientationTask(),
                    1000, TIME_CONSTANT);
            gyroMatrix[0] = 1f; gyroMatrix[1] = 0f; gyroMatrix[2] = 0f;
            gyroMatrix[3] = 0f; gyroMatrix[4] = 1f; gyroMatrix[5] = 0f;
            gyroMatrix[6] = 0f; gyroMatrix[7] = 0f; gyroMatrix[8] = 1f;
        }

        public void reset() { /*センサー値の初期化*/
            Log.d("OrientationEstimater", "reset");
            resetTime = System.currentTimeMillis(); /*現在の時間を返す*/
            posVec.set(0, 0, 0);
            vVec.set(0, 0, 0);
            vVec2.set(0, 0, 0);
            accVec.set(0, 0, 0);

        }

        /*背景楽曲終了時の処理:
        * ファイル書き込みを終了しファイルを閉じる
        */
        public void flagstop(long DelayTime1,long DelayTime2) {
            flag = 0;
            flag2 = 0;
            printclass.Printclose(DelayTime1,DelayTime2);
        }

        /*処理手順4:
        * ファイル書き込みを開始するフラグをたてる
        * 計測開始時間を決定
        * センサー値を初期化
        * */
        public void flagset() {
            if(flag == 0 && flag2 == 0) {
                flag = 1;
                startTime = System.currentTimeMillis();
                reset();
            }
        }

        public void print(double op){
            printclass.print_flow(op , secondTime);
    }

        /**
         * current orientation array If require matrix of OpenGL, it is necessary to
         * rotate in the following order: 1. roll 2. pitch 3. yaw
         *
         * @return float array [x,y,z]
         */


        /**
         * Current rotation matrix.
         *
         * @return
         */


        /**
         * @return float array [x,y,z] unit:mm
         */
        /*
        public float getPosition() {
            outputPosition[0] = position[0] + posVec.values[0];
            outputPosition[1] = position[1] + posVec.values[1];
            outputPosition[2] = position[2] + posVec.values[2];

            currentPosition = outputPosition[1];


            return currentPosition;
        }
*/

        public void onSensorEvent(SensorEvent event) {
            /*センサー処理手順1:
            * 加速度センサーの値から重力加速度センサーの値を引くことで加速度のみを抽出
            */
            if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
                accVec.values[0] = event.values[0];
                accVec.values[1] = event.values[1];
                accVec.values[2] = event.values[2];

                accVec.values[1] = accVec.values[1] * (float)Math.cos(fusedOrientation[1]) * (float)Math.cos(fusedOrientation[2]);
                //Log.d("OrientationEstimater","orientation1="+accMagOrientation[1]*180/Math.PI+",orientation2="+accMagOrientation[2]*180/Math.PI);
                /*前回の更新処理からの経過時間決定(timestampの単位はナノなので秒に修正)*/
                float dt = (event.timestamp - lastAccelTime) * 0.000000001f; // dt(sec)

                currentTime = System.currentTimeMillis() - startTime;
                secondTime = currentTime;
                /*計測スタートから500ms以上経過してから計測を開始する*/
                if (lastAccelTime > 0 && dt < 0.5f && System.currentTimeMillis() - resetTime > 500) {
                    // m/s^2
                    /*加速度の大きさが0.5以下の場合には0に収束(細かいブレの抑制)*/
                    if (accVec.values[0] <= 0.5f && accVec.values[0] >= -0.5f)
                        accVec.values[0] = 0;
                    if (accVec.values[1] <= 0.5f && accVec.values[1] >= -0.5f)
                        accVec.values[1] = 0;
                    if (accVec.values[2] <= 0.5f && accVec.values[2] >= -0.5f)
                        accVec.values[2] = 0;


               /* Log.d("Sensor", "currentPosition="+acc);*/

                    /*センサー処理手順2:加速度を時間に変換(値が小さすぎると変化がわかりずらいので100倍に拡大):*/
                    vVec.values[0] += accVec.values[0] * dt * 100;
                    vVec.values[1] += accVec.values[1] * dt * 100;
                    vVec.values[2] += accVec.values[2] * dt * 100;
                    // velocity limit
                    /*
                    if (vVec.length() > 5000) {
                        vVec.scale(0.95f);
                    }
                    */
                    boolean resting = false;
                    /*一定間隔時間の加速度の大きさの中の最大最小の大きさの間隔がある値以下ならば動いていないものと判定し速度を0に*/
                    accHistory[(accHistoryCount++) % accHistory.length] = accVec.length();
                    if (accHistoryCount > accHistory.length) {
                        final float l = accVec.length();
                        float min = l, max = l, sum = 0;
                        for (float a : accHistory) {
                            sum += a;
                            if (a > max)
                                max = a;
                            if (a < min)
                                min = a;
                        }
                        if (sum < 2.5f && max - min < 0.2f) {
                            resting = true;
                            vVec.scale(0.9f);
                            if (max - min < 0.1f) {
                                vVec.set(0, 0, 0);
                            }
                        }
                    }

                    /*センサー処理手順3:速度から求めた移動距離を求める*/
                    // position(mm)
                    if (vVec.length() > 0.5f) {
                        posVec.values[0] += vVec.values[0] * dt;
                        posVec.values[1] += vVec.values[1] * dt;
                        posVec.values[2] += vVec.values[2] * dt;
                    }
                    //posIntegretedError += vVec.length() * 0.0001f + accVec.length() * 0.1f;

                    // position limit
                    /*移動距離の限界値を設定,現状y軸しか使ってないのでy軸は180を限界値に設定*/
                    if (posVec.values[0] > 100) {
                        posVec.values[0] *= 0.9f;
                    } else if (posVec.values[0] < -100) {
                        posVec.values[0] *= 0.9f;
                    }

                    if (posVec.values[2] > 100) {
                        posVec.values[2] *= 0.9f;
                    } else if (posVec.values[2] < -100) {
                        posVec.values[2] *= 0.9f;
                    }

                    if (posVec.values[1] < -180) {
                        posVec.values[1] *= 0.8f;
                    } else if (posVec.values[1] > 180) {
                        posVec.values[1] *= 0.8f;
                    }
                    /*
                    // snap to 0
                    if (resting && zeroSnap && posIntegretedError > 0) {
                        if (posIntegretedError > 0) {
                            tmpVec.set(posVec.array());
                            posVec.scale(0.995f);
                            posIntegretedError -= tmpVec.sub(posVec).length();
                        }
                    }
                    */
                    /*センサー処理手順4:速度の変化量の計測
                    * 前回の更新時の速度からの変化量から動きを推測するため
                    */
                    v2 = vVec.values[1] - vVec2.values[1];
                    eventCount++;

                    /*センサー処理手順5:記録したセンサー値をファイル書き込み
                    書き込み処理自体はprintclass.javaで行うのでそちらを参照
                    */
                    if (startTime != 0 && secondTime < 70000 && flag == 1 && flag2 == 0) {
                        printclass.Printfile(flag, posVec.values[0], posVec.values[1], posVec.values[2], vVec.values[0], vVec.values[1], vVec.values[2], v2, accVec.values[0], accVec.values[1], accVec.values[2],gyroVec.values[0],gyroVec.values[1],gyroVec.values[2],rotation_vec[0],rotation_vec[1],rotation_vec[2], Gy, secondTime);
                    }
                        /*速度の記録を更新*/
                        vVec2.values[0] = vVec.values[0];
                        vVec2.values[1] = vVec.values[1];
                        vVec2.values[2] = vVec.values[2];

                    // Log.d("Sensor", "TYPE_PRESSURE pressureHeightCurrent: " + pressureHeightCurrent + ", " + posVec.values[1]);
                    /* 気圧センサーを使った処理，参考にしたプログラムに元々あった処理だが気圧センサーを使わないので凍結
                    if (pressureHeightErrorBaseTime > 0 && applyPressureHeight) {
                        float eh = pressureHeightErrorHigh + pressureHeightErrorFactor * ((event.timestamp - pressureHeightErrorBaseTime) * 0.000001f);
                        float el = pressureHeightErrorLow + pressureHeightErrorFactor * ((event.timestamp - pressureHeightErrorBaseTime) * 0.000001f);
                        if (posVec.values[1] > pressureHeightCurrent + eh) {
                            posVec.values[1] += (pressureHeightCurrent + eh - posVec.values[1]) * 0.1;
                        /*if (vVec.values[1] > 0) {
                            vVec.values[1] -= Math.abs(vVec.values[1]) * 0.3f;
                        }*/
                    /*
                        }
                        if (posVec.values[1] < pressureHeightCurrent - el) {
                            posVec.values[1] += (pressureHeightCurrent - el - posVec.values[1]) * 0.1;
                       /* if (vVec.values[1] < 0) {
                            vVec.values[1] += Math.abs(vVec.values[1]) * 0.3f;
                        }*/
                       /*
                        }
                    }
                    */
                }
                //センサー値更新時間を更新
                lastAccelTime = event.timestamp;
            } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                System.arraycopy(event.values,0,mag,0,mag.length);
                //mag[0] = event.values[0];
                //mag[1] = event.values[1];
                //mag[2] = event.values[2];
            } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY) { /*重力加速度センサーで重力加速度を取得*/
                Gx = event.values[0];
                Gy = event.values[1];
                Gz = event.values[2];
                gravity.set(Gx, Gy, Gz);
            /*
            Log.d("Sensor", "gravity.length="+gravity.length());*/
            } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) { /*処理ジャイロセンサーでスマートフォンの回転量を計測する*/
                /*if (lastGyroTime > 0) {
                    float dt = (event.timestamp - lastGyroTime) * 0.000000001f;
                    gyroVec.set(event.values[0],event.values[1],event.values[2]);
                    Matrix.rotateM(rotationMatrix, 0, gyroVec.length() * dt * 180 / PI, gyroVec.array()[0], gyroVec.array()[1], gyroVec.array()[2]);
                    posIntegretedError += gyroVec.length() * dt * 5.0f; // TODO: error ratio control.

                }
                accVec.set(accVecN.values);
                accVec.normalize();
                lastGyroTime = event.timestamp;*/
                gyroFunction(event);
            }

            else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
                System.arraycopy(event.values,0,rotation_vec,0,rotation_vec.length);
                /*
                float[] deltaRotationVector = new float[4];
                float[] values = new float[5];
                float[] deltaRotationMatrix = new float[9];
                deltaRotationVector[0] = event.values[0];
                deltaRotationVector[1] = event.values[1];
                deltaRotationVector[2] = event.values[2];
                deltaRotationVector[3] = event.values[3];
                SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);
                SensorManager.getOrientation(deltaRotationMatrix,values);
                //Log.d("OrientationEstimater",""+values[0]);
                //Log.d("OrientationEstimater",""+values[1]);
                //Log.d("OrientationEstimater",""+values[2]);
                */
            }
            else if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
                System.arraycopy(event.values,0,accel,0,accel.length);
                //accel[0] = event.values[0];
                //accel[1] = event.values[1];
                //accel[2] = event.values[2];
                calculateAccMagOrientation();
            }


            // adjust ground vector.
            /*センサー処理手順6:ジャイロセンサーの値を使って加速度の値を修正するセクション
            * この部分については処理の理解が不十分なところがあるんで先輩のジャイロを使って加速度を修正するように作り直していただきたいです．
            */
            /*
            if (gyroVec.length() < 0.3f && Math.abs(accVecN.length() - Gy) < 0.5f) {
                // estimated ground vec.
                Matrix.multiplyMV(tmpVec.array(), 0, rotationMatrix, 0, accVec.values, 0);
                float theta = (float) Math.acos(tmpVec.dot(gravityVecI));
                if (theta > 0) {
                    float[] cross = tmpVec.cross(gravityVecI).normalize().array();
                    float factor = (System.currentTimeMillis() - resetTime < 500) ? 0.9f : 0.0005f;

                    Matrix.rotateM(rotationMatrix, 0, theta * 180 / PI * factor, cross[0], cross[1], cross[2]);
                    Matrix.setRotateM(rotationMatrix_t1, 0, theta * 180 / PI * factor, cross[0], cross[1], cross[2]);
                    Matrix.multiplyMM(rotationMatrix_t2, 0, rotationMatrix_t1, 0, rotationMatrix, 0);
                    float tm[] = rotationMatrix_t2;
                    rotationMatrix_t2 = rotationMatrix;
                    rotationMatrix = tm;
                }
            }
            */
        }
    /*傾きを計測するための関数一覧*/
    public void calculateAccMagOrientation() {
        if(SensorManager.getRotationMatrix(rotateM, null, accel, mag)) {
            SensorManager.remapCoordinateSystem(rotateM, SensorManager.AXIS_X, SensorManager.AXIS_Z, rotateM2);
            //SensorManager.remapCoordinateSystem(rotateM, SensorManager.AXIS_X,SensorManager.AXIS_Y,rotateM2);
            SensorManager.getOrientation(rotateM2, accMagOrientation);
        }
    }

    private void getRotationVectorFromGyro(float[] gyroValues,
                                           float[] deltaRotationVector,
                                           float timeFactor)
    {
        float[] normValues = new float[3];

        // Calculate the angular speed of the sample
        float omegaMagnitude =
                (float)Math.sqrt(gyroValues[0] * gyroValues[0] +
                        gyroValues[1] * gyroValues[1] +
                        gyroValues[2] * gyroValues[2]);

        // Normalize the rotation vector if it's big enough to get the axis
        if(omegaMagnitude >  0.000000001f) {
            normValues[0] = gyroValues[0] / omegaMagnitude;
            normValues[1] = gyroValues[1] / omegaMagnitude;
            normValues[2] = gyroValues[2] / omegaMagnitude;
        }

        // Integrate around this axis with the angular speed by the timestep
        // in order to get a delta rotation from this sample over the timestep
        // We will convert this axis-angle representation of the delta rotation
        // into a quaternion before turning it into the rotation matrix.
        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
        deltaRotationVector[3] = cosThetaOverTwo;
    }

    public void gyroFunction(SensorEvent event) {
        // don't start until first accelerometer/magnetometer orientation has been acquired
        if (accMagOrientation == null)
            return;

        // initialisation of the gyroscope based rotation matrix
        if(initState) {
            float[] initMatrix = new float[9];
            initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
            float[] test = new float[3];
            SensorManager.getOrientation(initMatrix, test);
            gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
            initState = false;
        }

        // copy the new gyro values into the gyro array
        // convert the raw gyro data into a rotation vector
        float[] deltaVector = new float[4];
        if(timestamp != 0) {
            final float dT = (event.timestamp - timestamp) * NS2S;
            System.arraycopy(event.values, 0, gyro, 0, 3);
            getRotationVectorFromGyro(gyro, deltaVector, dT / 2.0f);
        }

        // measurement done, save current time for next interval
        timestamp = event.timestamp;

        // convert rotation vector into rotation matrix
        float[] deltaMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);

        // apply the new rotation interval on the gyroscope based rotation matrix
        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);

        // get the gyroscope based orientation from the rotation matrix
        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
    }

    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float)Math.sin(o[1]);
        float cosX = (float)Math.cos(o[1]);
        float sinY = (float)Math.sin(o[2]);
        float cosY = (float)Math.cos(o[2]);
        float sinZ = (float)Math.sin(o[0]);
        float cosZ = (float)Math.cos(o[0]);

        // rotation about x-axis (pitch)
        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

        // rotation about y-axis (roll)
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

        // rotation about z-axis (azimuth)
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

        // rotation order is y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }
    class calculateFusedOrientationTask extends TimerTask {
        public void run() {
            float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;
            fusedOrientation[0] =
                    FILTER_COEFFICIENT * gyroOrientation[0]
                            + oneMinusCoeff * accMagOrientation[0];

            fusedOrientation[1] =
                    FILTER_COEFFICIENT * gyroOrientation[1]
                            + oneMinusCoeff * accMagOrientation[1];

            fusedOrientation[2] =
                    FILTER_COEFFICIENT * gyroOrientation[2]
                            + oneMinusCoeff * accMagOrientation[2];

            // overwrite gyro matrix and orientation with fused orientation
            gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
            System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);
            //Log.d("OrientationEstimater","orientation1="+fusedOrientation[1]*180/Math.PI+",orientation2="+fusedOrientation[2]*180/Math.PI);
        }
    }

}



