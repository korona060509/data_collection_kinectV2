package cmx.crestmuse.jp.data_collection_kinect_v2;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothDevice;
import android.content.DialogInterface;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.MediaPlayer;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;

import java.util.List;
import java.util.Set;

public class MainActivity extends Activity implements SensorEventListener,CameraBridgeViewBase.CvCameraViewListener2 {

/*カメラに使用する変数一覧
* */
    // カメラ
    private CameraBridgeViewBase mOpenCvCameraView;
    // カメラ画像
    private Mat image, image_small;
    // オプティカルフロー用
    private Mat image_prev, image_next;
    private MatOfPoint2f pts_prev, pts_next;
    //カメラが起動中かのフラグ
    private boolean isCameraOpened = false;

    // OpenCVライブラリのロード
    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    pts_prev = new MatOfPoint2f();
                    pts_next = new MatOfPoint2f();
                    break;
                }
                default: {
                    super.onManagerConnected(status);
                    break;
                }
            }
        }
    };

    public double move_total_x = 0;
    public double move_total_y = 0;
    private int count = 0;

    // コンストラクタ
    public MainActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }
/*
* kinectと通信してトレーニングデータを収集するアプリ
* 作業手順xを順に見ていってください(例:作業手順1 みたいに文章検索してください)
*/
    private static final String TAG = "OCVSample::Activity";
    private final static int DEVICES_DIALOG = 1;
    private final static int ERROR_DIALOG = 2;

    private BluetoothTask bluetoothTask = new BluetoothTask(this);
    public BluetoothDevice device;

    private ProgressDialog waitDialog;
    private String errorMessage = "";
    private OrientationEstimater orientationEstimater = new OrientationEstimater();
    /*
    private SoundPool soundPool;
    private int[] soundIDList = new int[20];
    */
    private MediaPlayer mediaPlayer = null;

    public TextView orientationText1;
    public TextView orientationText2;
    public TextView name_box;
    public String subject_name;



    @Override /*アプリ起動後に実行*/
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        orientationText1 = (TextView) findViewById(R.id.Orientation1);
        orientationText2 = (TextView) findViewById(R.id.Orientation2);
        name_box = (EditText) findViewById(R.id.Name_Box);

        subject_name = "example";
        /*カメラviewの処理*/
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.camera_view);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        /*背景楽曲を選択*/
        mediaPlayer = MediaPlayer.create(this, R.raw.practicedata);


        /*接続リセットボタン:bluetooth接続状態のリセットしてアプリをリスタートする*/
        final Button resetButton = (Button) findViewById(R.id.resetBtn);
        resetButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                restart();
            }
        });

        /*実験開始ボタン(処理手順2):
        *1. 接続機器(Kinectのデータを収集するアプリ)とのデータの送受信を開始する
        *2. スマートフォンセンサーのデータ収集の開始
        *3. 背景楽曲再生
        */
        final Button SignalButton = (Button)findViewById(R.id.ShakeButton);

        final Button getText_Btn = (Button) findViewById(R.id.getText_btn);
        getText_Btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                subject_name = name_box.getText().toString();
                orientationEstimater.printfile_setup(subject_name);
                if (SignalButton.getVisibility() == View.INVISIBLE) {
                    // 表示なら非表示する
                    SignalButton.setVisibility(View.VISIBLE);
                }
            }
        });
        /*playボタンに関する処理*/
        SignalButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(subject_name != "example") {
                    if (resetButton.getVisibility() == View.VISIBLE) {
                        // 表示なら非表示する
                        resetButton.setVisibility(View.INVISIBLE);
                    }
                    if (SignalButton.getVisibility() == View.VISIBLE) {
                        // 表示なら非表示する
                        SignalButton.setVisibility(View.INVISIBLE);
                    }
                /*処理手順3はBluetoothTask.javaに飛ぶ*/
                    bluetoothTask.signalsend();

                /*処理手順4はOrientationEstimater.javaに飛ぶ*/
                    orientationEstimater.flagset();
                    isCameraOpened = true;
                    mediaPlayer.start();
                }
                /*処理手順Extra:アプリ開始時にPrintClassのコンストラクタを呼びだしで書き込むファイルを生成printclass.javaを参照*/

                /*実験の前準備処理はここまでで終了，残りはOrientationEstimater.java内のセンサー値の処理部分参照*/

                /*背景楽曲終了時の処理:
                *1. 通信遅延時間の計測
                *2. スマートフォンセンサーの計測のストップ
                * */
                mediaPlayer.setOnCompletionListener(new MediaPlayer.OnCompletionListener(){
                    @Override
                    public void onCompletion(MediaPlayer mediaplayer){
                        /*
                        Delay1：通信の往復にかかった時間
                        Delay2：通信終了からタイマー開始までの時間
                         */
                        long DelayTime1 = bluetoothTask.EndTime_send - bluetoothTask.StartTime_send;
                        long DelayTime2 = orientationEstimater.startTime - bluetoothTask.EndTime_send;
                        orientationEstimater.stopWiter(DelayTime1,DelayTime2);
                        Log.d("ddddttttttt", "startTime="+orientationEstimater.startTime+",endTime="+bluetoothTask.EndTime_send);

                        if (resetButton.getVisibility() == View.INVISIBLE) {
                            // 非表示されている時に表示に
                            resetButton.setVisibility(View.VISIBLE);
                        }
                        if (SignalButton.getVisibility() == View.INVISIBLE) {
                            // 非表示されている時に表示に
                            SignalButton.setVisibility(View.VISIBLE);
                        }
                        Log.v("MediaPlayer", "onCompletion");
                        isCameraOpened = false;
                    }
                });
            }


        });
        final Handler handler = new Handler();
        handler.post(new Runnable() { /*UI部分の操作のためにスレッドを送る*/
            @Override
            public void run() {/*y軸の高さを画面に表示(特にデータとるのに必要ないので消してもOK)*/
                ((TextView) findViewById(R.id.Orientation1)).setText(""+orientationEstimater.fusedOrientation[1]*180/Math.PI);
                ((TextView) findViewById(R.id.Orientation2)).setText(""+orientationEstimater.fusedOrientation[2]*180/Math.PI);

                handler.postDelayed(this, 30);
            }
        });
    }

    /*処理手順1:アプリ開始時やリスタート時にBluetooth接続機器を選択*/
    @SuppressWarnings("deprecation") //非推奨APIに関する警告無視
    @Override
    protected void onResume() {
        super.onResume();
        // Bluetooth初期化
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
        bluetoothTask.init();

        // ペアリング済みデバイスの一覧を表示してユーザに選ばせる。
        showDialog(DEVICES_DIALOG);
    }

    //アプリ終了時にサーバーとの接続を閉じる
    @Override
    protected void onDestroy() {
        if(orientationEstimater.canWrite) {
            long DelayTime1 = bluetoothTask.EndTime_send - bluetoothTask.StartTime_send;
            long DelayTime2 = orientationEstimater.startTime - bluetoothTask.EndTime_send;
            orientationEstimater.stopWiter(DelayTime1, DelayTime2);
        }
        bluetoothTask.doClose();
        super.onDestroy();
    }


    //リセットボタンが押された時にactivityを再起動する
    protected void restart() {
        Intent intent = this.getIntent();
        this.finish();
        this.startActivity(intent);
    }

    // カメラ開始時
    public void onCameraViewStarted(int width, int height) {
        image = new Mat(height, width, CvType.CV_8UC3);
        image_small = new Mat(height/8, width/8, CvType.CV_8UC3);
        image_prev = new Mat(image_small.rows(), image_small.cols(), image_small.type());
        image_next = new Mat(image_small.rows(), image_small.cols(), image_small.type());
    }

    // カメラ停止時
    public void onCameraViewStopped() {
    }

    // 画像取得時
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        // 縮小
        image = inputFrame.rgba();
        Imgproc.resize(image, image_small, image_small.size(), 0, 0, Imgproc.INTER_NEAREST);

        // グレースケール
        Mat gray = new Mat(image_small.rows(), image_small.cols(), CvType.CV_8UC1);
        Imgproc.cvtColor(image_small, gray, Imgproc.COLOR_RGB2GRAY);

        // 特徴点抽出
        MatOfPoint features = new MatOfPoint();
        //Imgproc.goodFeaturesToTrack(image(画像), vector&corner(検出されたコーナーの出力), int(出力するコーナーの最大数), double(コーナーの品質度), )
        Imgproc.goodFeaturesToTrack(gray, features, 50, 0.01, 10);
        Log.i(TAG, "called onCreate");

        // 特徴点が見つかった
        if (features.total() > 0 && isCameraOpened == true) {
            // 過去のデータが存在する
            if (pts_prev.total() > 0) {
                // 現在のデータ
                gray.copyTo(image_next);
                pts_next = new MatOfPoint2f(features.toArray());

                // オプティカルフロー算出
                MatOfByte status = new MatOfByte();
                MatOfFloat err = new MatOfFloat();
                Video.calcOpticalFlowPyrLK(image_prev, image_next, pts_prev, pts_next, status, err);

                // 表示
                long flow_num = status.total();
                if (flow_num > 0) {
                    List<Byte> list_status = status.toList();
                    List<Point> list_features_prev = pts_prev.toList();
                    List<Point> list_features_next = pts_next.toList();
                    double scale_x = image.cols() / image_small.cols();
                    double scale_y = image.rows() / image_small.rows();
                    double move_x = 0;
                    double move_y = 0;
                    double move_average_x = 0;
                    double move_average_y = 0;
                    count = 0;
                    for (int i = 0; i < flow_num; i++) {
                        if (list_status.get(i) == 1) {
                            Point p1 = new Point();
                            p1.x = list_features_prev.get(i).x * scale_x;
                            p1.y = list_features_prev.get(i).y * scale_y;
                            //Core.circle(image, p1, 3, new Scalar(255,0,0), -1, 8, 0 );
                            Point p2 = new Point();
                            p2.x = list_features_next.get(i).x * scale_x;
                            p2.y = list_features_next.get(i).y * scale_y;
                            //Core.circle(image, p2, 3, new Scalar(255,255,0), -1, 8, 0 );

                            move_x = p2.x - p1.x;
                            move_y = p2.y - p1.y;
                            // フロー描画
                            //int thickness = 5;
                            //Imgproc.line(image, p1, p2, new Scalar(0,255,0), thickness);
                            count++;
                        }
                    }
                    move_average_x = move_x / count;
                    move_average_y = move_y / count;
                    if(isCameraOpened == true) {
                        orientationEstimater.printFlow(move_average_x,move_average_y);
                    }
                    /*
                    if(!Double.isNaN(move_average_y)) {
                        move_total_x += move_average_x;
                        move_total_y += move_average_y;
                    }
                    if(isCameraOpened == true) {
                        orientationEstimater.print(move_total_y);
                    }
                    */
                }
            }
            //Imgproc.line(image, new Point(50,100), new Point(150,300), new Scalar(0,255,0), 5);
            // 過去のデータ
            gray.copyTo(image_prev);
            pts_prev = new MatOfPoint2f(features.toArray());
        }
        //Log.i(TAG, "move_total_y="+move_total_y);
        return image;
    }

    /*センサー使用に必要な記述なので特にいじる必要なし*/
    @Override
    protected void onStart() {
        super.onStart();
        SensorManager sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        Sensor sensorAccelLinear = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        Sensor sensorAccel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor sensorGyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        Sensor sensorMag = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        //Sensor sensorPressure = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE);
        Sensor sensorGravity = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        Sensor sensorRotation = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        /*
        Sensor sensorProximity = sensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY);
        */
        sensorManager.registerListener(this, sensorAccelLinear, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, sensorAccel, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, sensorGyro, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, sensorMag, SensorManager.SENSOR_DELAY_FASTEST);
        //sensorManager.registerListener(this, sensorPressure, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, sensorGravity, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, sensorRotation, SensorManager.SENSOR_DELAY_FASTEST);

        /*
        sensorManager.registerListener(this, sensorProximity, SensorManager.SENSOR_DELAY_FASTEST);
        */

    }
    /*センサー使用に必要な処理なので特にいじる必要なし*/
    @Override
    protected void onStop() {
        SensorManager sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        sensorManager.unregisterListener(this);
        super.onStop();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        orientationEstimater.onSensorEvent(event);
    }


    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    // 以下、動作に合わせて表示するダイアログ関連
    //Bluetooth接続時に各処理に対してダイアログを表示する

    @Override
    protected Dialog onCreateDialog(int id) {
        if (id == DEVICES_DIALOG) return createDevicesDialog();
        if (id == ERROR_DIALOG) return createErrorDialog();
        return null;
    }
    @SuppressWarnings("deprecation")
    @Override
    protected void onPrepareDialog(int id, Dialog dialog) {
        if (id == ERROR_DIALOG) {
            ((AlertDialog) dialog).setMessage(errorMessage);
        }
        super.onPrepareDialog(id, dialog);
    }

    public Dialog createDevicesDialog() {
        AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(this);
        alertDialogBuilder.setTitle("Select device");

        // ペアリング済みデバイスをダイアログのリストに設定する。
        Set<BluetoothDevice> pairedDevices = bluetoothTask.getPairedDevices();
        final BluetoothDevice[] devices = pairedDevices.toArray(new BluetoothDevice[0]);
        String[] items = new String[devices.length];
        for (int i=0;i<devices.length;i++) {
            items[i] = devices[i].getName();
        }

        alertDialogBuilder.setItems(items, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
                // 選択されたデバイスを通知する。そのまま接続開始。
                device = devices[which];
                bluetoothTask.doConnect(devices[which]);
            }
        });
        alertDialogBuilder.setCancelable(false);
        return alertDialogBuilder.create();
    }

    @SuppressWarnings("deprecation")
    public void errorDialog(String msg) {
        if (this.isFinishing()) return;
        this.errorMessage = msg;
        this.showDialog(ERROR_DIALOG);
    }
    public Dialog createErrorDialog() {
        AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(this);
        alertDialogBuilder.setTitle("Error");
        alertDialogBuilder.setMessage("");
        alertDialogBuilder.setPositiveButton("Exit", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
                finish();
            }
        });
        return alertDialogBuilder.create();
    }
    //サーバとの接続待ち状態に表示するダイアログ
    public void showWaitDialog(String msg) {
        if (waitDialog == null) {
            waitDialog = new ProgressDialog(this);
        }
        waitDialog.setMessage(msg);
        waitDialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
        waitDialog.show();
    }
    public void hideWaitDialog() {
        waitDialog.dismiss();
    }

}