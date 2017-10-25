package cmx.crestmuse.jp.data_collection_kinect_v2;

/**
 * Created by korona on 2017/04/11.
 */

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.AsyncTask;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Set;
import java.util.UUID;

public class BluetoothTask {
    private static final String TAG = "BluetoothTask";

    /**
     * UUIDはサーバと一致している必要がある。
     * - 独自サービスのUUIDはツールで生成する。（ほぼ乱数）
     * - 注：このまま使わないように。
     */
    private static final UUID APP_UUID = UUID.fromString("17fcf242-f86d-4e35-805e-546ee3040b84");

    private MainActivity activity;
    private BluetoothAdapter bluetoothAdapter;
    private BluetoothDevice bluetoothDevice = null;
    private BluetoothSocket bluetoothSocket;
    private InputStream btIn;
    private OutputStream btOut;
    public long StartTime_send = 0;
    public long EndTime_send = 0;

    public BluetoothTask(MainActivity activity) {
        this.activity = activity;
    }

    /**
     * Bluetoothの初期化。
     */
    public void init() {
        // BTアダプタ取得。取れなければBT未実装デバイス。
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (bluetoothAdapter == null) {
            activity.errorDialog("This device is not implement Bluetooth.");
            return;
        }
        // BTが設定で有効になっているかチェック。
        if (!bluetoothAdapter.isEnabled()) {
            // TODO: ユーザに許可を求める処理。
            activity.errorDialog("This device is disabled Bluetooth.");
            return;
        }
    }

    /**
     * @return ペアリング済みのデバイス一覧を返す。デバイス選択ダイアログ用。
     */
    public Set<BluetoothDevice> getPairedDevices() {
        return bluetoothAdapter.getBondedDevices();
    }

    /**
     * 非同期で指定されたデバイスの接続を開始する。
     * - 選択ダイアログから選択されたデバイスを設定される。
     *
     * @param device 選択デバイス
     */
    public void doConnect(BluetoothDevice device) {
        bluetoothDevice = device;
        try {
            bluetoothSocket = bluetoothDevice.createRfcommSocketToServiceRecord(APP_UUID);
            new ConnectTask().execute();
        } catch (IOException e) {
            Log.e(TAG, e.toString(), e);
            activity.errorDialog(e.toString());
        }
    }

    //経過時間を計算してサーバ側に送信するタスクを生成

    /*処理手順3:Kincet側に実験開始合図を送信し，合図を受信したKincet側からの合図を受信し通信の遅延時間を確定*/
    public void signalsend() {
        /*
        new SignalSend().execute("お試し");
        */
        try {
            byte[] buff = new byte[512];
            StartTime_send = System.currentTimeMillis();
            btOut.write("シグナル".getBytes());
            btIn.read(buff);
            EndTime_send = System.currentTimeMillis();
            Log.i("ddddd",String.valueOf(EndTime_send)+"ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");
        }catch (Throwable t) {
            doClose();
        }

    }

    /**
     * 非同期でBluetoothの接続を閉じる。
     */
    public void doClose() {
        new CloseTask().execute();
    }

    /**
     * AsyncTaskは非同期処理のための
     * Bluetoothと接続を開始する非同期タスク。
     * - 時間がかかる場合があるのでProcessDialogを表示する。
     * - 双方向のストリームを開くところまで。
     */
    private class ConnectTask extends AsyncTask<Void, Void, Object> {
        @Override
        protected void onPreExecute() {
            activity.showWaitDialog("Connect Bluetooth Device.");
        }

        //非同期処理の前に行われる処理
        @Override
        protected Object doInBackground(Void... params) {
            //非同期で処理を実行
            try {
                bluetoothSocket.connect();
                btIn = bluetoothSocket.getInputStream();
                btOut = bluetoothSocket.getOutputStream();
            } catch (Throwable t) {
                doClose();
                return t;
            }
            return null;
        }

        @Override
        protected void onPostExecute(Object result) {
            //接続エラーが起きた場合エラーダイアログを表示
            if (result instanceof Throwable) {
                Log.e(TAG, result.toString(), (Throwable) result);
                activity.errorDialog(result.toString());
            } else {
                activity.hideWaitDialog();
            }
        }
    }

    /**
     * Bluetoothと接続を終了する非同期タスク。
     * - 不要かも知れないが念のため非同期にしている。
     */
    private class CloseTask extends AsyncTask<Void, Void, Object> {
        @Override
        protected Object doInBackground(Void... params) {
            try {
                try {
                    btOut.close();
                } catch (Throwable t) {/*ignore*/}
                try {
                    btIn.close();
                } catch (Throwable t) {/*ignore*/}
                bluetoothSocket.close();
            } catch (Throwable t) {
                return t;
            }
            return null;
        }

        @Override
        protected void onPostExecute(Object result) {
            if (result instanceof Throwable) {
                Log.e(TAG, result.toString(), (Throwable) result);
                activity.errorDialog(result.toString());
            }
        }
    }


    /*
    サーバーに経過時間を送信する非同期タスク
     */
    private class TimeSend extends AsyncTask<String, Void, Void> {
        @Override
        protected Void doInBackground(String... params) {
            try {
                btOut.write(params[0].getBytes());
                btOut.flush();

            } catch (Throwable t) {
                doClose();
            }
            return (null);
        }
    }

    private class SignalSend extends AsyncTask<String, Void, Void> {
        @Override
        protected Void doInBackground(String... params) {
            try {


                byte[] buff = new byte[512];
                /*
                Log.d("bluetooth", "待ち状態");
                int len = btIn.read(buff); // TODO:ループして読み込み
                Log.d("bluetooth", "もらった");
                //String s = new String(buff,0,len);
                //String resultString = new String(buff,"UTF-8");
                if (len == 4) {
                    Log.d("bluetooth", "課題クリア");
                }
                Log.d("bluetooth", "長さ"+len);
                */
                //btOut.write("シグナル".getBytes());
                Log.d("bluetooth", "待ち状態");
                btIn.read(buff);
                Log.d("bluetooth", "もらった");
                /*
                long endtime = System.currentTimeMillis() - starttime;
                Log.d("bluetooth","endtime="+endtime);
                */
            } catch (Throwable t) {
                Log.d("bluetooth", "エラー");
                doClose();
            }
            return (null);
        }

    }
}