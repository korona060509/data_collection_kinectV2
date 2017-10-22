package cmx.crestmuse.jp.data_collection_kinect_v2;

import android.os.Environment;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.util.Date;

/**
 * Created by korona on 2016/10/07.
 */
public class printclass  {
    private PrintWriter out;
    private PrintWriter out2;
    //MainActivity mainActivity = new MainActivity();
    /*コンストラクタ呼び出し時に書き込むファイルを生成する*/
    public printclass() {
        try {      /*実験日時をファイル名に指定するに確定*/
                    Date date = new Date();
                    /*ファイルの書き込み先と生成するファイル名の決定(勝手に変えてください)*/
                    String path = Environment.getExternalStorageDirectory().getPath() + "/Android/data/trainingdata/"+date.toString()+"_sensordata.txt";
                    String path2 = Environment.getExternalStorageDirectory().getPath() + "/Android/data/trainingdata/"+date.toString()+"_opticalflow.txt";

                    File file = new File(path);
                    File file2 = new File(path2);
                    /*書き込みファイルを生成*/
                    file.createNewFile();
                    file2.createNewFile();
                    out = new PrintWriter(new OutputStreamWriter(new FileOutputStream(file), "SHIFT_JIS"));
                    out2 = new PrintWriter(new OutputStreamWriter(new FileOutputStream(file2), "SHIFT_JIS"));

                    out.print("移動距離(x,y,z),速度(x,y,z),加速度(x,y,z),ジャイロ(x,y,z),回転軌道(x,y,z),重力加速度y\r\n");
                    out2.print("オプティカルフロ-y軸,時間\r\n");
        }catch(IOException e){

        }
    }
    public void Printfile(int i, float pos0, float pos1, float pos2, float v0, float v1, float v2, float vc, float acc0, float acc1, float acc2,float gyro0,float gyro1,float gyro2, float rotation0, float rotation1, float rotation2,float Gy, long secondTime) {

        if (i == 1) {
                /*ファイルに記録したい値を書き込むセクション
                * 記録する値を追加する場合には同じように書き足してください
                * */
                //optical_flow = (float)mainActivity.move_total_y;
                out.print(secondTime + "\r\n");
                out.print("(" + String.format("%.2f", pos0) + ",");
                out.print(String.format("%.2f", pos1) + ",");
                out.print(String.format("%.2f", pos2) + "),(");
                out.print(String.format("%.2f", v0) + ",");
                out.print(String.format("%.2f", v1) + ",");
                out.print(String.format("%.2f", v2) + "),");
                out.print(String.format("%.2f", vc) + ",(");
                out.print(String.format("%.2f", acc0) + ",");
                out.print(String.format("%.2f", acc1) + ",");
                out.print(String.format("%.2f", acc2) + "),(");
                out.print(String.format("%.2f", gyro0)+ ",");
                out.print(String.format("%.2f", gyro1)+ ",");
                out.print(String.format("%.2f", gyro2)+ "),(");
                out.print(String.format("%.2f", rotation0)+ ",");
                out.print(String.format("%.2f", rotation1)+ ",");
                out.print(String.format("%.2f", rotation2)+ "),");
                out.print(String.format("%.2f", Gy) );
                //if(bf_optical_flow != optical_flow)
                    //out.print(String.format("%.2f", optical_flow));
                //else
                    //out.print("×");
                out.print("\r\n");
                //bf_optical_flow = optical_flow;

        }
    }

    public void print_flow(double op,long secondTime){
        out2.print(String.format("%.2f", op) + ",");
        out2.print(secondTime + "\r\n");
    }


    public void Printclose(long DelayTime1,long DelayTime2){
        out.flush();
        out2.flush();
        out.println("DelayTime1(通信往復にかかった時間)="+DelayTime1);
        out.println("DelayTime2(通信終了からタイマー開始までにかかった時間)"+DelayTime2);
        out2.println("DelayTime1(通信往復にかかった時間)="+DelayTime1);
        out2.println("DelayTime2(通信終了からタイマー開始までにかかった時間)"+DelayTime2);
        out.close();
        out2.close();
    }
    /*
    public void finalize(){
        out.close();
    }*/
}