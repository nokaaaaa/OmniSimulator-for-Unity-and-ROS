# オムニホイールのPID制御
## 必要なソフトウェア
次の環境で動作確認ができました
- Ubuntu 22.04.4 LTS
- ROS2(humble)
- Unity(editor version:2022.3.49f1)
## 必要なセットアップ
### ROS TCPコネクター
次の記事を参考にしてください
https://note.com/oki0808/n/ne81bf0f37b78
### Unityのシーン作成
#### import
こちらにあるSimCore.unitypackageをUnity内にimportしてください
https://github.com/TakanoTaiga/template_ros2unity/releases/tag/UP-1.0
#### オブジェクトのセット
Hierarchy内に`SimCore/Demo`を追加し、+ボタンからCreateEmptyでPublisher,Subscriberという名前の空のオブジェクトと地面用のTerrainを作ってください(名前は何でも良い)
下のようになるはずです
![Screenshot from 2024-10-18 13-01-21](https://github.com/user-attachments/assets/fef01611-df82-422f-ac8b-84ff2b274a8f)
ここで上の写真のようにGlue-1のInspectorを開いてRigidbodyのFreezeRotationのx,zに対してチェックを入れます(こうしないと発進、停止時の挙動がすごくおかしくなる)
これをGlue-2,3,4にもやってあげます
#### スクリプトの生成
`SimCore/Scripts`フォルダー内のRobotというスクリプトを削除して、MyPublisherとMySubscriberという名前のスクリプトを作ってください(一番右は無視してください)
![Screenshot from 2024-10-18 13-06-49](https://github.com/user-attachments/assets/e3e0cbb6-8f31-429f-95a5-d22b2192c58d)
各スクリプトに次のコードをペーストしてください
 ```MyPublisher
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using Int32Msg = RosMessageTypes.Std.Int32Msg;
using Float32Msg = RosMessageTypes.Std.Float32Msg;
using TwistMsg = RosMessageTypes.Geometry.TwistMsg;
using PoseMsg = RosMessageTypes.Geometry.PoseMsg;

public class MyPublisher : MonoBehaviour
{
    ROSConnection ros;

    public GameObject targetObject;

    private Rigidbody rb;

    // Start is called before the first frame update
    void Start()
    {
        //ここで初期位置の座標を(0,0)としてもいいかもしれない
        // ROSコネクションの取得
        ros = ROSConnection.GetOrCreateInstance();

        GameObject glue1 = GameObject.Find("Glue-1");
        rb = glue1.GetComponent<Rigidbody>();

        // パブリッシャの登録
        ros.RegisterPublisher<TwistMsg>("cmd_vel");
        ros.RegisterPublisher<PoseMsg>("pose");
        
    }

    // Update is called once per frame
    void Update()
    {

        float xPos = targetObject.transform.position.x;
        float zPos = targetObject.transform.position.z;
        float yRotation = targetObject.transform.rotation.eulerAngles.y;

        float xVelocity = rb.velocity.x;
        float zVelocity = rb.velocity.z;
        float yAngularVelocity = rb.angularVelocity.y;

        //poseのパブリッシュ
        PoseMsg poseMsg = new PoseMsg();

        poseMsg.position.x = xPos; 
        poseMsg.position.y = 0.0f;
        poseMsg.position.z = zPos;

        poseMsg.orientation.x = 0.0f;
        poseMsg.orientation.y = yRotation;  
        poseMsg.orientation.z = 0.0f;
        poseMsg.orientation.w = 0.0f;
        
        ros.Publish("pose", poseMsg); 

        //cmd_velのパブリッシュ
        TwistMsg twistMsg = new TwistMsg();

        twistMsg.linear.x = xVelocity;  // Set linear velocity along x-axis
        twistMsg.linear.y = 0.0f;
        twistMsg.linear.z = zVelocity;

        twistMsg.angular.x = 0.0f;
        twistMsg.angular.y = yAngularVelocity;  // Set angular velocity around y-axis
        twistMsg.angular.z = 0.0f;
        
        ros.Publish("cmd_vel", twistMsg); 
}
}
 ```
 ```MySubscriber
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using MyTwistMsg = RosMessageTypes.Geometry.TwistMsg; 
using MyFloat32Msg = RosMessageTypes.Std.Float32Msg;

public class MySubscriber : MonoBehaviour
{
    public ArticulationBody moto_joint_1;
    public ArticulationBody moto_joint_2;
    public ArticulationBody moto_joint_3;
    public ArticulationBody moto_joint_4;

    private float vx = 0.0f; // Linear velocity in x-direction
    private float vz = 0.0f; // Linear velocity in z-direction
    private float ry = 0.0f; // Yaw angle (in radians)
    private float ry_velo = 0.0f; // Yaw angular velocity (in radians/sec)

    private float r = 0.05f; // Wheel radius
    private float R = 0.70710678f ; // Robot radius

    private float pointx=0.0f;
    private float pointz=0.0f;
    
    public float speed = 1.0f;

    // Start is called before the first frame update
    void Start()
    {
        ROSConnection.instance.Subscribe<MyTwistMsg>("/pid_cmd_vel", TwistCallback);
        ROSConnection.instance.Subscribe<MyFloat32Msg>("/point_x", PointXCallback);
        ROSConnection.instance.Subscribe<MyFloat32Msg>("/point_z", PointZCallback);
    }
    
    void PointXCallback(MyFloat32Msg msg)
    {
        pointx = (float)msg.data;
    }

    void PointZCallback(MyFloat32Msg msg)
    {
        pointz = (float)msg.data;
    }

    void TwistCallback(MyTwistMsg msg)
    {
        vx = (float)msg.linear.x; // Linear velocity in x-direction
        vz = (float)msg.linear.z; // Linear velocity in z-direction
        ry = (-1)*(float)msg.linear.y * Mathf.PI / 180; // Yaw angle in radians
        ry_velo = (-1)*(float)msg.angular.y * Mathf.PI / 180; // Yaw angular velocity in radians/sec

        float motor1_velo =(-1)* (-Mathf.Sin(ry + Mathf.PI / 4) * vx + Mathf.Cos(ry + Mathf.PI / 4) * vz +R * ry_velo) / r;
        float motor2_velo = (-1)* (-Mathf.Sin(ry + Mathf.PI / 4 + Mathf.PI / 2) * vx + Mathf.Cos(ry + Mathf.PI / 4 + Mathf.PI / 2) * vz + R * ry_velo) / r;
        float motor3_velo = (-1)* (-Mathf.Sin(ry + Mathf.PI / 4 + Mathf.PI) * vx + Mathf.Cos(ry + Mathf.PI / 4 + Mathf.PI) * vz + R * ry_velo) / r;
        float motor4_velo = (-1)* (-Mathf.Sin(ry + Mathf.PI / 4 + 3 * Mathf.PI / 2) * vx + Mathf.Cos(ry + Mathf.PI / 4 + 3 * Mathf.PI / 2) * vz + R * ry_velo) / r;

        // Apply velocities to the motors with slight random variations for realism
        moto_joint_1.SetDriveTargetVelocity(ArticulationDriveAxis.X, speed* motor1_velo * Random.Range(0.9f, 1.1f));
        moto_joint_2.SetDriveTargetVelocity(ArticulationDriveAxis.X, speed* motor2_velo * Random.Range(0.9f, 1.1f));
        moto_joint_3.SetDriveTargetVelocity(ArticulationDriveAxis.X, speed* motor3_velo * Random.Range(0.9f, 1.1f));
        moto_joint_4.SetDriveTargetVelocity(ArticulationDriveAxis.X, speed* motor4_velo * Random.Range(0.9f, 1.1f));

        // Debugging the received velocities
        Debug.Log("Linear Velocity: (" + vx + ", 0, " + vz + "), Angular Velocity: " + ry_velo);
    }

}
```
このスクリプトをそれぞれの空のオブジェクトにアタッチしてください Publisherのインスペクターを開いて、targetobjectでbodyを選択します(これもbodyが複数あるが、Demo内の一番上のディレクトリのbodyを選んでください) 
そして、Demo内のjointのTarget Velocityの値を0にしてください(4つすべて)
すると写真のようにになるのでmoto_joint1からjointを選んでください(ここで注意なのが全部同じ名前なのでmoto_joint<>の数字と、jointがの上のディレクトリにあるomni100mm_lite-<>の数字が一致するようにしてください3日溶けました)
![Screenshot from 2024-10-18 13-15-17](https://github.com/user-attachments/assets/da21b364-469f-4566-9c11-7f1e2526fa3d)
※追記 上の写真にあるスクリプトを有効にするかのチェックマークがあるのですが 最初からあるRobotというスクリプト(Demoのインスペクター内にあるやつ）のチェックマークは外してください
## シミュレーションの開始
ターミナル上で次の２つを実行(ビルドを行ってから実行してください)
```
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```
```
ros2 run pid_omni omni_pid_controller
```
Unityでシーンを起動します 成功したら次のようになるはずです
https://youtu.be/0jU1YdLFrzw?si=J0aoBV9WCb8mnFkG
