import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        """
        PID制御器の初期化
        :param kp: 比例ゲイン
        :param ki: 積分ゲイン
        :param kd: 微分ゲイン
        :param setpoint: 目標値
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_error = 0   # 前回のエラー
        self.integral = 0     # 積分項

    def update(self, error, dt):
        """
        PID制御の更新
        :param current_value: 現在の値（制御対象のフィードバック値）
        :param dt: 前回からの時間差分
        :return: 制御量
        """
        
        # 比例項
        proportional = self.kp * error
        
        # 積分項
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # 微分項
        derivative = self.kd * (error - self.prev_error) / dt
        
        # PIDの合計
        output = proportional + integral + derivative
        
        # エラーを次のステップに保存
        self.prev_error = error
        
        return output


class OmniPIDController(Node):

    def __init__(self):
        super().__init__('omni_pid_controller')

        #設定
        self.point_x = 5#目標座標
        self.point_z = 6  #目標座標
        self.point_theta = 90.0 #目標角度(度数法)
        self.limit_speed =100000.0 #制限速度
        self.limit_accel =100000.0 #制限加速度
        self.limit_rotate_speed =100000.0 #制限回転速度
        self.limit_rotate_accel =100000.0 #制限回転加速度
        self.goal_range = 0.1 #どこまで近づいたら目標地点についたとみなすか
        self.goal_rotate=1 #どこまで角度が近づいたら目標角度についたとみなすか
        self.period = 0.01  #0.01秒ごとに速度をpublish

        # オブジェクトの位置,速度をsubscribe
        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.twist_sub = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)

        # 目標位置のパブリッシュ設定 (point_x, point_z)
        self.pointx = self.create_publisher(Float32, 'point_x', 10)
        self.pointz = self.create_publisher(Float32, 'point_z', 10)
        msg_x = Float32()
        msg_z = Float32()

        msg_x.data = float(self.point_x)
        msg_z.data = float(self.point_z)

        self.pointx.publish(msg_x)
        self.pointz.publish(msg_z)

        
       # vx,vz,回転速度をPublish
        self.publisher_ = self.create_publisher(Twist, 'pid_cmd_vel', 10)


    

        #pidの初期化(x,z平面)
        self.pid = PIDController(kp=10.0, ki=0.1, kd=0.05) 

        # 回転制御
        self.pid_rotate = PIDController(kp=7.0, ki=0.1, kd=0.05)
        # 制御ループ用のタイマー
        self.timer = self.create_timer(self.period, self.timer_callback)

        # 現在の座標
        self.current_pose_x = 0.0
        self.current_pose_z = 0.0
        self.current_ori_y = 0.0

        # 現在の速度
        self.current_li_x = 0.0
        self.current_li_z = 0.0
        self.current_ang_y = 0.0
        
        #publishする速度
        self.vel_x = 0.0
        self.vel_z = 0.0
        self.ang_y = 0.0

        #加速度計算用の前回の速度
        self.prev_vel_value = 0.0 #x,yの速度ベクトルの大きさ
        self.prev_ang_y = 0.0

        #10Hzでpid_cmd_velをpublish
        self.timer = self.create_timer(self.period, self.timer_callback)

    def pose_callback(self, msg):
        position = msg.position
        orientation = msg.orientation

        self.current_pose_x = position.x
        self.current_pose_z = position.z
        #yaw角を前方を0度として-180~180度になるように変換
        self.current_ori_y = self.mod(orientation.y)

    def twist_callback(self, msg):
        linear = msg.linear
        angular = msg.angular

        self.current_li_x = linear.x
        self.current_li_z = linear.z
        self.current_ang_y = angular.y
    #-180から180にする
    def mod(self,angle):
    # 360で割ったあまりを計算
        normalized_angle = angle % 360
        # 180より大きい場合は360を引く
        if normalized_angle > 180:
           normalized_angle -= 360
      # -180より小さい場合は360を足す
        elif normalized_angle < -180:
           normalized_angle += 360
        return normalized_angle

    def timer_callback(self):
       
       error=math.sqrt((self.point_x-self.current_pose_x)**2+(self.point_z-self.current_pose_z)**2) #目標値との距離
       rotate_error = self.point_theta - self.current_ori_y #目標角度との差
       print(rotate_error)
       #v_x,v_zを計算
       if error > self.goal_range:#閾値内に来たらvx,vyを0にする
           velo=self.pid.update(error, self.period) #目標値への速度ベクトルの大きさ
        
           #出そうとする速度 加速度制限を考慮　制限速度　この3つから値を選ぶ
           if velo > self.prev_vel_value :
               velo=min(velo , self.prev_vel_value + self.limit_accel*self.period , self.limit_speed)

           else:
               velo=max(velo , self.prev_vel_value - self.limit_accel*self.period , -self.limit_speed)
        
           self.vel_x = velo*(self.point_x-self.current_pose_x)/error #目標値への速度ベクトルのx成分
           self.vel_z = velo*(self.point_z-self.current_pose_z)/error #目標値への速度ベクトルのz成分

           #加速度計算用の前回の速度
           self.prev_vel_value =  math.sqrt(self.current_li_x **2 + self.current_li_z **2)
       else :
           self.vel_x = 0.0
           self.vel_z = 0.0
       #角速度を計算
       if abs(rotate_error) > self.goal_rotate:#閾値ないに来たら回転を0にする
           self.ang_y = self.pid_rotate.update(rotate_error, self.period)

            #出そうとする角速度 回転加速度制限を考慮　制限回転速度　この3つから値を選ぶ
           if self.ang_y > self.prev_ang_y :
               self.ang_y = min(self.ang_y , self.prev_ang_y + self.limit_rotate_accel*self.period , self.limit_rotate_speed)
           else:
               self.ang_y = max(self.ang_y , self.prev_ang_y - self.limit_rotate_accel*self.period , -self.limit_rotate_speed)
           
           #加速度制限用の前回の角速度
           self.prev_ang_y = self.current_ang_y
       else:
            self.ang_y = 0.0

       # Twistメッセージの作成
       twist = Twist()
       twist.linear.x = float(self.vel_x)  
       twist.linear.z = float(self.vel_z)
       twist.angular.y =float(self.ang_y)  # 回転速度
       twist.linear.y = float(self.current_ori_y) #y方向の速度にyaw角を入れてる(あんまりよくないね)
  
        # メッセージのパブリッシュ
       self.publisher_.publish(twist)
        
def main(args=None):
    rclpy.init(args=args)
    node = OmniPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
