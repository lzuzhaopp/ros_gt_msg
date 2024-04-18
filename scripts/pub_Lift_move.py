import rospy
from ros_gt_msg.msg import Lift_control


def Lift_Control(mode:int, data:int, clear_flag:int):
    control = Lift_control()
    control.mode = mode
    control.data = data
    control.clear_flag = clear_flag
    return control

#发布函数
def dopub(pub_lift, control):
    '''
    #指令发布成功
    #指令发布失败
    #间隔1秒发布一条指令
    '''
    try:
        pub_lift.publish(control)
        rospy.loginfo("指令发布成功")
    except Exception as e:
        rospy.logerr(f"指令发布失败：{str(e)}")
    du_x = rospy.Duration(1)
    rospy.sleep(du_x)

#升降台运动函数
def move_lift(time, pub, control,mode):
    '''
    1、间隔一秒发布一次命令，定义time变量
    2、发布对象，后台以Lift_control格式发布运动消息，定义变量pub
    3、运动消息对象，格式为(mode,data,clear_flag),定义变量control
    4、区分位置模式或者速度模式，定义变量mode
    '''
    for _ in range(time):
        dopub(pub,control)
        if control.mode == 0x01:
            mode = "位置模式"
            rospy.loginfo(f"升降台控制模式为{mode},幅度是{control.data/10}mm")
        elif control.mode == 0x02:
            mode = "速度模式"
            rospy.loginfo(f"升降台控制模式为{mode},速度是{control.data}rpm")
        else:
            rospy.logerr(f"模式不正确，请输入正确模式")
        

if __name__ == "__main__":
    try:
        rospy.init_node("pub_lift_control")
        pub_lift = rospy.Publisher("/Lift_Control",Lift_control,queue_size=100)
        rospy.loginfo("创建发布节点成功")
        du_x = rospy.Duration(5)
        rospy.sleep(du_x)
    except Exception as e:
        rospy.logerr((f"初始化发布者或创建节点失败: {str(e)}"))

    #创建发布对象
    Lift_Control_Position = Lift_Control(0x01, 0x64, 0x00) # 用十进制
    Lift_Control_Speed = Lift_Control(0x02, 0x0F, 0x00)
    #Lift_Control_Clear = Lift_Control(0x01, 0x64, 0x01)

    #测试升降台位置模式，幅度为10mm
    move_lift(20, pub_lift, Lift_Control_Position, "位置模式")
    # move_lift(20, pub_lift, Lift_Control_Speed, "速度模式")

    