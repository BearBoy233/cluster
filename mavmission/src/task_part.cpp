
#include <task_part.h>

using namespace mav_mission;

//-------------------------------------------------
// Init 初始化
//-------------------------------------------------
Task_part::Task_part():
    tp_nh("~tp")        // /uav_mission/tp/xxx
{   
    // 数值 初始化
    task_init();

    current_mission_state = MISSION_STATE_NAN;

    // load param
    // TBC
    tp_nh.param<int>("my_id", my_id, 100);
	std::cout << "tp/uav_mission/my_id = " << my_id << std::endl;

    // 话题订阅     | gcs -> uav
	mission_info_sub = tp_nh.subscribe<mavcomm_msgs::mission_info>
        ("/mavcomm/receive/mission_info", 10, &Task_part::mission_info_cb, this );
    
    // 话题订阅     | gcs -> uav
    // 接收 获取具体的每一条任务
    mission_set_sub = tp_nh.subscribe<mavcomm_msgs::mission_set>
        ("/mavcomm/receive/mission_set", 10, &Task_part::mission_set_cb, this );

    // 发送         | uav -> gcs
    // 任务设置反馈
    mission_back_info_pub = tp_nh.advertise<mavcomm_msgs::mission_back_info>
        ("/mavcomm/send/mission_back_info", 1);

}

// 数值 初始化
void Task_part::task_init()
{
    // TBC
    mis_total = 0;
    mis_array_current = 0;

    for (int i=0; i<MAX_NUM_MIS; i++)
    {
        mis_array[i].flag_set = 0;
        mis_array[i].flag_this_uav = 0;
        mis_array[i].last_uav_mis_no = 0;
        mis_array[i].next_uav_mis_no = 0;
    }

    mis_save_param[0] = 0;
    mis_save_param[1] = 0;

    msg_mission_back_info.header.seq = 0;

}

//-------------------------------------------------
// Func                     任务设置-mission_info_cb
//-------------------------------------------------
void Task_part::mission_info_cb(const mavcomm_msgs::mission_info::ConstPtr &msg)
{
    msg_temp_mission_info = *msg;

    // 不区分 system_id  companion_id
    int t_no = msg_temp_mission_info.flag;

    // TODO 改成 枚举类型
    switch (t_no)
    {
        case MISSION_INFO_SET_INIT: // 任务设置 初始化

            mission_settings_init(msg);

        break;

        case MISSION_INFO_CHECK: // 任务设置完成进行 校验

            if (current_mission_state != MISSION_STATE_CHECKED)
            {
                current_mission_state = MISSION_STATE_CHECKING;
                mission_setting_check(msg);
            }

        break;

        case MISSION_INFO_LOAD: // 读取

            // mission_setting_check();

        break;

        case MISSION_INFO_SAVE: // 保存

            // mission_setting_check();

        break;

        case MISSION_INFO_DEL: // 删除

            // mission_setting_check();

        break;
    }

}

// gcs -> uav   flag==1    |任务初始化   |需要回应 
// mavcomm_msgs::mission_info   .flag=1 
void Task_part::mission_settings_init(const mavcomm_msgs::mission_info::ConstPtr& msg)
{
    msg_temp_mission_info = *msg;
 
    // 数值初始化
    task_init();

    mis_total = (int) msg_temp_mission_info.mission_num;
    mis_array_current = 0;

    mis_save_param[0] = msg_temp_mission_info.param1;
    mis_save_param[1] = msg_temp_mission_info.param2;

    // mission 标志位设置
    current_mission_state = MISSION_STATE_SETTING;

    // mission_back_info
    // 告知 gcs 本机已经在 等待接收状态了

    mission_F_settings_back_info( current_mission_state );
}

// gcs -> uav   flag==2    |任务设置完成进行 校验  |需要回应 
// mavcomm_msgs::mission_info   .flag=2 
void Task_part::mission_setting_check(const mavcomm_msgs::mission_info::ConstPtr& msg)
{
    // 校验 长度
    if ( mis_total != msg->mission_num )
    {
        // mission 标志位设置
        current_mission_state = MISSION_STATE_CHECK_FAIL_LEN;
        // mission_back_info
        // 告知 gcs
        mission_F_settings_back_info( current_mission_state );
        return;
    }

    // 校验 flag
    if ( (mis_save_param[0] == msg->param1) &&
         (mis_save_param[1] == msg->param2) )
    {
        // mission 标志位设置
        current_mission_state = MISSION_STATE_CHECK_FAIL_CRC;
        // mission_back_info
        // 告知 gcs
        mission_F_settings_back_info( current_mission_state );
        return;
    }

    // 校验 数组是否完整
    for (int i=0; i<mis_total; i++)
    {
        if ( mis_array[i].flag_set == 0 )
        {
            // mission 标志位设置
            current_mission_state = MISSION_STATE_CHECK_FAIL_INCOMPLETE;
            // mission_back_info
            // 告知 gcs
            mission_F_settings_back_info( current_mission_state, i);
            return;
        }
    }

    // 确认本机的 上一个&下一个 任务编号
    int temp_last = -1;
    // mis_array 参数 赋值
    for (int i=0; i<mis_total; i++)
    {
        mis_array[i].last_uav_mis_no = temp_last;

        if ( mis_array[i].flag_this_uav == true)
        {
            temp_last = i;
        }
    }

    int temp_next = -1;
    for (int i=mis_total-1; i>-1; i--)
    {
        mis_array[i].next_uav_mis_no = temp_next;

        if ( mis_array[i].flag_this_uav == true)
        {
            temp_next = i;
        }
    }

    // 校验无误
    // mission 标志位设置
    current_mission_state = MISSION_STATE_CHECKED;
    // mission_back_info
    // 告知 gcs
    mission_F_settings_back_info( current_mission_state );
}

//-------------------------------------------------
// Func                  任务设置反馈-mission_back_info
//-------------------------------------------------

// uav -> gcs   任务设置 回应  [由 flag 决定返回信息]
void Task_part::mission_F_settings_back_info(int flag_state, int param1)
{
    switch ( flag_state )
    {
    
    //--------------------------------------
    // 设置模式
    case MISSION_STATE_SETTING:   // 设置模式
        msg_mission_back_info.flag = MISSION_STATE_SETTING;
        msg_mission_back_info.mission_num = mis_total;
        msg_mission_back_info.param1 = mis_save_param[0];
        msg_mission_back_info.param2 = mis_save_param[1];
    break;

    //--------------------------------------
    // 校验 失败
    case MISSION_STATE_CHECK_FAIL_LEN:    // 校验 LEN 失败
    case MISSION_STATE_CHECK_FAIL_CRC:    // 校验 CRC 失败
        msg_mission_back_info.flag = flag_state;
        msg_mission_back_info.mission_num = mis_total;
        msg_mission_back_info.param1 = mis_save_param[0];
        msg_mission_back_info.param2 = mis_save_param[1];
    break;

    case MISSION_STATE_CHECK_FAIL_INCOMPLETE:    // 校验 不完整
        msg_mission_back_info.flag = MISSION_STATE_CHECK_FAIL_INCOMPLETE;
        msg_mission_back_info.mission_num = mis_total;
        msg_mission_back_info.param1 = param1;
        msg_mission_back_info.param2 = param1;
    break;

    //-------------------------------------- 
    // 校验 完成
    case MISSION_STATE_CHECKED:   // 设置模式
        msg_mission_back_info.flag = MISSION_STATE_CHECKED;
        msg_mission_back_info.mission_num = mis_total;
        msg_mission_back_info.param1 = mis_save_param[0];
        msg_mission_back_info.param2 = mis_save_param[1];
    break;
    

    default:
        return;
        break;
    }

    // common ID
    msg_mission_back_info.sysid = my_id;
    msg_mission_back_info.compid = ID_GCS;
    // header
    msg_mission_back_info.header.seq++;
    msg_mission_back_info.header.stamp = ros::Time::now(); 
    // publish
    mission_back_info_pub.publish(msg_mission_back_info);
}


// uav -> gcs   任务设置 回应  [常规-msg]
void Task_part::mission_settings_back_info(mavcomm_msgs::mission_back_info msg)
{
    // 回应信号
    msg_mission_back_info.flag = msg.flag;
    msg_mission_back_info.mission_num = msg.mission_num;
    msg_mission_back_info.param1 = msg.param1;
    msg_mission_back_info.param2 = msg.param2;
    // 发送&接收 ID 
    msg_mission_back_info.sysid = msg.sysid;
    msg_mission_back_info.compid = msg.compid;

    // 
    msg_mission_back_info.header.seq++;
    msg_mission_back_info.header.stamp = ros::Time::now(); 
    mission_back_info_pub.publish(msg_mission_back_info);
}

//-------------------------------------------------
// Func                   任务设置 每一条-mission_set
//-------------------------------------------------

void Task_part::mission_set_cb(const mavcomm_msgs::mission_set::ConstPtr &msg)
{
    // 在 设置模式 下，才能进行 任务设置
    if ( current_mission_state==MISSION_STATE_SETTING ) 
    {   
        task_mission_set(msg);
    } else if (   current_mission_state==MISSION_STATE_CHECKING
                ||current_mission_state==MISSION_STATE_CHECK_FAIL_INCOMPLETE )
    {   // 在 校验/校验不完全 的模式下，才能进行 任务设置
        task_mission_set(msg);
    }
}

// gcs -> uav   任务设置 每一条
void Task_part::task_mission_set(const mavcomm_msgs::mission_set::ConstPtr& msg)
{
    int i = (int) msg->mission_no;

    mis_array[i].msg_mission_set = *msg;
    mis_array[i].flag_set = 1;

    if (msg->uav_no == my_id) 
    {
        mis_array[i].flag_this_uav = 1;
    }
    else 
    {
        mis_array[i].flag_this_uav = 0;
    }

    // std::cont << "test" << std::endl;msg
    // ROS_INFO_STREAM( "only test i = " << i );
}
