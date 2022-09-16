
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
        
        incomplete_task_array[i] = 0;
    }

    mis_save_param[0] = 0;
    mis_save_param[1] = 0;

    flag_incomplete_task_num = 0;
    incomplete_task_array_total = 0;

    msg_mission_back_info.header.seq = 0;

}

//-------------------------------------------------
// 回调函数
//-------------------------------------------------
void Task_part::mission_info_cb(const mavcomm_msgs::mission_info::ConstPtr &msg)
{
    msg_temp_mission_info = *msg;

    // 不区分 system_id  companion_id
    int t_no = msg_temp_mission_info.flag;

    // TODO 改成 枚举类型
    switch (t_no)
    {
        case 1: // 任务设置 初始化

            mission_settings_init(msg);
            break;

        case 2: // 任务设置完成进行 校验

            mission_setting_check();

            break;


    }

}

void Task_part::mission_set_cb(const mavcomm_msgs::mission_set::ConstPtr &msg)
{

}

// uav -> gcs   任务设置 回应 
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
// Func                     任务设置-mission_info_cb
//-------------------------------------------------

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
    flag_mission_set = 1;       // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误
    flag_mission_start = 0;     // 0-未开始 1-任务运行 2-暂停

    // mission_back_info
    // 告知 gcs 本机已经在 等待接收状态了
    msg_temp_mission_back_info.sysid = my_id;
    msg_temp_mission_back_info.compid = ID_GCS;

    msg_temp_mission_back_info.flag = 1;
    msg_temp_mission_back_info.mission_num = mis_total;
    msg_temp_mission_back_info.param1 = mis_save_param[0];
    msg_temp_mission_back_info.param2 = mis_save_param[1];

    mission_settings_back_info( msg_temp_mission_back_info );

}

// gcs -> uav   flag==2    |任务设置完成进行 校验  |需要回应 
// mavcomm_msgs::mission_info   .flag=2 
void Task_part::mission_setting_check()
{

}


//-------------------------------------------------
// Func                   任务设置 每一条-mission_set
//-------------------------------------------------

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

    // std::cont << "test" << std::endl;msg
    // ROS_INFO_STREAM( "only test i = " << i );
}
