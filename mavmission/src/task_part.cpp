
#include <task_part.h>

using namespace mav_mission;

//-------------------------------------------------
// Init 初始化
//-------------------------------------------------
Task_part::Task_part():
    tp_nh("~tp")        // /uav_mission/tp/test
{   
    // 数据 init
    task_init();

    // 话题订阅
    test_sub = tp_nh.subscribe("test", 1, &Task_part::test_cb, this);



}

// 数值 初始化
void Task_part::task_init()
{
    // TODO


}

//-------------------------------------------------
// 回调函数
//-------------------------------------------------
void Task_part::test_cb(const std_msgs::Int32::ConstPtr &msg)
{
		int t;

        t = msg->data;

        std::cout << "t=" << t << std::endl;
}


//-------------------------------------------------
// Public Func
//-------------------------------------------------
// 任务的初始化


void Task_part::mission_init(const mavcomm_msgs::mission_info::ConstPtr& msg)
{
    // msg
    mavcomm_msgs::mission_info test_msg = *msg;

    mis_total = (int) test_msg.mission_num;

    for (int i=0; i<mis_total; i++)
    {
        mis_array[i].flag_set = 0;
        mis_array[i].flag_this_uav = 0;
        mis_array[i].last_uav_mis_no = 0;
        mis_array[i].next_uav_mis_no = 0;
    }

    mis_array_current = 0;
    mis_save_param[0] = test_msg.param1;
    mis_save_param[1] = test_msg.param2;
    mis_save_param[2] = (test_msg.param1 + test_msg.param2) & 0xFF;

    // 标志位设置
    // flag_mission_set = 1;       // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误
    // flag_mission_start = 0;     // 0-未开始 1-任务运行 2-暂停

}




// 无人机 任务 设置
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



