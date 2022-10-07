
#include <task_part.h>
 
using namespace mav_mission;

//-------------------------------------------------
// Init 初始化
//-------------------------------------------------
Task_part::Task_part():
    tp_nh("~"),             // param    /uav_mission/xxx
    mavcomm_nh("mavcomm")   // mavcomm  /mavcomm/XXX   pub&sub
{   
    // 数值 初始化
    task_init();

    current_mission_state = MISSION_STATE_NAN;

    // load param
    tp_nh.param<int>("my_id", my_id, 100);
    // file_storage_path_head = XXX/src/cluster/mavcomm
    tp_nh.param<std::string>("file_storage_path_head", file_storage_path_head, "nan");
    if (file_storage_path_head != "nan")
    {
        std::string str_temp;
        str_temp = "/../mavmission/data/";
        file_storage_path_head = file_storage_path_head + str_temp;
    }

    std::cout << "task_part file_storage_path_head = " << file_storage_path_head << std::endl;
    std::cout << "task_part uav_mission/my_id = " << my_id << std::endl;

    // 话题订阅     | gcs -> uav
	mission_info_sub = mavcomm_nh.subscribe<mavcomm_msgs::mission_info>
        ("receive/mission_info", 10, &Task_part::mission_info_cb, this );
    
    // 话题订阅     | gcs -> uav
    // 接收 获取具体的每一条任务
    mission_set_sub = mavcomm_nh.subscribe<mavcomm_msgs::mission_set>
        ("receive/mission_set", 10, &Task_part::mission_set_cb, this );

    // 发送         | uav -> gcs
    // 任务设置反馈
    mission_back_info_pub = mavcomm_nh.advertise<mavcomm_msgs::mission_back_info>
        ("send/mission_back_info", 1);

    // Json Test
    /*
    mis_total = 2;
    // 读取测试
    file_storage_path = file_storage_path_head + std::string("test")
        + std::string(".json");
    strcpy(file_storage_path_cstr, file_storage_path.c_str());
    int back_i;
    back_i = nljson_file_load(file_storage_path_cstr, mis_total, 1, 1);
    // 保存测试
    file_storage_path = file_storage_path_head + std::string("test_save")
        + std::string(".json");
    strcpy(file_storage_path_cstr, file_storage_path.c_str());
    back_i = nljson_file_save(file_storage_path_cstr, mis_total);
    */
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
void Task_part::mission_info_cb
    (const mavcomm_msgs::mission_info::ConstPtr &msg)
{
    msg_temp_mission_info = *msg;

    // 不区分 system_id  companion_id
    int t_no = msg_temp_mission_info.flag;

    // enum MISSION_INFO
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
            // 校验完了-不反馈了

        break;

        case MISSION_INFO_LOAD: // 读取本地存档

            mission_setting_load(msg);

        break;

        case MISSION_INFO_SAVE: // 保存到本地存档

            mission_setting_save(msg);

        break;

        case MISSION_INFO_RUN: // 仅供测试用,正常直接调用

            // TODO

        break;

        default:
            // return;
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
void Task_part::mission_setting_check
    (const mavcomm_msgs::mission_info::ConstPtr& msg)
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


// gcs -> uav   |读取本地任务   |需要回应 
// mavcomm_msgs::mission_info   .flag=MISSION_INFO_LOAD 
void Task_part::mission_setting_load
    (const mavcomm_msgs::mission_info::ConstPtr& msg)
{
    mis_total = msg->mission_num;
    mis_save_param[0] = msg->param1; 
    mis_save_param[1] = msg->param2; 

    // mission 标志位设置
    current_mission_state = MISSION_STATE_LOADING;

    int back_i;
    back_i = nljson_file_load(file_storage_path_cstr, mis_total, mis_save_param[0], mis_save_param[1]);

    if ( back_i == 11)
    {   // 读取成功
        // mission 标志位设置
        current_mission_state = MISSION_STATE_LOADED;
        // mission_back_info
        // 告知 gcs
        mission_F_settings_back_info( current_mission_state );
    } else if ( back_i == 22)
    {   // 读取失败
        // mission 标志位设置
        current_mission_state = MISSION_STATE_LOAD_FAIL;
        // mission_back_info
        // 告知 gcs
        mission_F_settings_back_info( current_mission_state );

    }

}

// gcs -> uav   |保存到本地任务   |需要回应 
// mavcomm_msgs::mission_info   .flag=MISSION_INFO_SAVE 
void Task_part::mission_setting_save
    (const mavcomm_msgs::mission_info::ConstPtr& msg)
// 不改变当前状态   current_mission_state 不变
{
    // 通过校验后保存到本地
    if ( (current_mission_state==MISSION_STATE_CHECKED)
        || (current_mission_state==MISSION_STATE_SAVED )
        ) 
    {
        // 校验 长度
        if ( mis_total != msg->mission_num )
        {
            mission_F_settings_back_info( MISSION_STATE_SAVE_FAIL, 1);
            return;
        }

        // 校验 flag
        if ( (mis_save_param[0] == msg->param1) &&
             (mis_save_param[1] == msg->param2) )
        {
            mission_F_settings_back_info( MISSION_STATE_SAVE_FAIL, 2);
            return;
        }

        // 确认打开 Json 文件目录
        file_storage_path = file_storage_path_head + std::string("task_")
            + std::to_string(mis_total) + std::string("_")
            + std::to_string(mis_save_param[0]) + std::to_string(mis_save_param[1])
            + std::string(".json");
        strcpy(file_storage_path_cstr, file_storage_path.c_str());

        // json 保存
        int back_i;
        back_i = nljson_file_save(file_storage_path_cstr, mis_total);

        if ( back_i == 11)
        {   // 保存成功
            // mission_back_info
            // 告知 gcs
            mission_F_settings_back_info( MISSION_STATE_SAVED );
        } else if ( back_i == 22)
        {   // 保存失败
            // mission_back_info
            // 告知 gcs
            mission_F_settings_back_info( MISSION_STATE_SAVE_FAIL, 3);
        }
    } else
    {
        mission_F_settings_back_info( MISSION_STATE_SAVE_FAIL, 4);
    }
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
    
    //-------------------------------------- 
    // 保存 成功
    case MISSION_STATE_SAVED:   // 保存成功
        msg_mission_back_info.flag = MISSION_STATE_SAVED;
        msg_mission_back_info.mission_num = mis_total;
        msg_mission_back_info.param1 = mis_save_param[0];
        msg_mission_back_info.param2 = mis_save_param[1];
    break;

    //-------------------------------------- 
    // 保存 失败
    case MISSION_STATE_SAVE_FAIL:   // 保存失败
        msg_mission_back_info.flag = MISSION_STATE_SAVE_FAIL;
        msg_mission_back_info.mission_num = mis_total;
        msg_mission_back_info.param1 = param1;
        msg_mission_back_info.param2 = param1;
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
    // 1-在 设置模式 下，才能进行 任务设置
    if ( current_mission_state==MISSION_STATE_SETTING ) 
    {   
        task_mission_set(msg);
    } else if (   current_mission_state==MISSION_STATE_CHECKING
                ||current_mission_state==MISSION_STATE_CHECK_FAIL_INCOMPLETE )
    {   // 2-在 校验/校验不完全 的模式下，才能进行 任务设置
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

//-------------------------------------------------
// Func                          json & struct 转换
//-------------------------------------------------

/* detail mis_array[i].msg_mission_set

## msg_mission_set.header;
## msg_mission_set.compid;
## msg_mission_set.sysid;

msg_mission_set.mission_no;
msg_mission_set.flag;
msg_mission_set.mission_task;
msg_mission_set.uav_no;
msg_mission_set.x;
msg_mission_set.y;
msg_mission_set.z;
msg_mission_set.yaw;
msg_mission_set.param1;
msg_mission_set.param2;
msg_mission_set.param3;
*/
// json 转到 数组
void Task_part::from_json_to_mis_array()
{
    for (int i=0; i<test_json_data.size(); i++ )
    {
        test_json_data[i].at("flag").get_to( mis_array[i].msg_mission_set.flag );
        test_json_data[i].at("mission_no").get_to( mis_array[i].msg_mission_set.mission_no );
        test_json_data[i].at("mission_task").get_to( mis_array[i].msg_mission_set.mission_task );
        test_json_data[i].at("param1").get_to( mis_array[i].msg_mission_set.param1 );
        test_json_data[i].at("param2").get_to( mis_array[i].msg_mission_set.param2 );
        test_json_data[i].at("param3").get_to( mis_array[i].msg_mission_set.param3 );
        test_json_data[i].at("uav_no").get_to( mis_array[i].msg_mission_set.uav_no );
        test_json_data[i].at("x").get_to( mis_array[i].msg_mission_set.x );
        test_json_data[i].at("y").get_to( mis_array[i].msg_mission_set.y );
        test_json_data[i].at("yaw").get_to( mis_array[i].msg_mission_set.yaw );
        test_json_data[i].at("z").get_to( mis_array[i].msg_mission_set.z );

        mis_array[i].flag_set = 1;

        if ( mis_array[i].msg_mission_set.uav_no == my_id ) 
        {
            mis_array[i].flag_this_uav = 1;
        }
        else 
        {
            mis_array[i].flag_this_uav = 0;
        }

    }
}

// 数组 转 json
void Task_part::from_mis_array_to_json(int num)
{
    test_json_data.clear();

    for (int i=0; i<num; i++ )
    {
        temp_json_data = json{
            {"flag", mis_array[i].msg_mission_set.flag}, 
            {"mission_no", mis_array[i].msg_mission_set.mission_no}, 
            {"mission_task", mis_array[i].msg_mission_set.mission_task},
            {"param1", mis_array[i].msg_mission_set.param1}, 
            {"param2", mis_array[i].msg_mission_set.param2}, 
            {"param3", mis_array[i].msg_mission_set.param3},
            {"uav_no", mis_array[i].msg_mission_set.uav_no}, 
            {"x", mis_array[i].msg_mission_set.x}, 
            {"y", mis_array[i].msg_mission_set.y},
            {"z", mis_array[i].msg_mission_set.z}, 
            {"yaw", mis_array[i].msg_mission_set.yaw}
            };

        test_json_data.push_back(temp_json_data);

    }

}

//-------------------------------------------------
// Func                            nljson save&load
//-------------------------------------------------
// 从文档读取
int Task_part::nljson_file_load(char *path, int num, uint8_t param1, uint8_t param2)
{
    std::cout << "load mission data from json file: " << path << std::endl; 

    // 读取
    std::ifstream in(path); //打开文件，关联到流in

    if (in.is_open() )
    {   // 文件存在 打开
        in >> test_json_data;       //从流in中(也就是./person.json文件)读取内容到json对象中，会覆盖之前内容
        in.close();                 //关闭文件流in
        // 清空
        task_init();

        mis_total = num;
        mis_array_current = 0;

        mis_save_param[0] = param1;
        mis_save_param[1] = param2;

        // json 转到数组
        from_json_to_mis_array();

        /* test 
        std::cout << "0.param1=" << ( int (mis_array[0].msg_mission_set.param1)&0xff) << std::endl;
        std::cout << "1.param1=" << ( int (mis_array[1].msg_mission_set.param1)&0xff) << std::endl;

        std::cout << "0.x=" << ( (mis_array[0].msg_mission_set.x) ) << std::endl;
        std::cout << "1.x=" << ( (mis_array[1].msg_mission_set.x) ) << std::endl;

        std::cout << "j.size()=" << test_json_data.size() << std::endl;
        std::cout << std::setw(4) << test_json_data << std::endl;
        std::cout << "test_json_data[0]" << std::endl << test_json_data[0] << std::endl;
        */
        return 11;
    } else 
    { // 不存在 报错
        in.close(); 
        std::cout << "Error! json file not exist!" << std::endl; 
        return 22;
    } 
}

// 保存到文档
int Task_part::nljson_file_save(char *path, int num)
{
    std::cout << "save mission data to json file: " << path << std::endl; 
    
    // 保存
    std::ofstream of(path); //打开文件，关联到流of

    if ( of.is_open()==0 )
    {   
        // 打开失败
        of.close();
        std::cout << "Error! can not open json file !" << std::endl; 
        return 22;
    } else 
    {
        // 数组 转到 json
        from_mis_array_to_json(num);

        of << test_json_data.dump(4);
        of.close(); 
        return 11;
    }
}

// json 处理 END

