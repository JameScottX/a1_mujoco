#include "mj_bridge.h"
#include <unistd.h>

char mjb_comd_title[1024];
char mjb_comd_info[1024];
bool mjb_ctl_flag = false;

std::string Leg_lf[JIT_NUM] = { "FL_hip_joint","FL_hip_joint","FL_hip_joint" };
std::string Leg_lb[JIT_NUM] = { "RL_hip_joint","RL_thigh_joint","RL_calf_joint" };
std::string Leg_rf[JIT_NUM] = { "FR_hip_joint","FR_hip_joint","FR_hip_joint" };
std::string Leg_rb[JIT_NUM] = { "RR_hip_joint","RR_thigh_joint","RR_calf_joint" };

std::string body_name = "trunk";

std::string Leg_lf_tc = "FL_foot";
std::string Leg_lb_tc = "RL_foot";
std::string Leg_rf_tc = "FR_foot";
std::string Leg_rb_tc = "RR_foot";

std::string Foot_Name[4] = {Leg_lf_tc, Leg_rf_tc, Leg_lb_tc, Leg_rb_tc};

long long time_once = 0;
static const int drive_sensor_p_ids[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, };
static const int drive_sensor_v_ids[] = {12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
static const int drive_ids[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

mjtByte geomgroup[5];
int geomid[5];
VideoRec video_rec;

// 关节角度和角速度
double lf_ang[3] = {0.0};
double lb_ang[3] = {0.0};
double rf_ang[3] = {0.0};
double rb_ang[3] = {0.0};
double lf_dang[3] = {0.0};
double lb_dang[3] = {0.0};
double rf_dang[3] = {0.0};
double rb_dang[3] = {0.0};

double toqsf[12] = {0.0};

// IMU
double quat[4] = {0.0};
double drpy[3] = {0.0}; 
double dxyz[3] = {0.0};
double acc[3] = {0.0};
double magn[3] = {0.0};
// 判断接触腿
double cfrc[24] = {0.0};


void sensor_get_from_mj(mjModel *m, mjData *d, double time_step_){
    
    for(short i = 0; i< 12; ++i){
        short idp = drive_sensor_p_ids[i];
        short idv = drive_sensor_v_ids[i];
        double ratio = m->actuator_gear[6 * m->sensor_objid[idp]];

        if ( i < 3){
            rf_ang[i] = d->sensordata[idp] / ratio;
            rf_dang[i] = d->sensordata[idv] / ratio;
        }else if(i>=3 && i<6){
            lf_ang[i-3] = d->sensordata[idp] / ratio;
            lf_dang[i-3] = d->sensordata[idv] / ratio;
        }else if(i>=6 && i<9){
            rb_ang[i-6] = d->sensordata[idp] / ratio;
            rb_dang[i-6] = d->sensordata[idv] / ratio;
        }else if(i>=9 && i<12){
            lb_ang[i-9] = d->sensordata[idp] / ratio;
            lb_dang[i-9] = d->sensordata[idv] / ratio;
        }
    }

    mju_copy(quat, &d->sensordata[12], 4);
    mju_copy(drpy, &d->sensordata[16], 3);
    mju_copy(dxyz, &d->sensordata[19], 3);
    mju_copy(acc, &d->sensordata[22], 3);
    mju_copy(magn, &d->sensordata[25], 3);
    
    // 判断接触腿
    double cfrc[24];
    foot_sim_force(m, d, cfrc);

    // SHOW_VECTOR("d->sensordata[drive_sensor_p_ids[0]]", d->sensordata[drive_sensor_p_ids[0]]);
    // SHOW_VECTOR("d->sensordata[drive_sensor_v_ids[0]]", d->sensordata[drive_sensor_v_ids[0]]);

    // SHOW_VECTOR("m->nsensor", m->nsensor);
    // SHOW_VECTOR("m->nuser_sensor", m->nuser_sensor);
    // SHOW_VECTOR("m->nu", m->nu);
    // SHOW_VECTOR("m->nq", m->nq);

    // SHOW_VECTOR("actuator_force", );
    // for(short i =0; i< m->nq; i++){
    //     std::cout << d->actuator_force[i] << " ";
    //     if ( i == m->nq -1 ) std::cout << std::endl;
    // }
    
    // SHOW_VECTOR("m->nv", m->nv);
    // SHOW_VECTOR("m->njnt", m->njnt);
    
    // for(short i =0; i< m->nv; i++){
    //     std::cout << m->dof_bodyid[i] << " ";
    //     if ( i == m->nv -1 ) std::cout<< std::endl;
    // }
    // for(short i =0; i< m->nv; i++){
    //     std::cout << m->dof_jntid[i] << " ";
    //     if ( i == m->nv -1 ) std::cout<< std::endl;
    // }

    // for(short i =0; i< m->nu; i++){
    //     std::cout << d->actuator_force[i] << " ";
    //     if ( i == m->nu -1 ) std::cout<< std::endl;
    // }
   
}

int func_call(void *arg, char *buf, int len){
    double *val = (double *)buf;
    return 0;
}

void mj_main_init(mjModel *m, mjData *d){

    // 初始化机器人位置姿态
    double body_init[7] = {0.0, 0.0, 0.5,  1.0, 0.0, 0.0, 0.0};
    mju_copy(&d->qpos[0], body_init, 7);
    // 初始化关节角度 
    double q_init[] = {0.,0.,0.,0.,0.,0.,0., 0.,0.,
                       0.,0.,0.,0.,0., 0.};
    mju_copy(&d->qpos[7], q_init, 12);

    SHOW_INFO("Init is ok!");
}


void mj_main_loop(mjModel *m, mjData *d, double time_step_){

    sensor_get_from_mj(m, d, time_step_);

//  do something here


//
    driver_sim_input(m,d);
}

void foot_sim_force(mjModel *m, mjData *d, double cfrc[24]){
    
}

void body_sim_hold(mjModel *m, mjData *d){
    // Set stiffness/damping for body translation joints
    for (short i = 0; i < 3; ++i) {
        m->jnt_stiffness[i] = 1e5;
        m->dof_damping[i] = 1e4;
        m->qpos_spring[i] = d->qpos[i];
    }
    // Set damping for body rotation joint
    for (short i = 3; i < 6; ++i)
        m->dof_damping[i] = 1e3;
}


void body_sim_release(mjModel *m, mjData *d){
    // Zero stiffness/damping for body translation joints
    for (short i = 0; i < 3; ++i) {
        m->jnt_stiffness[i] = 0;
        m->dof_damping[i] = 0;
    }
    // Zero damping for body rotation joint
    for (short i = 3; i < 6; ++i)
        m->dof_damping[i] = 0;
}


void driver_sim_input(const mjModel* m, mjData *d){

    for(short i = 0; i< 12; ++i){
        // 此处表示的是输出轴 转矩
        short id = drive_ids[i];
        double ratio = m->actuator_gear[6 * m->sensor_objid[id]];
        d->ctrl[id] = toqsf[i]/ratio;
    }
}


void mj_infotext(char* title, char* content){
}


void mj_init_recording(VideoRec *sim, const char* videofile, int width, int height, GLFWwindow* window){
    // 初始化仿真视频记录
    char ffmpeg_cmd[1000] = "ffmpeg -hide_banner -loglevel error -y -f rawvideo -vcodec rawvideo -pix_fmt rgb24 -s ";
    char integer_string[32];
    
    sprintf(integer_string, "%d", width); // Convert and write widthxheight
    strcat(ffmpeg_cmd, integer_string);
    strcat(ffmpeg_cmd, "x");
    sprintf(integer_string, "%d", height);
    strcat(ffmpeg_cmd, integer_string);

    strcat(ffmpeg_cmd, " -r "); //Frame Rate
    strcat(ffmpeg_cmd, VIDEO_FRAMERATE); //Frame Rate

    strcat(ffmpeg_cmd, " -i - -f mp4 -an -c:v libx264 -preset slow -crf 17 -vf \"vflip\" ");
    strcat(ffmpeg_cmd, videofile);
    if(strstr(videofile,".mp4") == NULL) // add a .mp4 if you forgot
        strcat(ffmpeg_cmd,".mp4");

    sim->video_width = width;
    sim->video_height = height;
    glfwSetWindowSize(window, width, height);
    sim->frame = (unsigned char*)malloc(3*width*height);

    sim->pipe_video_out = popen(ffmpeg_cmd, "w"); 

    sim->record_status = VideoRec::init;
}

void mj_record_frame(VideoRec *sim, GLFWwindow* window, mjrContext *con){
    // 仿真视频记录
    if(!sim || !window)
        return;

    mjrRect viewport = {0, 0, 0, 0};

    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // This checks if the window is resized and loops until it is released and corrected
    if(viewport.width != sim->video_width || viewport.height != sim->video_height){
        while(viewport.width != sim->video_width || viewport.height != sim->video_height){
            glfwSetWindowSize(window, sim->video_width, sim->video_height);
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            usleep(10000);
            if(!sim || !window)
                return;
        }
    }
    else{ //Normal case where the right size so it can render to file
        mjr_readPixels(sim->frame, NULL, viewport, con);
        //Write frame to output pipe
        fwrite(sim->frame, 1, sim->video_width*sim->video_height*3, sim->pipe_video_out);
    }

    sim->record_status = VideoRec::rec;
}

void mj_close_recording(VideoRec *sim){
    // 结束仿真视频记录
    if (sim->pipe_video_out){
        fflush(sim->pipe_video_out);
        pclose(sim->pipe_video_out);
        sim->pipe_video_out = NULL;
        free(sim->frame);

        sim->record_status = VideoRec::close;
    }
}

double peroid_control(bool mode, long long micro_second_peroid){
    // 周期控制器
    if (mode == 0 ){
        time_once = get_microseconds();
        return 0.0;
    }else if( mode == 1 ){
        // 此处等待一个固定的周期 
        while (get_microseconds() - time_once < micro_second_peroid) {}
        // SHOW_VECTOR("get_microseconds() - time_once",  get_microseconds() - time_once);
        double time_step_ = (get_microseconds() - time_once) / 1000000.0;
        time_once  = 0;
        return time_step_;
    }
}

long long get_microseconds(void){
    // 得到计算机当前时间 微秒单位
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000000 + now.tv_nsec / 1000;
}







