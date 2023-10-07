#include "robo.h"
#include <cmath>
#include "aid.h"

Robo::Robo(double real_time_, char jit_num_,std::string urdf_name_, 
            std::string robo_name, std::string body_name, 
            std::string *foot_name, std::string *leg_name1, 
            std::string *leg_name2, std::string *leg_name3, std::string *leg_name4, 
            std::string param_file_){

    this->real_time = real_time_;
    // 单个腿的维度 
    this->jit_num = jit_num_;
    this->param_file = param_file_;

    this->Leg_lf = new Leg(jit_num_, real_time_);
    this->Leg_rf = new Leg(jit_num_, real_time_);
    this->Leg_lb = new Leg(jit_num_, real_time_);
    this->Leg_rb = new Leg(jit_num_, real_time_);
    this->Leg_lf->init(leg_name1);
    this->Leg_rf->init(leg_name2);
    this->Leg_lb->init(leg_name3);
    this->Leg_rb->init(leg_name4);

    att = new Attitude();
    att_last = new Attitude();
    datt = new Attitude();

    gps_pos_now = new Pos();           //最后矫正坐标
    gps_pos = new Pos();               //此处 gps 坐标
    gps_pos_last = new Pos();
    gps_pos_bias = new Pos();
    gps_speed = new Pos();
    gps_acce = new Pos();

    cog_bias = new Pos();
    cog_bias_com = new Pos();

    // 机器人模型的预先加载 
    this->urdf_name = urdf_name_;
    pinocchio::urdf::buildModel(urdf_name, JointModelFreeFlyer(), this->model);
    this->data = Data(this->model);
    
    // 确定机器人的名称 关键部位
    this->rbn.roboname = robo_name;
    this->rbn.bodyname = body_name;
    this->rbn.body_id = (unsigned short)this->model.getFrameId(body_name);
    for(char i =0;i<ROBO_LEG_NUM;i++){
        this->rbn.footname.push_back(foot_name[i]);
        this->rbn.foot_id.push_back((unsigned short)model.getFrameId(foot_name[i]));
        // printf("aa %d \n", this->rbn.foot_id[i]);
    }

    this->robo_name = robo_name;

    // 确定机器人维度 按照 pinocchio 标准 
    this->nq = this->model.nq;
    this->nv = this->model.nv;
    
    //**************Robot attributes**************
    Eigen::Vector3d temp_loc1 = this->init_pos_get_com(leg_name1[0], true );
    Eigen::Vector3d temp_loc2 = this->init_pos_get_com(leg_name1[1], false);
    Eigen::Vector3d temp_loc3 = this->init_pos_get_com(leg_name1[2], false);
    Eigen::Vector3d temp_loc4 = this->init_pos_get_com(foot_name[0], false);

    this->Swing_Lenth = 0.0;
    this->Thign_Lenth = ( temp_loc3 - temp_loc2 ).norm();
    this->Calf_Lenth = ( temp_loc4 - temp_loc3 ).norm();
    this->Body_Height = fabs(temp_loc1[2]);
    //腿一般伸长
    this->leg_normal_len = this->Thign_Lenth / sqrt(2) * 2;
    //#直线一般保持的坐标
    this->body_normal_z = this->leg_normal_len + this->Body_Height;
    this->body_mass = 16.0 ;

    // printf("this->body_normal_z %f \n", this->body_normal_z);
    // printf("this->Thign_Lenth %f \n", this->Thign_Lenth);

    //这里是每个hip的坐标
    leg_loc_real = new Eigen::Vector3d[4];
    leg_loc_com = new Eigen::Vector3d[4];

    leg_loc_real[0] = this->init_pos_get_com(leg_name1[0], true);
    leg_loc_real[1] = this->init_pos_get_com(leg_name2[0], false);
    leg_loc_real[2] = this->init_pos_get_com(leg_name3[0], false);
    leg_loc_real[3] = this->init_pos_get_com(leg_name4[0], false);
    // 清除 Body_Height
    // this->leg_loc_real[0](2) = 0.0;  
    // this->leg_loc_real[1](2) = 0.0;  
    // this->leg_loc_real[2](2) = 0.0;  
    // this->leg_loc_real[3](2) = 0.0;  

    // 初始化机器人常规参数
    double temp_flag = AID::paramsFromFile(this->param_file, "CoG_indentification");
    if (temp_flag > 0.5){
        Eigen::Vector3d temp_val = AID::paramsFromFile(this->param_file, "__cog_bias", 3);
        this->cog_bias->assigE(temp_val);
    }

    // 环境感知传感器参数
    this->lidar_val_f.clear();
    this->lidar_val_h.clear();
    

};

Robo::~Robo(){
    delete Leg_lf, Leg_rf, Leg_lb, Leg_rb;
    delete att, att_last, datt;
    delete gps_pos_now, gps_pos, gps_pos_last, gps_pos_bias,
     gps_speed, gps_acce, cog_bias, cog_bias_com;
    delete [] leg_loc_real, leg_loc_com;
};

void Robo::refresh(double time_step_){
    // 此函数刷新循环时间 
    this->Leg_lf->refresh(time_step_);
    this->Leg_rf->refresh(time_step_);
    this->Leg_lb->refresh(time_step_);
    this->Leg_rb->refresh(time_step_);
    // 此处刷新运行时间
    this->real_time = time_step_;

    // 角速度 
    *datt = *att - *att_last;
    datt->div(this->real_time);
    att_last->assig(att->draw());
    // 位移速度 
    *gps_speed = *gps_pos - *gps_pos_last;
    gps_speed->div(this->real_time);
    gps_pos_last->assig(gps_pos->draw());

    // 初始GPS 进行矫正 
    // *gps_pos_now =  *gps_pos - *gps_pos_bias;

    double temp[3] = {0., 0., 0.};
    gps_pos_now->assig(temp);
    
    // 此处已经在 dog_c 中回调函数刷新 
    // this->touchstate[0] = this->Leg_lf->touchstate;
    // this->touchstate[1] = this->Leg_rf->touchstate;
    // this->touchstate[2] = this->Leg_lb->touchstate;
    // this->touchstate[3] = this->Leg_rb->touchstate;

}

void Robo::stand_up(){
    double *temp = gps_pos->draw();
    temp[2] -= this->body_normal_z;
    gps_pos_bias->assig(temp);
}

void Robo::angle_set(Eigen::VectorXd angs){

    this -> controlMode = ANG_CONTROL;
    double temp[this->jit_num];
    Eigen::Map<Eigen::VectorXd>(temp, 3, 1) = angs.block(0,0,3,1);
    this->Leg_lf->angf->assig(temp);
    Eigen::Map<Eigen::VectorXd>(temp, 3, 1) = angs.block(3,0,3,1);
    this->Leg_rf->angf->assig(temp);
    Eigen::Map<Eigen::VectorXd>(temp, 3, 1) = angs.block(6,0,3,1);
    this->Leg_lb->angf->assig(temp);
    Eigen::Map<Eigen::VectorXd>(temp, 3, 1) = angs.block(9,0,3,1);
    this->Leg_rb->angf->assig(temp);
    
}


void Robo::torque_set(Eigen::VectorXd toqs){

    this->controlMode = TOQ_CONTROL;
    double temp[this->jit_num];
    Eigen::Map<Eigen::VectorXd>(temp, 3, 1) = toqs.block(0,0,3,1);
    this->Leg_lf->toqf->assig(temp);
    Eigen::Map<Eigen::VectorXd>(temp, 3, 1) = toqs.block(3,0,3,1);
    this->Leg_rf->toqf->assig(temp);
    Eigen::Map<Eigen::VectorXd>(temp, 3, 1) = toqs.block(6,0,3,1);
    this->Leg_lb->toqf->assig(temp);
    Eigen::Map<Eigen::VectorXd>(temp, 3, 1) = toqs.block(9,0,3,1);
    this->Leg_rb->toqf->assig(temp);

}


Eigen::Vector3d Robo::init_pos_get_com(std::string name, bool cal = false){

    if (cal == true){
        Eigen::VectorXd __q =  Eigen::VectorXd::Zero(this->model.nq);
        __q(6) = 1.;
        computeJointJacobians(this->model, this->data, __q);
        framesForwardKinematics(this->model, this->data, __q);
    }

    FrameIndex id_tar = this->model.getFrameId(name);
    updateFramePlacement(this->model, this->data, id_tar);
    updateFramePlacement(this->model, this->data, this->rbn.body_id);
    
    Eigen::Vector3d temp = data.oMf[id_tar].translation() - data.oMf[this->rbn.body_id].translation();

    return temp;

}



