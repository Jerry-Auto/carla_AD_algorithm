#include "cilqr_planner/cost_define.h"
#include <iostream>
namespace AD_algorithm {
namespace planner {

planner_problem::planner_problem(const std::shared_ptr<CILQRPlannerparams>& params) : planner_params_(params) {
    n_x_ = planner_params_->ilqr_params->nx;
    n_u_ = planner_params_->ilqr_params->nu;
    lower_bound_ = Eigen::VectorXd::Zero(num_c_ / 2);
    upper_bound_ = Eigen::VectorXd::Zero(num_c_ / 2);
    lower_bound_[0] = planner_params_->road_lower_bound; // road lower bound, will be set later
    upper_bound_[0] = planner_params_->road_upper_bound;  // road upper bound, will be set later
    lower_bound_[1] = planner_params_->min_a;
    upper_bound_[1] = planner_params_->max_a;
    lower_bound_[2] = planner_params_->min_v;
    upper_bound_[2] = planner_params_->max_v;
    lower_bound_[3] = planner_params_->min_steer;
    upper_bound_[3] = planner_params_->max_steer;
    alm_rho_ = planner_params_->alm_rho_init;
    alm_rho_next_ = alm_rho_;;
    barrier_rho_ = planner_params_->barrier_rho_init;
    barrier_rho_next_ = barrier_rho_;
    Q_=Eigen::MatrixXd::Identity(planner_params_->ilqr_params->nx, planner_params_->ilqr_params->nx);
    R_=Eigen::MatrixXd::Identity(planner_params_->ilqr_params->nu, planner_params_->ilqr_params->nu);
    Qf_=Eigen::MatrixXd::Identity(planner_params_->ilqr_params->nx, planner_params_->ilqr_params->nx);
    alm_mu_must_ = Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, num_c_);
    alm_mu_must_next_ = Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, num_c_);
    build_functions();
}

void planner_problem::set_frenet_frame(std::shared_ptr<AD_algorithm::general::FrenetFrame> frenet_frame){
    global_frenet_frame_=frenet_frame;
}

void planner_problem::set_obstacles(const std::vector<AD_algorithm::general::Obstacle>& obstacles){
    obstacles_=obstacles;
    num_obstacles_=obstacles_.size();
    alm_mu_obstacles_ = Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, num_obstacles_ * 2); // 每个障碍物2个不等式约束
    alm_mu_obstacles_next_ = Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, num_obstacles_ * 2);
}

void planner_problem::set_reference_speed(double v_ref){
    reference_speed_=v_ref;
}

void planner_problem::set_cost_weights(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Qf){
    Q_=Q;
    R_=R;
    Qf_=Qf;
}

void planner_problem::set_road_bounds(const double& lower_bound, const double& upper_bound){
    planner_params_->road_lower_bound = lower_bound;
    planner_params_->road_upper_bound = upper_bound;
    lower_bound_[0] = lower_bound;
    upper_bound_[0] = upper_bound;
}


    Eigen::VectorXd planner_problem::state_transition(const Eigen::VectorXd& state_and_control){
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << state_and_control.transpose();
        casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
        auto next_state = f_(std::vector<casadi::DM>{z_row_dm});
        casadi::DM next_state_dm = next_state.at(0);
        Eigen::Map<Eigen::VectorXd> next_state_map(next_state_dm.ptr(), n_x_ );
        return next_state_map;
    }

    // 计算状态转移函数的雅可比矩阵，输入是z=[x;u]合并后的向量,输出是雅可比矩阵堆成的张量
    Eigen::Tensor<double, 3,Eigen::RowMajor> planner_problem::state_total_jacobian(const Eigen::MatrixXd& x_and_u){
        int N = x_and_u.rows();
        int nz = n_x_ + n_u_;
        Eigen::Tensor<double, 3,Eigen::RowMajor> jacobian_tensor(N, n_x_, nz);
        for(int i=0;i<N;++i){
            Eigen::RowVectorXd z_row(nz);
            z_row << x_and_u.row(i);
            casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
            auto jacobian_res = f_jacobian_(std::vector<casadi::DM>{z_row_dm});
            casadi::DM jacobian_dm = jacobian_res[0];
            Eigen::Map<Eigen::MatrixXd> jacobian_mat(jacobian_dm.ptr(), n_x_, nz);
            for(int r=0;r<n_x_;++r){
                for(int c=0;c<nz;++c){
                    jacobian_tensor(i,r,c)=jacobian_mat(r,c);
                }
            }
        }
        return jacobian_tensor;
    }

    // 计算状态转移函数的Hessian张量，输入是z=[x;u]合并后的向量,输出是Hessian堆成的四维张量
    Eigen::Tensor<double, 4,Eigen::RowMajor> planner_problem::state_total_hessian(const Eigen::MatrixXd& x_and_u){
        int N = x_and_u.rows();
        int nz = n_x_ + n_u_;
        Eigen::Tensor<double, 4,Eigen::RowMajor> hessian_tensor(N, n_x_, nz, nz);
        for(int i=0;i<N;++i){
            Eigen::RowVectorXd z_row(nz);
            z_row << x_and_u.row(i);
            casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
            auto hessian_res = f_hessian_(std::vector<casadi::DM>{z_row_dm});
            for(int r=0;r<n_x_;++r){
                casadi::DM h_r = hessian_res[r];
                for(int c1=0;c1<nz;++c1){
                    for(int c2=0;c2<nz;++c2){
                        hessian_tensor(i,r,c1,c2) = (double)h_r(c1,c2);
                    }
                }
            }
        }
        return hessian_tensor;
    }

    void planner_problem::build_dynamics_function(){
        casadi::SX z = casadi::SX::sym("z", n_x_+n_u_);
        casadi::SX state= z(casadi::Slice(0, (int)n_x_));
        casadi::SX control= z(casadi::Slice((int)n_x_, (int)(n_x_+n_u_)));
        // 使用运动学模型，控制量：[steering_angle，a],状态量：[x, y,heading,v] 
        casadi::SX x = state(0);
        casadi::SX y = state(1);
        casadi::SX heading = state(2);
        casadi::SX v = state(3); 
        casadi::SX delta = control(0);
        casadi::SX a = control(1);
        casadi::SX dt = get_params()->dt;
        casadi::SX x_next = x + v * casadi::SX::cos(heading) * dt;
        casadi::SX y_next = y + v * casadi::SX::sin(heading) * dt;
        casadi::SX heading_next = heading + v * casadi::SX::tan(delta) / (vehicle_params_.lf + vehicle_params_.lr) * dt; //车辆轴距
        casadi::SX v_next = v + a * dt;
        casadi::SX next_state = casadi::SX::vertcat({x_next, y_next, heading_next, v_next});
        f_ = casadi::Function("dynamics_function", {z}, {next_state}, {"state+control"}, {"next_state"});
        casadi::SX f_jacobian = casadi::SX::jacobian(next_state, z);
        f_jacobian_ = casadi::Function("dynamics_jacobian", {z}, {f_jacobian}, {"state+control"}, {"dynamics_jacobian"});
        
        // 计算Hessian：遍历每个状态分量
        casadi::SXVector hess_vec;
        std::vector<std::string> hess_names;
        for(int i = 0; i < n_x_; ++i){
            casadi::SX grad_i = casadi::SX::gradient(next_state(i), z);
            casadi::SX hess_i = casadi::SX::jacobian(grad_i, z);
            hess_vec.push_back(hess_i);
            hess_names.push_back("hess_" + std::to_string(i));
        }
        f_hessian_ = casadi::Function("dynamics_hessian", {z}, hess_vec, {"state+control"}, hess_names);   
    }

    void planner_problem::build_terminal_cost_function(){
        casadi::SX x_N = casadi::SX::sym("x_N", n_x_);
        casadi::SX Qf = casadi::SX::sym("Qf", n_x_, n_x_);
        casadi::SX terminal_ref = casadi::SX::sym("terminal_ref", n_x_);
        casadi::SX state_error = x_N - terminal_ref;
        casadi::SX terminal_cost = 0.5 * casadi::SX::mtimes({state_error.T(), Qf, state_error});
        terminal_l_ = casadi::Function("terminal_cost_function", {x_N, Qf, terminal_ref}, {terminal_cost}, {"x_N", "Qf", "terminal_ref"}, {"terminal_cost"});

        // 终端代价梯度
        casadi::SX terminal_l_x = casadi::SX::gradient(terminal_cost, x_N);
        terminal_l_jacobian_ = casadi::Function("terminal_cost_gradient", {x_N, Qf, terminal_ref}, {terminal_l_x}, {"x_N", "Qf", "terminal_ref"}, {"terminal_l_x"});

        // 终端代价Hessian
        casadi::SX terminal_l_xx = casadi::SX::jacobian(terminal_l_x, x_N);
        terminal_l_hessian_ = casadi::Function("terminal_cost_hessian", {x_N, Qf, terminal_ref}, {terminal_l_xx}, {"x_N", "Qf", "terminal_ref"}, {"terminal_l_xx"});
    }

    void planner_problem::build_origin_cost_function(){
        casadi::SX z = casadi::SX::sym("z", n_x_ + n_u_);
        casadi::SX state = z(casadi::Slice(0, n_x_));
        // [x,y,heading,v]
        casadi::SX control = z(casadi::Slice(n_x_, n_x_ + n_u_));
        casadi::SX Q = casadi::SX::sym("Q", n_x_, n_x_);
        casadi::SX R = casadi::SX::sym("R", n_u_, n_u_);
        // 注意，参考状态需要通过frenet坐标系转换得到匹配点
        casadi::SX ref = casadi::SX::sym("ref", n_x_);
        casadi::SX state_error = state - ref;
        casadi::SX stage_cost = 0.5 * casadi::SX::mtimes({state_error.T(), Q, state_error}) + 0.5 * casadi::SX::mtimes({control.T(), R, control});
        l_ = casadi::Function("stage_cost_function", {z, Q, R, ref}, {stage_cost}, {"state+control", "Q", "R", "ref"}, {"stage_cost"});

        // 阶段代价梯度
        casadi::SX l_jacobian = casadi::SX::gradient(stage_cost, z);
        l_jacobian_ = casadi::Function("stage_cost_gradient", {z, Q, R, ref}, {l_jacobian}, {"state+control", "Q", "R", "ref"}, {"stage_l_jacobian"});

        // 阶段代价Hessian
        casadi::SX l_hessian = casadi::SX::jacobian(l_jacobian, z);
        l_hessian_ = casadi::Function("stage_cost_hessian", {z, Q, R, ref}, {l_hessian}, {"state+control", "Q", "R", "ref"}, {"stage_l_hessian"});
    }

    double planner_problem::compute_terminal_cost(const Eigen::VectorXd& x_N){
        casadi::DM x_N_dm(std::vector<double>(x_N.data(), x_N.data() + x_N.size()));
        casadi::DM Qf_dm = casadi::DM::reshape(
            casadi::DM(std::vector<double>(Qf_.data(), Qf_.data() + Qf_.size())),
            n_x_, n_x_);
        auto matched_point = global_frenet_frame_->get_matched_point(double(x_N(0)),double(x_N(1)),double(x_N(2)));
        casadi::DM terminal_ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,reference_speed_});
        auto res = terminal_l_(std::vector<casadi::DM>{x_N_dm, Qf_dm, terminal_ref_dm});
        return static_cast<double>(res[0]);
    }
    double planner_problem::compute_total_cost(const Eigen::MatrixXd& x_and_u){
        int N = x_and_u.rows() - 1;
        double total_cost = 0.0;
        for(int k=0;k<N;++k){
            Eigen::VectorXd x_u_vec = Eigen::VectorXd(n_x_ + n_u_);
            x_u_vec << x_and_u.row(k).transpose();
            total_cost += compute_one_step_cost(x_u_vec,k);
        }
        Eigen::VectorXd x_N = x_and_u.row(N).head(n_x_).transpose();
        total_cost += compute_terminal_cost(x_N);
        return total_cost;
    }
    Eigen::MatrixXd planner_problem::compute_total_gradient(const Eigen::MatrixXd& x_and_u){
        int N = x_and_u.rows() - 1;
        int nx = n_x_ + n_u_;
        Eigen::MatrixXd total_gradient = Eigen::MatrixXd::Zero(N, nx);
        for(int k=0;k<N;++k){
            Eigen::VectorXd x_u_vec = Eigen::VectorXd(n_x_ + n_u_);
            x_u_vec << x_and_u.row(k).transpose();
            total_gradient.row(k) = compute_gradient_one_step(x_u_vec,k).transpose();
            auto mu_grad=compute_mu_gradient_one_step(x_u_vec,k);
            alm_mu_must_next_.row(k)=mu_grad.head(num_c_); // 更新
            alm_mu_obstacles_next_.row(k)=mu_grad.tail(num_obstacles_*2);
        }
        return total_gradient;
    }
    void planner_problem::update(int iter){
        // 更新乘子和障碍系数等内部状态，在新一轮迭代开始前调用
        iter_=iter;
        if(iter==0){
            // 第一轮迭代不更新乘子，只初始化为0
            alm_mu_must_.setZero();
            alm_mu_obstacles_.setZero();
            alm_mu_must_next_.setZero();
            alm_mu_obstacles_next_.setZero();
            alm_rho_=planner_params_->alm_rho_init;
            barrier_rho_=planner_params_->barrier_rho_init;
            return;
        }

        if(planner_params_->use_alm){
            // 更新乘子
            alm_mu_must_ = alm_mu_must_next_;
            alm_mu_obstacles_ = alm_mu_obstacles_next_;
            alm_mu_must_next_.setZero();
            alm_mu_obstacles_next_.setZero();
        }
        // 更新障碍系数
        if(planner_params_->use_alm){
            alm_rho_ *= planner_params_->alm_beta;
        }else{
            barrier_rho_ *= planner_params_->barrier_beta;
        }
    }
    
    Eigen::Tensor<double, 3,Eigen::RowMajor> planner_problem::compute_total_hessian(const Eigen::MatrixXd& x_and_u){
        int N = x_and_u.rows() - 1;
        int nx = n_x_ + n_u_;
        Eigen::Tensor<double, 3,Eigen::RowMajor> total_hessian(N, nx, nx);
        for(int k=0;k<N;++k){
            Eigen::VectorXd x_u_vec = Eigen::VectorXd(n_x_ + n_u_);
            x_u_vec << x_and_u.row(k).transpose();
            Eigen::MatrixXd hess_mat = compute_hessian_one_step(x_u_vec,k);
            for(int i=0;i<nx;++i){
                for(int j=0;j<nx;++j){
                    total_hessian(k,i,j) = hess_mat(i,j);
                }
            }
        }
        return total_hessian;
    }

    void planner_problem::build_one_step_inequal_cost_function_without_obstacle(){
        // 函数输入是速度，加速度，方向盘转角，道路上下界和参考点，通过计算与参考点的距离来判断是否超出道路边界，需要分清楚是上边界还是下边界
        casadi::SX z = casadi::SX::sym("z", n_x_ + n_u_);
        casadi::SX state = z(casadi::Slice(0, n_x_));
        // [x,y,heading,v]
        casadi::SX control = z(casadi::Slice(n_x_, n_x_ + n_u_));
        // 做成三组向量输入，上界，下界，参考点
        casadi::SX upper_bound = casadi::SX::sym("upper_bound",4);// [road_upper_bound, a_max, v_max, steer_max]
        casadi::SX lower_bound = casadi::SX::sym("lower_bound",4);// [road_lower_bound, a_min, v_min, steer_min]
        casadi::SX reference_point = casadi::SX::sym("reference_point", 3); // [x_ref, y_ref, heading_ref]

        casadi::SX road_upper_bound = upper_bound(0);
        casadi::SX road_lower_bound = lower_bound(0);
        casadi::SX a_max=upper_bound(1);
        casadi::SX a_min=lower_bound(1);
        casadi::SX v_max=upper_bound(2);
        casadi::SX v_min=lower_bound(2);
        casadi::SX steer_max=upper_bound(3);
        casadi::SX steer_min=lower_bound(3);

        // 参考点在外部计算好传入，这里直接计算与参考点的距离，要有方向，判断是上边界还是下边界
        casadi::SX dx = state(0) - reference_point(0);
        casadi::SX dy = state(1) - reference_point(1);
        casadi::SX heading_ref = reference_point(2);
        casadi::SX lateral_error = -dx * casadi::SX::sin(heading_ref) + dy * casadi::SX::cos(heading_ref);
        // road_lower_bound < lateral_error < road_upper_bound，就是两个不等式约束
        // road_lower_bound - lateral_error <= 0  和 lateral_error - road_upper_bound <= 0
        casadi::SX road_lower_cost = road_lower_bound - lateral_error; // 这里只返回一个约束的值，外部根据这个值计算乘子相关的代价
        casadi::SX road_upper_cost = lateral_error - road_upper_bound;
        // 加速度边界代价
        casadi::SX a_low_cost = control(1) - a_min;
        casadi::SX a_high_cost = a_max - control(1);
        // 速度边界代价
        casadi::SX v_low_cost = control(0) - v_min;
        casadi::SX v_high_cost = v_max - control(0);
        // 方向盘转角边界代价
        casadi::SX steer_low_cost = control(0) - steer_min;
        casadi::SX steer_high_cost = steer_max - control(0);
        // 现将约束堆叠成向量输出，方便矩阵运算
        casadi::SX ineq_constraints = casadi::SX::vertcat({road_lower_cost, road_upper_cost, a_low_cost, a_high_cost,
                                                         v_low_cost, v_high_cost, steer_low_cost, steer_high_cost});

        // 分别构造ALM和Barrier的代价函数
        // alm还需要乘子和惩罚系数作为输入
        casadi::SX mu = casadi::SX::sym("mu", 8); // 8个不等式约束
        casadi::SX rho = casadi::SX::sym("rho");
        casadi::SX alm_cost = casadi::SX::sum1(mu * ineq_constraints + 0.5 * rho * casadi::SX::pow(casadi::SX::fmax(ineq_constraints+mu/rho, 0), 2));
        // 这就是函数定义
        alm_cost_ = casadi::Function("alm_cost_function", {z, upper_bound, lower_bound, reference_point, mu, rho}, {alm_cost},
                                     {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"});
        // jacobian
        casadi::SX alm_cost_jacobian = casadi::SX::gradient(alm_cost, z);
        alm_cost_jacobian_ = casadi::Function("alm_cost_jacobian", {z, upper_bound, lower_bound, reference_point, mu, rho}, {alm_cost_jacobian},
                                             {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost_jacobian"});
        // hessian
        casadi::SX alm_cost_hessian = casadi::SX::jacobian(alm_cost_jacobian, z);
        alm_cost_hessian_ = casadi::Function("alm_cost_hessian", {z, upper_bound, lower_bound, reference_point, mu, rho}, {alm_cost_hessian},
                                            {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost_hessian"});
        // 另外，定义一个函数计算乘子导数，用于更新乘子
        casadi::SX alm_mu_derivative = casadi::SX::gradient(alm_cost, mu);
        alm_mu_derivative_ = casadi::Function("alm_mu_derivative_function", {z, upper_bound, lower_bound, reference_point, mu, rho}, {alm_mu_derivative},
                                             {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_mu_derivative"});
        
                                             // barrier代价函数,不需要乘子,而且pho是常量，所有时间步和所有约束共用一个pho
        casadi::SX barrier_rho = casadi::SX::sym("barrier_rho");
        casadi::SX barrier_cost = -barrier_rho * casadi::SX::sum1(casadi::SX::log(-ineq_constraints));
        barrier_cost_ = casadi::Function("barrier_cost_function", {z, upper_bound, lower_bound, reference_point, barrier_rho}, {barrier_cost},
                                        {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost"});
        // barrier jacobian
        casadi::SX barrier_cost_jacobian = casadi::SX::gradient(barrier_cost, z);
        barrier_cost_jacobian_ = casadi::Function("barrier_cost_jacobian", {z, upper_bound, lower_bound, reference_point, barrier_rho}, {barrier_cost_jacobian},
                                                {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost_jacobian"});
        // barrier hessian
        casadi::SX barrier_cost_hessian = casadi::SX::jacobian(barrier_cost_jacobian, z);
        barrier_cost_hessian_ = casadi::Function("barrier_cost_hessian", {z, upper_bound, lower_bound, reference_point, barrier_rho}, {barrier_cost_hessian},
                                               {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost_hessian"});
    }

    void planner_problem::build_one_step_obstacle_cost_functions(){
        // 定义障碍物代价，使用椭圆裕度算法
        // 输入是自车状态，障碍物位置和尺寸，先构建不等式约束函数c(z)，然后基于c(z)构建alm和barrier代价函数
        // 首先是椭圆裕度，假设障碍物是椭圆形的，以障碍物中心为原点，x轴和y轴方向的半轴长度分别为a和b，则椭圆的边界由方程(x/a)^2+(y/b)^2=1描述
        // 然后自车的前后轴中心点分别为(x_f, y_f)和(x_r, y_r），使用旋转矩阵将全局坐标系下的点转换到障碍物椭圆坐标系下，长轴方向为x轴，短轴方向为y轴
        // 转换后的点为(x_f', y_f')和(x_r', y_r')，代价要定义为越小越好，即c<=0表示满足约束，因此定义不等式约束函数为：
        // c_f = 1 - (x_f'/a)^2 + (y_f'/b)^2 <= 0,表示前轴点在椭圆外部
        // c_r = 1 - (x_r'/a)^2 + (y_r'/b)^2  <= 0
        // 其中(x_f', y_f') = R_theta * (x_f - x_obs, y_f - y_obs)，R_theta是旋转矩阵，theta是障碍物的朝向角
        casadi::SX z = casadi::SX::sym("z", n_x_ + n_u_);
        casadi::SX state = z(casadi::Slice(0, n_x_));
        // [x,y,heading,v]
        casadi::SX x = state(0);
        casadi::SX y = state(1);
        casadi::SX heading = state(2);
        casadi::SX v = state(3);
        // 自车前后轴中心点位置
        casadi::SX x_f = x + vehicle_params_.lf * casadi::SX::cos(heading);
        casadi::SX y_f = y + vehicle_params_.lf * casadi::SX::sin(heading);
        casadi::SX x_r = x - vehicle_params_.lr * casadi::SX::cos(heading);
        casadi::SX y_r = y - vehicle_params_.lr * casadi::SX::sin(heading);
        // 障碍物参数输入
        casadi::SX obs_params = casadi::SX::sym("obs_params", 5); // [x_obs, y_obs, width, length, theta]
        casadi::SX x_obs = obs_params(0);
        casadi::SX y_obs = obs_params(1);
        casadi::SX width = obs_params(2);
        casadi::SX length = obs_params(3);
        casadi::SX theta = obs_params(4);
        // 旋转矩阵
        casadi::SX cos_theta = casadi::SX::cos(theta);
        casadi::SX sin_theta = casadi::SX::sin(theta);
        casadi::SX R_Theta = casadi::SX::vertcat({casadi::SX::horzcat({cos_theta, sin_theta}),
                                                     casadi::SX::horzcat({-sin_theta, cos_theta})});
        // 计算转换后的点
        casadi::SX front_vec = casadi::SX::vertcat({x_f - x_obs, y_f - y_obs});
        casadi::SX rear_vec = casadi::SX::vertcat({x_r - x_obs, y_r - y_obs});
        casadi::SX front_rotated = casadi::SX::mtimes(R_Theta, front_vec);
        casadi::SX rear_rotated = casadi::SX::mtimes(R_Theta, rear_vec);
        casadi::SX x_f_prime = front_rotated(0);
        casadi::SX y_f_prime = front_rotated(1);
        casadi::SX x_r_prime = rear_rotated(0);
        casadi::SX y_r_prime = rear_rotated(1);
        // 计算椭圆长短轴，使用经验公式
        casadi::SX a = length / 2.0+ vehicle_params_.width / 2.0+vehicle_params_.safety_distance*6;
        casadi::SX b = width / 2.0+ vehicle_params_.length / 2.0+vehicle_params_.safety_distance;
        // 不等式约束函数
        casadi::SX c_f = 1 - casadi::SX::pow(x_f_prime / a, 2) - casadi::SX::pow(y_f_prime / b, 2);
        casadi::SX c_r = 1 - casadi::SX::pow(x_r_prime / a, 2) - casadi::SX::pow(y_r_prime / b, 2);
        casadi::SXVector ineq_constraints_vec = {c_f, c_r};
        casadi::SX ineq_constraints = casadi::SX::vertcat(ineq_constraints_vec);
        // 分别构造ALM和Barrier的代价函数
        // alm还需要乘子和惩罚系数作为输入  
        casadi::SX mu = casadi::SX::sym("mu", 2); // 2个不等式约束
        casadi::SX rho = casadi::SX::sym("rho");
        casadi::SX alm_cost = casadi::SX::sum1(mu * ineq_constraints + 0.5 * rho * casadi::SX::pow(casadi::SX::fmax(ineq_constraints+mu/rho, 0), 2));
        // 这就是函数定义
        alm_obstacle_cost_ = casadi::Function("alm_obstacle_cost_function", {z, obs_params, mu, rho}, {alm_cost},
                                     {"state+control", "obs_params", "mu", "rho"}, {"alm_obstacle_cost"});
        // jacobian
        casadi::SX alm_cost_jacobian = casadi::SX::gradient(alm_cost, z);
        alm_obstacle_jacobian_ = casadi::Function("alm_obstacle_cost_jacobian", {z, obs_params, mu, rho}, {alm_cost_jacobian},
                                             {"state+control", "obs_params", "mu", "rho"}, {"alm_obstacle_cost_jacobian"});
        // hessian
        casadi::SX alm_cost_hessian = casadi::SX::jacobian(alm_cost_jacobian, z);
        alm_obstacle_hessian_ = casadi::Function("alm_obstacle_cost_hessian", {z, obs_params, mu, rho}, {alm_cost_hessian},
                                            {"state+control", "obs_params", "mu", "rho"}, {"alm_obstacle_cost_hessian"});
        // 另外，定义一个函数计算乘子导数，用于更新乘子
        casadi::SX alm_mu_derivative = casadi::SX::gradient(alm_cost, mu);
        alm_obstacle_mu_derivative_ = casadi::Function("alm_obstacle_mu_derivative_function", {z, obs_params, mu, rho}, {alm_mu_derivative},
                                             {"state+control", "obs_params", "mu", "rho"}, {"alm_obstacle_mu_derivative"});

        // barrier代价函数,不需要乘子,而且pho是常量，所有时间步和所有约束共用一个pho
        casadi::SX barrier_rho = casadi::SX::sym("barrier_rho");
        casadi::SX barrier_cost = -barrier_rho * casadi::SX::sum1(casadi::SX::log(-ineq_constraints));
        barrier_obstacle_cost_ = casadi::Function("barrier_obstacle_cost_function", {z, obs_params, barrier_rho}, {barrier_cost},
                                        {"state+control", "obs_params", "barrier_rho"}, {"barrier_obstacle_cost"});
        // barrier jacobian
        casadi::SX barrier_cost_jacobian = casadi::SX::gradient(barrier_cost, z);
        barrier_obstacle_jacobian_ = casadi::Function("barrier_obstacle_cost_jacobian", {z, obs_params, barrier_rho}, {barrier_cost_jacobian},
                                                {"state+control", "obs_params", "barrier_rho"}, {"barrier_obstacle_cost_jacobian"});
        // barrier hessian
        casadi::SX barrier_cost_hessian = casadi::SX::jacobian(barrier_cost_jacobian, z);
        barrier_obstacle_hessian_ = casadi::Function("barrier_obstacle_cost_hessian", {z, obs_params, barrier_rho}, {barrier_cost_hessian},
                                               {"state+control", "obs_params", "barrier_rho"}, {"barrier_obstacle_cost_hessian"});
    }

    void planner_problem::build_functions(){
        build_dynamics_function();
        build_origin_cost_function();
        build_terminal_cost_function();
        build_one_step_inequal_cost_function_without_obstacle();
        build_one_step_obstacle_cost_functions();
    }

    double planner_problem::compute_one_step_cost(const Eigen::VectorXd& x_and_u,int step){
        double total_cost = 0.0;
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << x_and_u.transpose();
        casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
        // 计算阶段代价
        casadi::DM Q_dm = casadi::DM::reshape(
            casadi::DM(std::vector<double>(Q_.data(), Q_.data() + Q_.size())),
            n_x_, n_x_);
        casadi::DM R_dm = casadi::DM::reshape(
            casadi::DM(std::vector<double>(R_.data(), R_.data() + R_.size())),
            n_u_, n_u_);
        auto matched_point = global_frenet_frame_->get_matched_point(double(x_and_u(0)),double(x_and_u(1)),double(x_and_u(2)));
        casadi::DM ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,reference_speed_});
        auto res = l_(std::vector<casadi::DM>{z_row_dm, Q_dm, R_dm, ref_dm});
        total_cost+= static_cast<double>(res[0]);

        // 计算必有的不等式约束代价
        // 输入输出{"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost"}
        // 上下界由每次迭代外部传入
        casadi::DM upper_bound_dm(std::vector<double>(upper_bound_.data(), upper_bound_.data() + upper_bound_.size()));
        casadi::DM lower_bound_dm(std::vector<double>(lower_bound_.data(), lower_bound_.data() + lower_bound_.size()));
        casadi::DM reference_point_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading});
        std::vector<casadi::DM> res_must_c;
        if(planner_params_->use_alm){
            casadi::DM mu_dm(std::vector<double>(alm_mu_must_.row(step).data(), alm_mu_must_.row(step).data() + alm_mu_must_.cols()));
            casadi::DM rho_dm(alm_rho_);
            res_must_c = alm_cost_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, mu_dm, rho_dm});
        }
        else{
            casadi::DM barrier_rho_dm(barrier_rho_);
            res_must_c = barrier_cost_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, barrier_rho_dm});
        }

        total_cost += static_cast<double>(res_must_c[0]);
        // 障碍物代价
        if(obstacles_.size()>0){
            for(size_t i=0;i<obstacles_.size();++i){
                const auto& obs = obstacles_[i];
                casadi::DM obs_params_dm(std::vector<double>{obs.x, obs.y, obs.width, obs.length, obs.heading});
                std::vector<casadi::DM> res_obstacle;
                if(planner_params_->use_alm){
                    casadi::DM mu_dm(std::vector<double>(alm_mu_obstacles_.row(step).data()+i*2, alm_mu_obstacles_.row(step).data() + (i+1)*2));
                    casadi::DM rho_dm(alm_rho_);
                    res_obstacle = alm_obstacle_cost_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, mu_dm, rho_dm});
                }
                else{
                    casadi::DM barrier_rho_dm(barrier_rho_);
                    res_obstacle = barrier_obstacle_cost_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, barrier_rho_dm});
                }
                total_cost += static_cast<double>(res_obstacle[0]);
            }
        }
        return total_cost;
    }
    Eigen::VectorXd planner_problem::compute_gradient_one_step(const Eigen::MatrixXd& x_and_u,int step){
        Eigen::VectorXd total_gradient = Eigen::VectorXd::Zero(n_x_ + n_u_);
        // 输入输出
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost"}
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << x_and_u.transpose();
        casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
        // 计算阶段代价
        casadi::DM Q_dm = casadi::DM::reshape(
            casadi::DM(std::vector<double>(Q_.data(), Q_.data() + Q_.size())),
            n_x_, n_x_);
        casadi::DM R_dm = casadi::DM::reshape(
            casadi::DM(std::vector<double>(R_.data(), R_.data() + R_.size())),
            n_u_, n_u_);
        auto matched_point = global_frenet_frame_->get_matched_point(double(x_and_u(0)),double(x_and_u(1)),double(x_and_u(2)));
        casadi::DM ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,reference_speed_});
        auto res = l_jacobian_(std::vector<casadi::DM>{z_row_dm, Q_dm, R_dm, ref_dm});
        casadi::DM l_jac_dm = casadi::DM::densify(res[0]);
        Eigen::Map<Eigen::VectorXd> l_jac_mat(l_jac_dm.ptr(), n_x_ + n_u_);
        total_gradient+= l_jac_mat;

        // 计算必有的不等式约束代价
        // 输入输出{"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost"}
        // 上下界由每次迭代外部传入
        casadi::DM upper_bound_dm(std::vector<double>(upper_bound_.data(), upper_bound_.data() + upper_bound_.size()));
        casadi::DM lower_bound_dm(std::vector<double>(lower_bound_.data(), lower_bound_.data() + lower_bound_.size()));
        casadi::DM reference_point_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading});
        std::vector<casadi::DM> res_must_c;
        if(planner_params_->use_alm){
            casadi::DM mu_dm(std::vector<double>(alm_mu_must_.row(step).data(), alm_mu_must_.row(step).data() + alm_mu_must_.cols()));
            casadi::DM rho_dm(alm_rho_);
            res_must_c = alm_cost_jacobian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, mu_dm, rho_dm});
        }
        else{
            casadi::DM barrier_rho_dm(barrier_rho_);
            res_must_c = barrier_cost_jacobian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, barrier_rho_dm});
        }
        casadi::DM c_must_jac_dm = casadi::DM::densify(res_must_c[0]);
        Eigen::Map<Eigen::VectorXd> c_must_jac_mat(c_must_jac_dm.ptr(), n_x_ + n_u_);
        total_gradient+= c_must_jac_mat;

        // 障碍物代价
        if(obstacles_.size()>0){
            for(size_t i=0;i<obstacles_.size();++i){
                const auto& obs = obstacles_[i];
                casadi::DM obs_params_dm(std::vector<double>{obs.x, obs.y, obs.width, obs.length, obs.heading});
                std::vector<casadi::DM> res_obstacle;
                if(planner_params_->use_alm){
                    casadi::DM mu_dm(std::vector<double>(alm_mu_obstacles_.row(step).data()+i*2, alm_mu_obstacles_.row(step).data() + (i+1)*2));
                    casadi::DM rho_dm(alm_rho_);
                    res_obstacle = alm_obstacle_jacobian_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, mu_dm, rho_dm});
                }
                else{
                    casadi::DM barrier_rho_dm(barrier_rho_);
                    res_obstacle = barrier_obstacle_jacobian_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, barrier_rho_dm});
                }
                casadi::DM c_obs_jac_dm = casadi::DM::densify(res_obstacle[0]);
                Eigen::Map<Eigen::VectorXd> c_obs_jac_mat(c_obs_jac_dm.ptr(), n_x_ + n_u_);
                total_gradient+= c_obs_jac_mat;
            }
        }
        return total_gradient;
    }

    Eigen::VectorXd planner_problem::compute_terminal_gradient(const Eigen::VectorXd& x_N){
        casadi::DM x_N_dm(std::vector<double>(x_N.data(), x_N.data() + x_N.size()));
        casadi::DM Qf_dm = casadi::DM::reshape(
            casadi::DM(std::vector<double>(Qf_.data(), Qf_.data() + Qf_.size())),
            n_x_, n_x_);
        auto matched_point = global_frenet_frame_->get_matched_point(double(x_N(0)),double(x_N(1)),double(x_N(2)));
        casadi::DM terminal_ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,reference_speed_});
        auto res = terminal_l_jacobian_(std::vector<casadi::DM>{x_N_dm, Qf_dm, terminal_ref_dm});
        casadi::DM l_jac_dm = casadi::DM::densify(res[0]);
        Eigen::Map<Eigen::VectorXd> l_jac_mat(l_jac_dm.ptr(), n_x_);
        return l_jac_mat;
    }

    Eigen::MatrixXd planner_problem::compute_hessian_one_step(const Eigen::MatrixXd& x_and_u,int step){
        Eigen::MatrixXd total_hessian = Eigen::MatrixXd::Zero(n_x_ + n_u_ ,n_x_ + n_u_);
        // 输入输出
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost"}
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << x_and_u.transpose();
        casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
        // 计算阶段代价
        casadi::DM Q_dm = casadi::DM::reshape(
            casadi::DM(std::vector<double>(Q_.data(), Q_.data() + Q_.size())),
            n_x_, n_x_);
        casadi::DM R_dm = casadi::DM::reshape(
            casadi::DM(std::vector<double>(R_.data(), R_.data() + R_.size())),
            n_u_, n_u_);
        // 参考点需要通过frenet坐标系转换得到匹配点
        auto matched_point = global_frenet_frame_->get_matched_point(double(x_and_u(0)),double(x_and_u(1)),double(x_and_u(2)));
        casadi::DM ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,reference_speed_});
        auto res = l_hessian_(std::vector<casadi::DM>{z_row_dm, Q_dm, R_dm, ref_dm});
        casadi::DM l_hes_dm = casadi::DM::densify(res[0]);
        Eigen::Map<Eigen::MatrixXd> l_hes_mat(l_hes_dm.ptr(), n_x_ + n_u_, n_x_ + n_u_);
        total_hessian+= l_hes_mat;

        // 计算必有的不等式约束代价
        // 输入输出{"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost"}
        // 上下界由每次迭代外部传入
        casadi::DM upper_bound_dm(std::vector<double>(upper_bound_.data(), upper_bound_.data() + upper_bound_.size()));
        casadi::DM lower_bound_dm(std::vector<double>(lower_bound_.data(), lower_bound_.data() + lower_bound_.size()));
        casadi::DM reference_point_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading});
        std::vector<casadi::DM> res_must_c;
        if(planner_params_->use_alm){
            casadi::DM mu_dm(std::vector<double>(alm_mu_must_.row(step).data(), alm_mu_must_.row(step).data() + alm_mu_must_.cols()));
            casadi::DM rho_dm(alm_rho_);
            res_must_c = alm_cost_hessian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, mu_dm, rho_dm});
        }
        else{
            casadi::DM barrier_rho_dm(barrier_rho_);
            res_must_c = barrier_cost_hessian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, barrier_rho_dm});
        }
        casadi::DM c_must_hes_dm = casadi::DM::densify(res_must_c[0]);
        Eigen::Map<Eigen::MatrixXd> c_must_hes_mat(c_must_hes_dm.ptr(), n_x_ + n_u_, n_x_ + n_u_);
        total_hessian+= c_must_hes_mat;

        // 障碍物代价
        if(obstacles_.size()>0){
            for(size_t i=0;i<obstacles_.size();++i){
                const auto& obs = obstacles_[i];
                casadi::DM obs_params_dm(std::vector<double>{obs.x, obs.y, obs.width, obs.length, obs.heading});
                std::vector<casadi::DM> res_obstacle;
                if(planner_params_->use_alm){
                    casadi::DM mu_dm(std::vector<double>(alm_mu_obstacles_.row(step).data()+i*2, alm_mu_obstacles_.row(step).data() + (i+1)*2));
                    casadi::DM rho_dm(alm_rho_);
                    res_obstacle = alm_obstacle_hessian_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, mu_dm, rho_dm});
                }
                else{
                    casadi::DM barrier_rho_dm(barrier_rho_);
                    res_obstacle = barrier_obstacle_hessian_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, barrier_rho_dm});
                }
                casadi::DM c_obs_hes_dm = casadi::DM::densify(res_obstacle[0]);
                Eigen::Map<Eigen::MatrixXd> c_obs_hes_mat(c_obs_hes_dm.ptr(), n_x_ + n_u_, n_x_ + n_u_);
                total_hessian+= c_obs_hes_mat;
            }
        }
        return total_hessian;
    }

    Eigen::MatrixXd planner_problem::compute_terminal_hessian(const Eigen::VectorXd& x_N){
        casadi::DM x_N_dm(std::vector<double>(x_N.data(), x_N.data() + x_N.size()));
        casadi::DM Qf_dm = casadi::DM::reshape(
            casadi::DM(std::vector<double>(Qf_.data(), Qf_.data() + Qf_.size())),
            n_x_, n_x_);
        auto matched_point = global_frenet_frame_->get_matched_point(double(x_N(0)),double(x_N(1)),double(x_N(2)));
        casadi::DM terminal_ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,reference_speed_});
        auto res = terminal_l_hessian_(std::vector<casadi::DM>{x_N_dm, Qf_dm, terminal_ref_dm});
        casadi::DM l_hes_dm = casadi::DM::densify(res[0]);
        Eigen::Map<Eigen::MatrixXd> l_hes_mat(l_hes_dm.ptr(), n_x_, n_x_);
        return l_hes_mat;
    }

    Eigen::VectorXd planner_problem::compute_mu_gradient_one_step(const Eigen::MatrixXd& x_and_u,int step){
        Eigen::VectorXd total_gradient = Eigen::VectorXd::Zero(num_c_ + num_obstacles_ * 2);
        // 输入输出
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost"}
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << x_and_u.transpose();
        casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
        auto matched_point = global_frenet_frame_->get_matched_point(double(x_and_u(0)),double(x_and_u(1)),double(x_and_u(2)));
        // 计算必有的不等式约束代价
        // 输入输出{"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost"}
        // 上下界由每次迭代外部传入
        casadi::DM upper_bound_dm(std::vector<double>(upper_bound_.data(), upper_bound_.data() + upper_bound_.size()));
        casadi::DM lower_bound_dm(std::vector<double>(lower_bound_.data(), lower_bound_.data() + lower_bound_.size())); 
        casadi::DM reference_point_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading});
        casadi::DM mu_dm(std::vector<double>(alm_mu_must_.row(step).data(), alm_mu_must_.row(step).data() + alm_mu_must_.cols()));
        casadi::DM rho_dm(alm_rho_);
        auto res_must_c = alm_mu_derivative_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, mu_dm, rho_dm});
        casadi::DM c_must_jac_dm = casadi::DM::densify(res_must_c[0]);
        Eigen::Map<Eigen::VectorXd> c_must_jac_mat(c_must_jac_dm.ptr(), num_c_);
        total_gradient.head(num_c_) += c_must_jac_mat;

        // 障碍物代价
        if(obstacles_.size()>0){
            for(size_t i=0;i<obstacles_.size();++i){
                const auto& obs = obstacles_[i];
                casadi::DM obs_params_dm(std::vector<double>{obs.x, obs.y, obs.width, obs.length, obs.heading});
                casadi::DM mu_dm(std::vector<double>(alm_mu_obstacles_.row(step).data()+i*2, alm_mu_obstacles_.row(step).data() + (i+1)*2));
                casadi::DM rho_dm(alm_rho_);
                auto res_obstacle = alm_obstacle_mu_derivative_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, mu_dm, rho_dm});
                casadi::DM c_obs_jac_dm = casadi::DM::densify(res_obstacle[0]);
                Eigen::Map<Eigen::VectorXd> c_obs_jac_mat(c_obs_jac_dm.ptr(), 2);
                total_gradient.segment(num_c_ + i * 2, 2) += c_obs_jac_mat;
            }
        }
        return total_gradient;
    }
}

}
