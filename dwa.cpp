

void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f &pos,　                     //机器人的位置
    const Eigen::Vector3f &vel,　                     //当前机器人速度
    const Eigen::Vector3f &goal,　                    //目标点
    base_local_planner::LocalPlannerLimits *limits,　 //运动特性（加速度、最大最小速度…）
    const Eigen::Vector3f &vsamples,　                //样本
    bool discretize_by_time)
{
    //给定机器人的最大最小运动速度
    double max_vel_th = limits->max_vel_theta;
    double min_vel_th = -1.0 * max_vel_th;
    discretize_by_time_ = discretize_by_time;
    Eigen::Vector3f acc_lim = limits->getAccLimits();
    pos_ = pos;
    vel_ = vel;
    limits_ = limits;
    next_sample_index_ = 0;
    sample_params_.clear();

    double min_vel_x = limits->min_vel_x;
    double max_vel_x = limits->max_vel_x;
    double min_vel_y = limits->min_vel_y;
    double max_vel_y = limits->max_vel_y;

    // if sampling number is zero in any dimension, we don’t generate samples generically
    if (vsamples[0] * vsamples[1] * vsamples[2] > 0)
    {
        // compute the feasible velocity space based on the rate at which we run
        Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
        Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();
        if (!use_dwa_)
        {
            //根据机器人位置到目标点的距离，限制机器人的最大运动速度
            double dist = hypot(goal[0] - pos[0], goal[1] - pos[1]);
            max_vel_x = std::max(std::min(max_vel_x, dist / sim_time_), min_vel_x);
            max_vel_y = std::max(std::min(max_vel_y, dist / sim_time_), min_vel_y);
            // 根据控制周期和加速度特性，确定机器人可达的最大最小速度
            // 此处用的是sim_time_仿真时间，确定的是接下来一段时间内机器人可达的运动速度范围，默认是1s
            max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_time_);
            max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_time_);
            max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_time_);
            min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_time_);
            min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_time_);
            min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_time_);
        }
        else
        {
            // 此处用的sim_period_是控制周期，也就是只确定下一个控制周期机器人的运动速度范围
            max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_period_);
            max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_period_);
            max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_period_);
            min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_period_);
            min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_period_);
            min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_period_);
        }
        //根据给定的速度样本数，在速度空间内等间距的获取速度样本
        Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
        VelocityIterator x_it(min_vel[0], max_vel[0], vsamples[0]);
        VelocityIterator y_it(min_vel[1], max_vel[1], vsamples[1]);
        VelocityIterator th_it(min_vel[2], max_vel[2], vsamples[2]);
        for (; !x_it.isFinished(); x_it++)
        {
            vel_samp[0] = x_it.getVelocity();
            for (; !y_it.isFinished(); y_it++)
            {
                vel_samp[1] = y_it.getVelocity();
                for (; !th_it.isFinished(); th_it++)
                {
                    vel_samp[2] = th_it.getVelocity();
                    // ROS_DEBUG(“Sample %f, %f, %f”, vel_samp[0], vel_samp[1], vel_samp[2]);
                    sample_params_.push_back(vel_samp);
                }
                th_it.reset();
            }
            y_it.reset();
        }
    }
}

bool SimpleTrajectoryGenerator::generateTrajectory(
    Eigen::Vector3f pos,               //机器人的位姿
    Eigen::Vector3f vel,               //运动速度
    Eigen::Vector3f sample_target_vel, //样本速度
    base_local_planner::Trajectory &traj)
{ //需要生成的轨迹
    double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);
    double eps = 1e-4;
    traj.cost_ = -1.0; // placed here in case we return early
    // trajectory might be reused so we’ll make sure to reset it
    traj.resetPoints();

    //确定样本是否超过设定的最大移动速度
    // make sure that the robot would at least be moving with one of
    // the required minimum velocities for translation and rotation (if set)
    if ((limits_->min_vel_trans >= 0 && vmag + eps < limits_->min_vel_trans) &&
        (limits_->min_vel_theta >= 0 && fabs(sample_target_vel[2]) + eps < limits_->min_vel_theta))
    {
        return false;
    }
    // make sure we do not exceed max diagonal (x+y) translational velocity (if set)
    if (limits_->max_vel_trans >= 0 && vmag - eps > limits_->max_vel_trans)
    {
        return false;
    }

    //确定仿真使用的控制周期数
    int num_steps;
    if (discretize_by_time_)
    {
        num_steps = ceil(sim_time_ / sim_granularity_);
    }
    else
    {
        // compute the number of steps we must take along this trajectory to be “safe”
        double sim_time_distance = vmag * sim_time_;                    // the distance the robot would travel in sim_time if it did not change velocity
        double sim_time_angle = fabs(sample_target_vel[2]) * sim_time_; // the angle the robot would rotate in sim_time
        num_steps =
            ceil(std::max(sim_time_distance / sim_granularity_,
                          sim_time_angle / angular_sim_granularity_));
    }

    if (num_steps == 0)
    {
        return false;
    }

    //确定生成轨迹的时间间隔（仅对利用仿真时间进行速度采样的情况）
    // compute a timestep
    double dt = sim_time_ / num_steps;
    traj.time_delta_ = dt;

    Eigen::Vector3f loop_vel;
    //连续加速意味着用的是仿真时间进行的速度采样，不是单个控制周期能达到的运动速度。因此，需要根据机器人的运动特性确定接下来的控制周期内机器人能达到的运动速度
    if (continued_acceleration_)
    {
        // assuming the velocity of the first cycle is the one we want to store in the trajectory object
        loop_vel = computeNewVelocities(sample_target_vel, vel, limits_->getAccLimits(), dt);
        traj.xv_ = loop_vel[0];
        traj.yv_ = loop_vel[1];
        traj.thetav_ = loop_vel[2];
    }
    else
    {
        //否则用的就是仿真周期进行的采样，直接将采样速度作为生成轨迹的速度
        // assuming sample_vel is our target velocity within acc limits for one timestep
        loop_vel = sample_target_vel;
        traj.xv_ = sample_target_vel[0];
        traj.yv_ = sample_target_vel[1];
        traj.thetav_ = sample_target_vel[2];
    }

    //根据仿真的周期数，生成仿真轨迹
    for (int i = 0; i < num_steps; ++i)
    {
        // add the point to the trajectory so we can draw it later if we want
        traj.addPoint(pos[0], pos[1], pos[2]);
        //如果用的是仿真时间进行的速度采样，在每个仿真控制周期内，速度需要根据加减速特性确定
        if (continued_acceleration_)
        {
            // calculate velocities
            loop_vel = computeNewVelocities(sample_target_vel, loop_vel, limits_->getAccLimits(), dt);
            // ROS_WARN_NAMED(“Generator”, “Flag: %d, Loop_Vel %f, %f, %f”, continued_acceleration_, loop_vel[0], loop_vel[1], loop_vel[2]);
        }
        //根据速度和时间，确定运动轨迹上的下一个点
        // update the position of the robot using the velocities passed in
        pos = computeNewPositions(pos, loop_vel, dt);

    } // end for simulation steps

    return true; // trajectory has at least one point
}

double CostmapModel::footprintCost(const geometry_msgs::Point &position,                 //机器人在全局坐标系下的位置
                                   const std::vector<geometry_msgs::Point> &footprint,   //机器人轮廓
                                   double inscribed_radius, double circumscribed_radius) //内切圆、外接圆半径
{
    // used to put things into grid coordinates
    unsigned int cell_x, cell_y;
    // get the cell coord of the center point of the robot
    //获得机器人在地图坐标系下的坐标
    if (!costmap_.worldToMap(position.x, position.y, cell_x, cell_y))
        return -1.0;
    // if number of points in the footprint is less than 3, we’ll just assume a circular robot
    //当轮廓上的点数少于３时，认为机器人是个圆形机器人，并且只判断机器人中心是否在不可走区域
    if (footprint.size() < 3)
    {
        unsigned char cost = costmap_.getCost(cell_x, cell_y);
        // if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE)
        if (cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION)
            return -1.0;
        return cost;
    }
    // now we really have to lay down the footprint in the costmap grid
    unsigned int x0, x1, y0, y1;
    double line_cost = 0.0;
    double footprint_cost = 0.0;
    // footprint是一个多边形，判断该多边形是否与障碍物发生碰撞的方法是：计算多边形的所有边的最大代价值，从而确定是否与障碍物相撞
    // we need to rasterize each line in the footprint
    for (unsigned int i = 0; i < footprint.size() - 1; ++i)
    {
        // get the cell coord of the first point
        //获得地图中机器人轮廓上的一个点的坐标
        if (!costmap_.worldToMap(footprint[i].x, footprint[i].y, x0, y0))
            return -1.0;
        //获得地图中相邻轮廓点的坐标
        // get the cell coord of the second point
        if (!costmap_.worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1))
            return -1.0;
        //确定当前轮廓点与相邻轮廓点构成的边的最大代价值
        line_cost = lineCost(x0, x1, y0, y1);
        //选取所有边的最大代价值
        footprint_cost = std::max(line_cost, footprint_cost);
        // if there is an obstacle that hits the line… we know that we can return false right away
        if (line_cost < 0)
            return -1.0;
    }
    //获取第一个轮廓点与最后一个轮廓点构成的边的最大代价值
    // we also need to connect the first point in the footprint to the last point
    // get the cell coord of the last point
    if (!costmap_.worldToMap(footprint.back().x, footprint.back().y, x0, y0))
        return -1.0;
    // get the cell coord of the first point
    if (!costmap_.worldToMap(footprint.front().x, footprint.front().y, x1, y1))
        return -1.0;
    line_cost = lineCost(x0, x1, y0, y1);
    //确定所有边的最大代价值
    footprint_cost = std::max(line_cost, footprint_cost);
    if (line_cost < 0)
        return -1.0;
    // if all line costs are legal… then we can return that the footprint is legal
    return footprint_cost;
}

// mark the point of the costmap as local goal where global_plan first leaves the area (or its last point)
void MapGrid::setLocalGoal(const costmap_2d::Costmap2D &costmap,　                        //局部代价地图
                           const std::vector<geometry_msgs::PoseStamped> &global_plan) 　 //全局路径
{
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());
    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;
    //调整全局路径分辨率与地图分辨率一致
    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());
    // 将全局路径与局部代价地图边界的交点作为局部目标点
    for (unsigned int i = 0; i < adjusted_global_plan.size(); ++i)
    {
        double g_x = adjusted_global_plan[i].pose.position.x;
        double g_y = adjusted_global_plan[i].pose.position.y;
        unsigned int map_x, map_y;
        if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION)
        {
            local_goal_x = map_x;
            local_goal_y = map_y;
            started_path = true;
        }
        else
        {
            if (started_path)
            {
                break;
            } // else we might have a non pruned path, so we just continue
        }
    }
    if (!started_path)
    {
        ROS_ERROR(“None of the points of the global plan were in the local costmap, global plan points too far from robot”);
        return;
    }
    //构建距离优先队列，并添加局部目标点作为队列的第一个点
    queue<MapCell *> path_dist_queue;
    if (local_goal_x >= 0 && local_goal_y >= 0)
    {
        MapCell &current = getCell(local_goal_x, local_goal_y);
        costmap.mapToWorld(local_goal_x, local_goal_y, goal_x_, goal_y_);
        current.target_dist = 0.0;
        current.target_mark = true;
        path_dist_queue.push(&current);
    }
    //按优先队列的顺序，从局部目标点开始以单个栅格为步长向外膨胀，从而直接确定出每个栅格距离局部目标点的距离
    computeTargetDistance(path_dist_queue, costmap);
}
