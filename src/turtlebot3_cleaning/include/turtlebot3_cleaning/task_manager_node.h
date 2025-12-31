#ifndef TASK_MANAGER_NODE_H
#define TASK_MANAGER_NODE_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <map>
#include <string>

enum class MissionState {
    INIT,
    GO_TO_ROOM_ENTRY,
    QR_VERIFY,
    EXECUTE_CLEANING,
    NEXT_ROOM,
    REPORT,
    FINISH
};

struct Pose {
    double x, y, yaw;
};

struct Room {
    std::string name;
    Pose entry_goal;
    std::vector<Pose> cleaning_goals;
    std::string qr_expected;
    bool verified;
    bool cleaned;
    bool skipped;
    std::string status; // "SUCCESS", "FAIL", "SKIPPED" statülerini alabilir.
};

class TaskManager {
public:
    TaskManager();
    void run();

private:
    void loadMission();
    void qrCallback(const std_msgs::String::ConstPtr& msg);
    bool moveTo(const Pose& target, double timeout = 30.0);
    void recoverParams();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient* mb_client_;
    
    ros::Subscriber qr_sub_;
    std::string last_qr_code_;
    bool qr_received_;

    std::vector<Room> rooms_;
    int current_room_idx_;
    MissionState current_state_;
    
    int qr_retry_count_;
    int move_retry_count_;
};

#endif // TASK_MANAGER_NODE_H
