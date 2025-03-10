#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "kfzbshtl_msgs/msg/stamped_gesture.hpp"
#include "kfzbshtl_msgs/msg/stamped_control_guard_info.hpp"
#include "kfzbshtl_msgs/msg/stamped_control_guard_cmd_longitudinal.hpp"
#include "kfzbshtl_msgs/msg/stamped_control_guard_cmd_lateral.hpp"
#include "kfzbshtl_msgs/msg/decoded_can_msg.hpp"
#include <array>
#include <string>

enum State {
    OFF = 0,
    IDLE = 1,
    ACTIVE_FORWARD = 2,
    ACTIVE_REVERSE = 3,
};

std::string stateToString(State state) {
    switch (state) {
        case OFF: return "OFF";
        case IDLE: return "IDLE";
        case ACTIVE_FORWARD: return "ACTIVE_FORWARD";
        case ACTIVE_REVERSE: return "ACTIVE_REVERSE";
        default: return "UNKNOWN"; // Handle unknown enum values
    }
}

enum Gesture { 
    NONE = kfzbshtl_msgs::msg::StampedGesture::NONE,
    START = kfzbshtl_msgs::msg::StampedGesture::START,
    STOP = kfzbshtl_msgs::msg::StampedGesture::STOP, 
    REVERSE = kfzbshtl_msgs::msg::StampedGesture::REVERSE, 
    LESS_DISTANCE = kfzbshtl_msgs::msg::StampedGesture::LESS_DISTANCE,
    MORE_DISTANCE = kfzbshtl_msgs::msg::StampedGesture::MORE_DISTANCE,
    MORE_LEFT = kfzbshtl_msgs::msg::StampedGesture::MORE_LEFT,
    MORE_RIGHT = kfzbshtl_msgs::msg::StampedGesture::MORE_RIGHT
};

std::string gestureToString(Gesture gesture) {
    switch (gesture) {
        case NONE: return "NONE";
        case START: return "START";
        case STOP: return "STOP";
        case REVERSE: return "REVERSE";
        case LESS_DISTANCE: return "LESS_DISTANCE";
        case MORE_DISTANCE: return "MORE_DISTANCE";
        case MORE_LEFT: return "MORE_LEFT";
        case MORE_RIGHT: return "MORE_RIGHT";
        default: return "UNKNOWN"; // Handle unknown enum values
    }
}

class Pedestrian {
public: 
    Pedestrian(double x=0, double y = 0, std::string id ="" ) {
        this->x = x; 
        this->y = y; 
        this->id = id;
    };
    double getX() {return this->x;}
    double getY() {return this->y;}
    std::string getID() {return this->id;}
private:
    double x;
    double y;
    std::string id;
};


class PedFollowNode : public rclcpp::Node {
public:
    PedFollowNode() : Node("ped_follow"), state_(State::OFF)  {        
        this->reset();
        //declare parameters
        this->declare_parameter<int>("dist_long_level_max", 2);
        this->declare_parameter<int>("dist_lat_level_max", 2);
        this->declare_parameter<double>("t_dist_update_min", 1.0);
        this->declare_parameter<double>("dist_long_per_level", 0.5);
        this->declare_parameter<double>("dist_lat_per_level", 0.5);
        this->declare_parameter<double>("d_t_cmd_max", 0.5);
        this->declare_parameter<double>("dist_safety_long", 3.0);
        this->declare_parameter<double>("controller_long_K_d", 1.0);
        this->declare_parameter<double>("controller_long_K_v", 1.0);
        this->declare_parameter<double>("v_max_forward", 1.5);
        this->declare_parameter<double>("v_max_reverse", 0.5);
        this->declare_parameter<double>("a_limit_forward_incr", 0.50);
        this->declare_parameter<double>("a_limit_reverse_incr", 0.25);
        this->declare_parameter<double>("a_limit_decr", 3.0);
        this->declare_parameter<double>("controller_lat_lookahead_dist", 5.0);
        this->declare_parameter<double>("st_agl_limit", 24.3);
        this->declare_parameter<double>("d_st_agl_limit", 11.4);
        this->declare_parameter<std::string>("can_mode", "can");  
        this->declare_parameter<bool>("consider_gestures", true);

        // set parameters based on config file
        this->dist_long_level_max_ = this->get_parameter("dist_long_level_max").as_int();
        this->dist_lat_level_max_ = this->get_parameter("dist_lat_level_max").as_int();
        this->t_dist_update_min_ = this->get_parameter("t_dist_update_min").as_double();
        this->dist_long_per_level_ = this->get_parameter("dist_long_per_level").as_double();
        this->dist_lat_per_level_ = this->get_parameter("dist_lat_per_level").as_double();
        this->d_t_cmd_max_ = this->get_parameter("d_t_cmd_max").as_double();
        this->dist_safety_long_ = this->get_parameter("dist_safety_long").as_double();   
        this->controller_long_K_v_ = this->get_parameter("controller_long_K_v").as_double();   
        this->controller_long_K_d_ = this->get_parameter("controller_long_K_d").as_double();   
        this->v_max_forward_ = this->get_parameter("v_max_forward").as_double();
        this->v_max_reverse_ = this->get_parameter("v_max_reverse").as_double();     
        this->a_limit_forward_incr_ = 0.50; this->get_parameter("a_limit_forward_incr").as_double();
        this->a_limit_reverse_incr_ = this->get_parameter("a_limit_reverse_incr").as_double();
        this->a_limit_decr_ = this->get_parameter("a_limit_decr").as_double();        
        this->controller_lat_lookahead_dist_ = this->get_parameter("controller_lat_lookahead_dist").as_double();
        this->st_agl_limit_ = this->get_parameter("st_agl_limit").as_double();
        this->d_st_agl_limit_ = this->get_parameter("d_st_agl_limit").as_double();            
        this->can_mode_ = this->get_parameter("can_mode").as_string();
        this->consider_gestures_ = this->get_parameter("consider_gestures").as_bool();

        //adjust Parameters        
        if(this->dist_safety_long_<this->dist_safety_long_min_){ //check if safety distance is bigger than required minimum
            this->dist_safety_long_= this->dist_safety_long_min_;
            RCLCPP_ERROR(this->get_logger(), "Parameter 'dist_safety_long' is smaller than minimum allow safety distance of %f. Increase value!", this->dist_safety_long_min_);
        }
        if (this->a_limit_decr_<this->a_limit_decr_min_) { //breaking value, this cannot be too small
            this->a_limit_decr_=10; //do not set to zero, this is breaking!!
            RCLCPP_ERROR(this->get_logger(), "Parameter 'a_limit_decr_' is negative or too small (<%f m/s2). Set it to an increased value!", this->a_limit_decr_min_);
        }
        //check for non-negative values
        if (this->dist_long_level_max_<0) { 
            this->dist_long_level_max_ = 0;
            RCLCPP_ERROR(this->get_logger(), "Parameter 'dist_long_level_max' is negative. Set it to a non neagtive value!");
        }
        if (this->dist_lat_level_max_<0) { 
            this->dist_lat_level_max_ = 0;
            RCLCPP_ERROR(this->get_logger(), "Parameter 'dist_lat_level_max' is negative. Set it to a non neagtive value!");
        }
        if (this->st_agl_limit_<0) { 
            this->st_agl_limit_ = 0.0;
            RCLCPP_ERROR(this->get_logger(), "Parameter 'st_agl_limit' is negative. Set it to a non neagtive value!");
        }
        if (this->v_max_forward_<0) {
            this->v_max_forward_=0; 
            RCLCPP_ERROR(this->get_logger(), "Parameter 'v_max_forward' is negative. Set it to a non neagtive value!");
        }
        if (this->v_max_reverse_<0) {
            this->v_max_reverse_=0; 
            RCLCPP_ERROR(this->get_logger(), "Parameter 'v_max_reverse' is negative. Set it to a non neagtive value!");
        }
        if (this->a_limit_forward_incr_<0) {
            this->a_limit_forward_incr_=0; 
            RCLCPP_ERROR(this->get_logger(), "Parameter 'a_limit_forward_inc' is negative. Set it to a non neagtive value!");
        }
        if (this->a_limit_reverse_incr_<0) {
            this->a_limit_reverse_incr_=0; 
            RCLCPP_ERROR(this->get_logger(), "Parameter 'a_limit_reverse_incr' is negative. Set it to a non neagtive value!");
        }

        if (this->d_st_agl_limit_<0) {
            this->d_st_agl_limit_=0; 
            RCLCPP_ERROR(this->get_logger(), "Parameter 'd_st_agl_limit' is negative. Set it to a non neagtive value!");
        }

        //log Parameters
        RCLCPP_INFO(this->get_logger(), "dist_long_level_max: %i", this->dist_long_level_max_);
        RCLCPP_INFO(this->get_logger(), "dist_lat_level_max: %i", this->dist_lat_level_max_);
        RCLCPP_INFO(this->get_logger(), "t_dist_update_min: %g", this->t_dist_update_min_);
        RCLCPP_INFO(this->get_logger(), "dist_long_per_level: %g", this->dist_long_per_level_);
        RCLCPP_INFO(this->get_logger(), "dist_lat_per_level: %g", this->dist_lat_per_level_);
        RCLCPP_INFO(this->get_logger(), "dist_safety_long: %g", this->dist_safety_long_);
        RCLCPP_INFO(this->get_logger(), "d_t_cmd_max: %g", this->d_t_cmd_max_);
        RCLCPP_INFO(this->get_logger(), "dist_safety_long: %g", this->dist_safety_long_);
        RCLCPP_INFO(this->get_logger(), "controller_long_K_v: %g", this->controller_long_K_v_);
        RCLCPP_INFO(this->get_logger(), "controller_long_K_d: %g", this->controller_long_K_d_);
        RCLCPP_INFO(this->get_logger(), "v_max_forward: %g", this->v_max_forward_);
        RCLCPP_INFO(this->get_logger(), "v_max_reverse: %g", this->v_max_reverse_);
        RCLCPP_INFO(this->get_logger(), "a_limit_forward_incr: %g", this->a_limit_forward_incr_);
        RCLCPP_INFO(this->get_logger(), "a_limit_reverse_incr: %g", this->a_limit_reverse_incr_);
        RCLCPP_INFO(this->get_logger(), "a_limit_decr: %g", this->a_limit_decr_);
        RCLCPP_INFO(this->get_logger(), "controller_lat_lookahead_dist: %g", this->controller_lat_lookahead_dist_);
        // RCLCPP_INFO(this->get_logger(), "controller_lat_yaw_K_p: %g", this->controller_lat_yaw_K_p_);
        RCLCPP_INFO(this->get_logger(), "st_agl_limit: %g", this->st_agl_limit_);
        RCLCPP_INFO(this->get_logger(), "d_st_agl_limit: %g", this->d_st_agl_limit_);
        RCLCPP_INFO(this->get_logger(), "can_mode: %s", this->can_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "consider_gestures: %s", (this->consider_gestures_ ? "true": "false") );

        // Initialize publishers
        control_cmd_long_publisher_ = this->create_publisher<kfzbshtl_msgs::msg::StampedControlGuardCmdLongitudinal>(
            "control_cmd_long", 1);

        control_cmd_lat_publisher_ = this->create_publisher<kfzbshtl_msgs::msg::StampedControlGuardCmdLateral>(
            "control_cmd_lat", 1);

        // Initialize subscribers
        control_guard_info_subscriber_ = this->create_subscription<kfzbshtl_msgs::msg::StampedControlGuardInfo>(
            "control_guard_info", 1, std::bind(&PedFollowNode::controlGuardInfoCallback, this, std::placeholders::_1));

        if (this->can_mode_ == "can") {
            can_id1155_subscriber_ = this->create_subscription<kfzbshtl_msgs::msg::DecodedCanMsg>(
            "from_can_bus/msgid1155", 1, std::bind(&PedFollowNode::can_id1155_callback, this, std::placeholders::_1));

            can_id451_subscriber_ = this->create_subscription<kfzbshtl_msgs::msg::DecodedCanMsg>(
            "from_can_bus/msgid451", 1, std::bind(&PedFollowNode::can_id451_callback, this, std::placeholders::_1));
        }

        if (this->consider_gestures_ == true) {
            ped_gesture_subscriber_ = this->create_subscription<kfzbshtl_msgs::msg::StampedGesture>(
            "ped_gesture", 1, std::bind(&PedFollowNode::gestureCallback, this, std::placeholders::_1));
        }   

        ped_object_subscriber_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            "ped_object", 1, std::bind(&PedFollowNode::pedObjectCallback, this, std::placeholders::_1));
    }

private:
    //Parameter 
    int dist_long_level_max_;
    int dist_lat_level_max_;
    double t_dist_update_min_;
    double dist_long_per_level_;
    double dist_lat_per_level_;
    double d_t_cmd_max_;
    double dist_safety_long_;    
    double v_max_forward_;
    double v_max_reverse_; // absolute value      
    double a_limit_forward_incr_; // absolute value  
    double a_limit_reverse_incr_; // absolute value  
    double a_limit_decr_; // absolute value  
    double st_agl_limit_; // absolute value  
    double d_st_agl_limit_; // absolute value  
    std::string can_mode_;
    bool consider_gestures_;
    double controller_lat_lookahead_dist_;
    double controller_long_K_d_;
    double controller_long_K_v_;

    // Variables
    double dist_safety_long_min_ = 0.5; //m; minimum safty distance
    double a_limit_decr_min_ = 1.0; //m/s2; lowest value for maximum breaking, cannot go lower than this 
    int mode_use_ = 4; //vehicle mode in which driving is possible (all others: driving not possible)
    int mode_standby_ = 2; //vehicle mode for standby    
    double f_ws_ = 0.5/13.5142; //conversion factor wheelspeed rpm to vheicle speed m/s
    double wheelbase_ = 2.9; //m    
    State state_; 
    int dist_level_long_; 
    int dist_level_lat_;
    rclcpp::Time latest_distance_update_time_;
    rclcpp::Time latest_control_cmd_update_time_;
    double latest_vel_tar;
    double latest_st_agl_cmd_f_; 
    double latest_st_agl_cmd_r_; 
    Pedestrian latest_ped_obj_ = Pedestrian(); 
    int latest_mode_act_ = -1; //actual vehicle mode (init with impossible value)
    double v_act_ = 0; //actual vehicle speed
    
    // Publishers
    rclcpp::Publisher<kfzbshtl_msgs::msg::StampedControlGuardCmdLongitudinal>::SharedPtr control_cmd_long_publisher_;
    rclcpp::Publisher<kfzbshtl_msgs::msg::StampedControlGuardCmdLateral>::SharedPtr control_cmd_lat_publisher_;
    
    // Subscribers
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr ped_object_subscriber_;
    rclcpp::Subscription<kfzbshtl_msgs::msg::StampedGesture>::SharedPtr ped_gesture_subscriber_;
    rclcpp::Subscription<kfzbshtl_msgs::msg::StampedControlGuardInfo>::SharedPtr control_guard_info_subscriber_;    
    rclcpp::Subscription<kfzbshtl_msgs::msg::DecodedCanMsg>::SharedPtr can_id1155_subscriber_;  
    rclcpp::Subscription<kfzbshtl_msgs::msg::DecodedCanMsg>::SharedPtr can_id451_subscriber_;  

    //Methods
    void changeState(const State newState) {
        if (newState != this->state_) {
            this->state_ = newState;
            RCLCPP_INFO(this->get_logger(), "State changed to %s", stateToString(newState).c_str());
        }        
    }

    void reset() {
        this->dist_level_long_ = 0; 
        this->dist_level_lat_ = 0;
        this->latest_distance_update_time_ = this->get_clock()->now();
        this->latest_vel_tar = 0.0;
        this->latest_st_agl_cmd_f_ = 0.0; 
        this->latest_st_agl_cmd_r_ = 0.0; 
    }

    bool is_driving_mode_ready_for_driving() {
        //ignore actual vehicle mode is use in virtual
        return (this->latest_mode_act_==this->mode_use_ || this->can_mode_!="can");
    }
    
    // Subscriber callbacks
    void can_id1155_callback(const kfzbshtl_msgs::msg::DecodedCanMsg::SharedPtr msg) { //UCVE_M_ModeStates
        std::vector<std::string> sgnl_name = msg->sgnl_name;
        std::vector<float> sgnl_val = msg->sgnl_val;
        this->latest_mode_act_ = sgnl_val.at(std::find(sgnl_name.begin(), sgnl_name.end(), "VehicleMode")-sgnl_name.begin());
    }

    void can_id451_callback(const kfzbshtl_msgs::msg::DecodedCanMsg::SharedPtr msg) { //UCVE_M_Chassis1
        std::vector<std::string> sgnl_name = msg->sgnl_name;
        std::vector<float> sgnl_val = msg->sgnl_val;
        double ws_rr = sgnl_val.at(std::find(sgnl_name.begin(), sgnl_name.end(), "WheelSpeedRR")-sgnl_name.begin());
        double ws_rl = sgnl_val.at(std::find(sgnl_name.begin(), sgnl_name.end(), "WheelSpeedRL")-sgnl_name.begin());
        this->v_act_ = 0.5*(ws_rr+ws_rl)*this->f_ws_;
    }

    void gestureCallback(const kfzbshtl_msgs::msg::StampedGesture::SharedPtr msg) {
        Gesture gesture = static_cast<Gesture>(msg->data);
        State state = this->state_;
        //change State based on gesture
        switch (gesture) {
            case Gesture::NONE:
                break;
            case Gesture::START:
                if (state == State::IDLE) {
                    RCLCPP_INFO(this->get_logger(), "Acting on gesture '%s'", gestureToString(gesture).c_str());
                    this->changeState(State::ACTIVE_FORWARD);                    
                };
                break;
            case Gesture::STOP:
                if (state == State::ACTIVE_FORWARD || state == State::ACTIVE_REVERSE) {
                    RCLCPP_INFO(this->get_logger(), "Acting on gesture '%s'", gestureToString(gesture).c_str());
                    this->changeState(State::IDLE);
                };
                break;
            case Gesture::REVERSE:
                if (state == State::IDLE) {
                    RCLCPP_INFO(this->get_logger(), "Acting on gesture '%s'", gestureToString(gesture).c_str());
                    this->changeState(State::ACTIVE_REVERSE);
                };
                break;
            case Gesture::LESS_DISTANCE:
                if (state == State::IDLE || state == State::ACTIVE_FORWARD || state == State::ACTIVE_REVERSE) {
                    if ((this->get_clock()->now() - this->latest_distance_update_time_).seconds()>this->t_dist_update_min_) {
                        RCLCPP_INFO(this->get_logger(), "Acting on gesture '%s'", gestureToString(gesture).c_str());
                        int dist_level_long_temp = std::min(this->dist_level_long_-1, -1*dist_long_level_max_);
                        if (this->dist_safety_long_-dist_level_long_temp*this->dist_long_per_level_ >this->dist_safety_long_min_) { //only decrese long distance level, if minimum safety distance is respected
                            this->dist_level_long_= std::min(this->dist_level_long_-1, -1*dist_long_level_max_);
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Reducing longitudinal distance level would violate minimum safety distance of %.1fm and is therefore not conducted.", this->dist_safety_long_min_);
                        }                        
                        this->latest_distance_update_time_ = this->get_clock()->now();
                    }                    
                }                
                break;
            case Gesture::MORE_DISTANCE:
                if (state == State::IDLE || state == State::ACTIVE_FORWARD || state == State::ACTIVE_REVERSE) {
                    if ((this->get_clock()->now() - this->latest_distance_update_time_).seconds()>this->t_dist_update_min_) {
                        RCLCPP_INFO(this->get_logger(), "Acting on gesture '%s'", gestureToString(gesture).c_str());
                        this->dist_level_long_= std::max(this->dist_level_long_+1, dist_long_level_max_);
                        this->latest_distance_update_time_ = this->get_clock()->now();
                    }
                }
                break;
            case Gesture::MORE_LEFT:
                if (state == State::IDLE || state == State::ACTIVE_FORWARD || state == State::ACTIVE_REVERSE) {
                    if ((this->get_clock()->now() - this->latest_distance_update_time_).seconds()>this->t_dist_update_min_) {
                        RCLCPP_INFO(this->get_logger(), "Acting on gesture '%s'", gestureToString(gesture).c_str());
                        this->dist_level_lat_= std::max(this->dist_level_lat_+1, dist_lat_level_max_);
                        this->latest_distance_update_time_ = this->get_clock()->now();
                    }
                }
                break;
            case Gesture::MORE_RIGHT:                
                if (state == State::IDLE || state == State::ACTIVE_FORWARD || state == State::ACTIVE_REVERSE) {
                    if ((this->get_clock()->now() - this->latest_distance_update_time_).seconds()>this->t_dist_update_min_) {
                        RCLCPP_INFO(this->get_logger(), "Acting on gesture '%s'", gestureToString(gesture).c_str());
                        this->dist_level_lat_= std::min(this->dist_level_lat_-1, -1*dist_lat_level_max_);
                        this->latest_distance_update_time_ = this->get_clock()->now();
                    }
                }
                break;
        }
    }


    void controlGuardInfoCallback(const kfzbshtl_msgs::msg::StampedControlGuardInfo::SharedPtr msg) {
        // StampedControlGuardInfo.msg:
        // std_msgs/Header header
        // bool automatic_mode_active
        // int16 automatic_mode_longitudinal
        // int16 automatic_mode_lateral   
        if (msg->automatic_mode_active) {
            if (state_== State::OFF) {
                if (this->consider_gestures_ == false) { // go directly to active if no gestures are used; otherwise use gesture START for that
                    this->changeState(State::ACTIVE_FORWARD);
                } else {
                    this->changeState(State::IDLE);
                }                
            }
        } else {
            this->changeState(State::OFF);
            this->reset();
        }
    }


    void pedObjectCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
        // //Detection3DArray
        // std_msgs/Header header
        // Detection3D[] detections

        // //Detection3D
        // std_msgs/Header header
        // ObjectHypothesisWithPose[] results
        //      ObjectHypothesis hypothesis
        //          String class_id
        //          float64 score
        //      geometry_msgs/PoseWithCovariance pose
        //          geometry_msgs/Pose pose
        //              geometry_msgs/Point position
        //                  float64 x
        //                  float64 y
        //                  float64 z
        //              geometry_msgs/Quaternion orientation
        //                  float64 x
        //                  float64 y
        //                  float64 z
        //                  float64 w
        //          float64[36] covariance
        // BoundingBox3D bbox
        //      geometry_msgs/Pose center
        //          geometry_msgs/Point position 
        //              float64 x
        //              float64 y
        //              float64 z
        //          geometry_msgs/Quaternion orientation
        //              float64 x
        //              float64 y
        //              float64 z
        //              float64 w
        //      geometry_msgs/Vector3 size
        //          float64 x
        //          float64 y
        //          float64 z
        // String id

        double vel_tar = 0.0; // m/s
        int mode_req = this->mode_standby_; //[0:"OFF (shutdown)", 2:"Standby", 4:"Use", 7:"Safety"]
        double steering_angle_front = 0; //deg
        double steering_angle_rear = 0; //deg
        rclcpp::Time time_now = this->get_clock()->now();
        State state = this->state_;        
        
        // extract pedestrian info from msg
        Pedestrian ped_obj = Pedestrian();
        if (msg->detections.empty() == false) { // a valid object was provided
            ped_obj = Pedestrian(
                msg->detections[0].results[0].pose.pose.position.x,
                msg->detections[0].results[0].pose.pose.position.y,
                msg->detections[0].id
                );
        } 

        //calculate time diff since latest cmd update         
        double dt = (time_now-this->latest_control_cmd_update_time_).seconds();
        if (dt >this->d_t_cmd_max_) {
            latest_ped_obj_ = Pedestrian(); //forget last object if it was recieved a long time ago
            RCLCPP_INFO(this->get_logger(), "Pedestrian object timed out due to slow update");
        }
        if (latest_ped_obj_.getID()!=ped_obj.getID()) {
            RCLCPP_INFO(this->get_logger(), "Pedestrian object ID changed from '%s' to '%s'", latest_ped_obj_.getID().c_str(), ped_obj.getID().c_str());         
        }

        if ((state == State::IDLE || state == State::ACTIVE_FORWARD || state == State::ACTIVE_REVERSE) && 
            (ped_obj.getID() != "" && latest_ped_obj_.getID()==ped_obj.getID())) {                       
            //// logitudinal controller          
            mode_req = this->mode_use_;   
            double offset_long = this->dist_level_long_* this->dist_long_per_level_ + this->dist_safety_long_;
            double dist_rel = sqrt(ped_obj.getX() *ped_obj.getX() + ped_obj.getY()*ped_obj.getY());
            double dist_rel_prev = sqrt(latest_ped_obj_.getX() *latest_ped_obj_.getX() + latest_ped_obj_.getY()*latest_ped_obj_.getY());
            double v_rel = (dist_rel-dist_rel_prev)/dt; //limit this value?
            double v_tar_controller = (dist_rel-offset_long)*this->controller_long_K_d_ + v_rel*this->controller_long_K_v_ + v_act_;
            
            // limit change in velocity per time
            double d_v_cmd = v_tar_controller-latest_vel_tar;
            double d_v_cmd_limit_incr = 0.0; //m/s²
            double d_v_cmd_limit_decr = this->a_limit_decr_*dt; //m/s²
            if (state == State::ACTIVE_FORWARD) {
                d_v_cmd_limit_incr = this->a_limit_forward_incr_*dt;
            } else if (state == State::ACTIVE_REVERSE) {
                d_v_cmd_limit_incr = this->a_limit_reverse_incr_*dt;
            } 
            if (latest_vel_tar>=0) { 
                if (d_v_cmd>d_v_cmd_limit_incr) { //change away from zero (more positive)
                    d_v_cmd = d_v_cmd_limit_incr; 
                } else if (d_v_cmd<-d_v_cmd_limit_decr) { // change towards zero
                    d_v_cmd = -d_v_cmd_limit_decr;
                }
            } else {
                if (d_v_cmd<-d_v_cmd_limit_incr) { //change away from zero (more negative)
                    d_v_cmd = -d_v_cmd_limit_incr; 
                } else if (d_v_cmd>d_v_cmd_limit_decr) { // change towards zero
                    d_v_cmd = d_v_cmd_limit_decr;
                }
            }
            vel_tar = latest_vel_tar + d_v_cmd;

            // limit max value            
            if (state == State::ACTIVE_FORWARD) {
                vel_tar = std::max(std::min(vel_tar, this->v_max_forward_), 0.0); //forward drive -> vel>=0; do not go faster than defined (negative) maximum forward velocity 
            } else if (state == State::ACTIVE_REVERSE) {
                vel_tar = std::min(0.0, std::max(vel_tar, -this->v_max_reverse_)); // reverse drive -> vel<=0; do not go faster than defined (negative) maximum reverse velocity
            } else {
                vel_tar = 0.0; 
            } 
            
            //check vehicle driving mode. Only send v!=0, if vehicle is in driving mode
            if (this->is_driving_mode_ready_for_driving() == false) {
                vel_tar = 0.0; 
            }
            vel_tar = 0.0; //TODO delete this line after implementation of longitudinal controller            
            
            //// lateral controler (Pure Persuit)
            double offset_lat = this->dist_level_lat_* this->dist_lat_per_level_;
            double e_lat = ped_obj.getY()-offset_lat;                        
            if (state == State::ACTIVE_FORWARD) {
                double alpha = atan2(e_lat, ped_obj.getX()); //TODO use distance to rear axle instead of ped_obj.getX()
                steering_angle_front = atan(2*this->wheelbase_*sin(alpha)/this->controller_lat_lookahead_dist_);
                steering_angle_rear = 0.0; 
            } else if(state == State::ACTIVE_REVERSE) {
                double alpha = atan2(e_lat, ped_obj.getX()); //TODO use distance to front axle instead of ped_obj.getX()
                steering_angle_front = 0.0; 
                steering_angle_rear = -1*atan(2*this->wheelbase_*sin(alpha)/this->controller_lat_lookahead_dist_);
            } else {
                steering_angle_front = 0.0;
                steering_angle_rear = 0.0;
            }
            //limit change in steering angle per time
            double d_st_agl_cmd_max = this->d_st_agl_limit_*dt;
            double d_st_agl_cmd_f = steering_angle_front - this->latest_st_agl_cmd_f_;
            if (d_st_agl_cmd_f > d_st_agl_cmd_max) {
                steering_angle_front = this->latest_st_agl_cmd_r_ + d_st_agl_cmd_max;
            } else if (d_st_agl_cmd_f < -d_st_agl_cmd_max) {
                steering_angle_front = this->latest_st_agl_cmd_r_ - d_st_agl_cmd_max;
            }
            double d_st_agl_cmd_r = steering_angle_rear - this->latest_st_agl_cmd_r_; 
            if (d_st_agl_cmd_r > d_st_agl_cmd_max) {
                steering_angle_rear = this->latest_st_agl_cmd_r_ + d_st_agl_cmd_max;
            } else if (d_st_agl_cmd_r < -d_st_agl_cmd_max) {
                steering_angle_rear = this->latest_st_agl_cmd_r_ - d_st_agl_cmd_max;
            }
            //limit max output
            if (steering_angle_front > this->st_agl_limit_) {
                steering_angle_front = this->st_agl_limit_;
            } else if (steering_angle_front < -this->st_agl_limit_) {
                steering_angle_front = -this->st_agl_limit_;
            }
            if (steering_angle_rear > this->st_agl_limit_) {
                steering_angle_rear = this->st_agl_limit_;
            } else if (steering_angle_rear < -this->st_agl_limit_) {
                steering_angle_rear = -this->st_agl_limit_;
            }
            steering_angle_front = 0.0; //TODO delete this line after implementation of lateral controller
            steering_angle_rear = 0.0; //TODO delete this line after implementation of lateral controller          
        }

        //// store and publish
        // store and update time
        this->latest_vel_tar = vel_tar;
        this->latest_st_agl_cmd_f_ = steering_angle_front; 
        this->latest_st_agl_cmd_r_ = steering_angle_rear;         
        this->latest_ped_obj_ = ped_obj;
        this->latest_control_cmd_update_time_ = time_now;

        // form and publish control messages
        auto msg_cmd_long = kfzbshtl_msgs::msg::StampedControlGuardCmdLongitudinal();
        msg_cmd_long.header = msg->header;
        msg_cmd_long.vel = vel_tar; 
        msg_cmd_long.mode_req = mode_req;

        auto msg_cmd_lat = kfzbshtl_msgs::msg::StampedControlGuardCmdLateral();
        msg_cmd_lat.header = msg->header;
        msg_cmd_lat.steering_angle_front = steering_angle_front; //deg
        msg_cmd_lat.steering_angle_rear = steering_angle_rear; //deg

        control_cmd_long_publisher_->publish(msg_cmd_long);
        control_cmd_lat_publisher_->publish(msg_cmd_lat);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PedFollowNode>());
    rclcpp::shutdown();
    return 0;
}
