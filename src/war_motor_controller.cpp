#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/Encoders.h"
#include "ras_arduino_msgs/PWM.h"
#include "math.h"

class MotorController
{
public:

    ros::NodeHandle n;
    ros::Subscriber encoder_subscriber;
    ros::Subscriber twist_subscriber;
    ros::Publisher pub;
    double pwmOut1,pwmOut2;
    double actualAngVelRight,actualAngVelLeft,desiredAngVelLeft,desiredAngVelRight;
    double pwm1, pwm2;
    double previousErrorLeft, previousErrorRight;
    double ItermL, ItermR, DtermL, DtermR;
    double KPR, KPL, KIL, KIR, KDL, KDR;
    double KUL, KUR, TUR, TUL;
    // Eliminate this when twist messages are ready;
    double twist_linVel_x, twist_angVel_x;
    
    MotorController()
    {
        n = ros::NodeHandle("~");

        motor_cotroller_ = NULL;
    }

    ~MotorController()
    {
        delete motor_cotroller_;
    }

    void GetTuningParameters()
    {
        n.getParam("KPR",KPR);
        n.getParam("KPL",KPL);
        n.getParam("KIL",KIL);
        n.getParam("KIR",KIR);
        n.getParam("KDR",KDR);
        n.getParam("KDL",KDL);
    }

    void GetZieglerNichlosParam()
    {
        n.getParam("KUR",KUR);
        n.getParam("KUR",KUL);
        n.getParam("KUR",TUR);
        n.getParam("KUR",TUL);

        KPR = 0.6*KUR;
        KPL = 0.6*KUL;
        KIL = 2*KPL/TUL;
        KIR = 2*KPR/TUR;
        KDL = KPL*TUL/8;
        KDR = KPR*TUR/8;
        

    }

    void GetVelocities()
    {
        n.getParam("linVel",twist_linVel_x);
        n.getParam("angVel",twist_angVel_x);
        double base=0.21;
        double r=0.05;
        //ROS_INFO("I heard: [%f]", twist_msg->angular.x);

        desiredAngVelRight = (twist_linVel_x+(base/2)*twist_angVel_x)/r;
        desiredAngVelLeft = (twist_linVel_x-(base/2)*twist_angVel_x)/r;
    } 
    

    void init()
    {
        motor_cotroller_ = new MotorController();
        encoder_subscriber = n.subscribe("/arduino/encoders", 1, &MotorController::encoderCallback,this);
        twist_subscriber = n.subscribe("/motor_controller/twist",1, &MotorController::twistCallback,this);
        pub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1);

    }
    
    // -------- CHECK WHICH MOTOR IS WHICH --------------------
    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &enc_msg)
    {
        double enc1=enc_msg->encoder1;
        double enc2=enc_msg->encoder2;
        double delta_enc1=enc_msg->delta_encoder1;
        double delta_enc2=enc_msg->delta_encoder2;
        double sampleTime=0.1;
        //ROS_INFO("I heard: [%d]", enc_msg->encoder1);
        actualAngVelLeft=(delta_enc2*(M_PI/180))/sampleTime;
        actualAngVelRight=(delta_enc1*(M_PI/180))/sampleTime;
        //ROS_INFO("Actual wR: [%f]",actualAngVelRight);
        //ROS_INFO("Actual wL: [%f]",actualAngVelLeft);
    }


    void twistCallback(const geometry_msgs::Twist::ConstPtr &twist_msg)
    {
        double twist_linVel_x=twist_msg->linear.x;
        double twist_angVel_x=twist_msg->angular.z;
        double base=0.21;
        double r=0.05;
        //ROS_INFO("I heard: [%f]", twist_msg->angular.x);

        desiredAngVelRight = (twist_linVel_x+(base/2)*twist_angVel_x)/r;
        desiredAngVelLeft = (twist_linVel_x-(base/2)*twist_angVel_x)/r;
        //ROS_INFO("wRight: [%f]\n wLeft: [%f]",desiredAngVelRight,desiredAngVelLeft);
    }
    

    // ----- CHECK PWM VALUES TO BE TO CORRECT MOTOR ---------------

    void controllerVelocities()
    {   
        // This controller uses the Ziegler-Nichols method
        // to tune the KP,KI,KD

        double dT=0.1;
        
        ras_arduino_msgs::PWM pwm_msg;
        //ROS_INFO("desiredAngVelRight: [%f]",desiredAngVelRight);
        //ROS_INFO("actualAngVelRight:[%f]",actualAngVelRight);
                
        // Controller for left wheel
        double errorLeft = (desiredAngVelLeft - actualAngVelLeft);
//        ROS_INFO("errorLeft: [%f]",errorLeft);
        ItermL = ItermL + KIL*errorLeft*dT;
        DtermL = (errorLeft - previousErrorLeft)/dT;
        pwm2 = pwm2 + KPL*errorLeft + ItermL + KDL*DtermL;


        // Controller for right wheel
        double errorRight = (desiredAngVelRight - actualAngVelRight);
//        ROS_INFO("errorRight: [%f]",errorRight);
        ItermR = ItermR + KIR*errorRight*dT;
        DtermR = (errorRight - previousErrorRight)/dT;   
        pwm1 = pwm1 + KPR*errorRight + ItermR + KDR*DtermR;
		
		if(pwm2>pwm1 && pwm2>150){
			double lowering=pwm2/150;
			pwm2= pwm2/lowering;
			pwm1= pwm1 / lowering;
		}		
		if(pwm1>pwm2 && pwm1>150){
			double lowering=pwm1/150;
			pwm2= pwm2/lowering;
			pwm1= pwm1 / lowering;
		}
		/*
		if (pwm2 > 150)
            pwm2 = 150;
        else if(pwm2 < -150)
            pwm2 =-150;
        if (pwm1 > 150)
            pwm1 = 150;
        else if(pwm1 < -150)
            pwm1 =-150;
	*/
        int pwmOut1 = (int)pwm1;
        int pwmOut2 = (int)pwm2;

        pwm_msg.PWM1 = pwmOut1;
        pwm_msg.PWM2 = pwmOut2;
        

        //ROS_INFO("ErrorR: [%f]",errorRight);
        //ROS_INFO("ErrorL:[%f]",errorLeft);
        pub.publish(pwm_msg);
      
        previousErrorLeft = errorLeft;
        previousErrorRight = errorRight;
    }
private:
    MotorController *motor_cotroller_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller");

    MotorController controller;
    
    controller.init();

    //----COMMENT OUT ONE FUNCTION TO GET PARAMETERS -----
    // GET KP,KI,KD from launch file
    controller.GetTuningParameters();

    //GET Ku,Tu to calculate KP,KI,KD (Ziegler-Nihcols)
    //controller.GetZieglerNichlosParam();
    // ---------------------------------------------------

    // This calls a linear velocity and angular velocity from launch file
    //controller.GetVelocities();

    ros::Rate loop_rate(10);

    
      while (controller.n.ok())
      {
        ros::spinOnce();
        controller.controllerVelocities();     
        loop_rate.sleep();
        
      } 

    return 0;

}
