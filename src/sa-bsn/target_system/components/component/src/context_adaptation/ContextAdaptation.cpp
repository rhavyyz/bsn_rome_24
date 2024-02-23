#include "component/context_adaptation/ContextAdaptation.hpp"
#include "ros/ros.h"
#include<string>

std::map<std::string, float> ContextAdaptation::states_avarages = {{"Sitting", 91}, {"Running", 110}, {"Sleeping", 72}};
std::map<std::string, std::vector<ContextAdaptation::hr_range>> ContextAdaptation::states_ranges = { 
    {"Sitting", {{85,97}, 
                 {70,85}, {97,115}, 
                 {0,70}, {115,300}}}, 
    {"Running", {{104,116}, 
                 {90,104}, {116,131}, 
                 {0,90}, {131,300}}}, 
    {"Sleeping", {{66,78}, 
                 {52,66}, {78,93}, 
                 {0,52}, {93,300}}}

};


ContextAdaptation::ContextAdaptation(int  &argc, char **argv, std::string name) : ROSComponent(argc, argv, name) {}

ContextAdaptation::~ContextAdaptation() {}

void ContextAdaptation::setUp() {
    srand(time(NULL));
    float freq;
    nh.getParam("context_frequency", freq);
    ROS_INFO("Setting Up");
    ROS_INFO("Freq = %f", freq);
    rosComponentDescriptor.setFreq(freq);
}

// ContextAdaptation::hr_values = std::vector<int>();


void ContextAdaptation::body() {
    ros::Subscriber TargetSystemDataSub = nh.subscribe("TargetSystemData", 10, &ContextAdaptation::collect, this);
    ros::Rate loop_rate(rosComponentDescriptor.getFreq());
    while (ros::ok){
        ROS_INFO("Running");

        //

        monitor();

        //

        ros::spinOnce();
        loop_rate.sleep();            
    }   

    return;
}

float ContextAdaptation::getMovingAvarage() {
    if(n == 0)
        return 0;
    
    return sum/n;
}

void ContextAdaptation::tearDown() {
    // ROS_INFO("Tearing down");
}

void ContextAdaptation::collect(const messages::TargetSystemData::ConstPtr& msg) {
    float heart_rate, spo2;
    heart_rate = msg->ecg_data;
    spo2 = msg->oxi_data;
    curr_risk = msg->patient_status;

    hr_values.push_back(heart_rate);

    n++;
    right++;
    sum+= hr_values.back();

    if(n > max_n)
    {
        n--;
        sum -= hr_values[left];
        left++;
    }


    ROS_INFO("----------------");
    ROS_INFO(("Heart Rate " + std::to_string(heart_rate)).c_str());
    ROS_INFO(("Moving Avarage "  + std::to_string(getMovingAvarage())).c_str());
    ROS_INFO(("Current state " + current_state).c_str());

}

bool ContextAdaptation::setRisks(std::string vitalSign, float* lowRisk, float* MidRisk0, float* MidRisk1, float* highRisk0, float* highRisk1) {
    ros::ServiceClient client = nh.serviceClient<services::PatientAdapt>("contextAdapt");
    services::PatientAdapt srv;
    srv.request.vitalSign = vitalSign;
    srv.request.lowRisk_floor   = lowRisk[0];
    srv.request.lowRisk_ceil    = lowRisk[1];
    srv.request.MidRisk0_floor  = MidRisk0[0];
    srv.request.MidRisk0_ceil   = MidRisk0[1];
    srv.request.MidRisk1_floor  = MidRisk1[0];
    srv.request.MidRisk1_ceil   = MidRisk1[1];
    srv.request.highRisk0_floor = highRisk0[0];
    srv.request.highRisk0_ceil  = highRisk0[1];
    srv.request.highRisk1_floor = highRisk1[0];
    srv.request.highRisk1_ceil  = highRisk1[1];

    if (client.call(srv)) {
        if(srv.response.set) {
            ROS_INFO("Params %s set successfully", vitalSign.c_str());
            return true;
        }
        else ROS_INFO("Could not write params");
    } else {
        ROS_INFO("Service is not answering");
    }
    return false;
}

void ContextAdaptation::monitor() {

    // The trigger to a possible adaptation is a situation of the heart rate avarage 
    // of the last minute is in a mid or high risk zone 

    if(&& (states_ranges[current_state][1].ceil > getMovingAvarage() || 
       states_ranges[current_state][2].floor < getMovingAvarage()))    
    {
        analyze();
    }

}

void ContextAdaptation::analyze() {

    // The adaptation consists basicly on keeping track on the avarage heart rate
    // of the last minute and verify which state whould be more compatible with 
    // this avarage. If this state is not selected right now the adaptation 
    // goes on to switch the current state

    int moving_avarage = getMovingAvarage();

    std::string state = "";
    double minimun_difference = 10000000000;

    for(auto el : states_avarages)
    {
        auto state_name = el.first;
        auto avarage = el.second;

        float difference = moving_avarage - avarage;

        if(difference < 0)
            difference *= -1;

        if(difference < minimun_difference)
        {
            state = state_name;
            minimun_difference = difference;
        }
    }

    if(state != current_state)
        execute(state);
}

void ContextAdaptation::plan() {

}

void ContextAdaptation::execute(const std::string & state) {

    auto ranges = states_ranges[state];

    ROS_INFO("------------------");
    ROS_INFO(("Changing from " + current_state + " to " + state).c_str());
    ROS_INFO("------------------");

    float lowRisk[2] = {ranges[0].floor, ranges[0].ceil};
    float MidRisk0[2] = {ranges[2].floor, ranges[1].ceil};
    float MidRisk1[2] = {ranges[2].floor, ranges[2].ceil};
    float highRisk0[2] = {ranges[3].floor, ranges[3].ceil};
    float highRisk1[2] = {ranges[4].floor, ranges[4].ceil};

    current_state = state;

    setRisks("heart_rate", lowRisk,  MidRisk0,  MidRisk1,  highRisk0, highRisk1);
}