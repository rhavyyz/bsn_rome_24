#ifndef CONTEXTAPAPTATION_HPP
#define CONTEXTAPAPTATION_HPP

#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <vector>
#include <map>

#include "archlib/ROSComponent.hpp"
#include "messages/TargetSystemData.h"
#include "services/PatientAdapt.h"

class ContextAdaptation : public arch::ROSComponent {
    public:
        ContextAdaptation(int &argc, char **argv, std::string name);
        ~ContextAdaptation();

        void setUp();
        void tearDown();
        void body();

        float getMovingAvarage();

        struct hr_range
        {
            float floor, ceil;
        };


        static std::map<std::string, float> states_avarages;// = {{"Sitting", 91}, {"Running", 110}, {"Sleeping", 72}};
        static std::map<std::string, std::vector<hr_range>> states_ranges;// = { {"Sitting", {{85,97}, {70,85}, {97,115}, {0,70}, {115,300}}}, {"Running", {}}, {"Sleeping", {}}};


    private:

        void collect(const messages::TargetSystemData::ConstPtr& msg);
        ros::NodeHandle nh;

        // Fields to calculate the moving avarage value
        std::vector<int> hr_values;
        int max_n = 60;
        int n = 0;
        int left =0;
        int right = 0;
        float sum = 0;
        float curr_risk = 0;
        std::string current_state = "Sitting";



        


        bool setRisks(std::string vitalSign,float* lowRisk, float* MidRisk0, float* MidRisk1, float* highRisk0, float* highRisk1);

        void monitor();
        void analyze();
        void plan();
        void execute(const std::string & state);


};


#endif