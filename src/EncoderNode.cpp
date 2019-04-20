#include <iostream>
#include <vector>

#include <pigpio.h>
#include "rotary_encoder.hpp"

#include "ros/ros.h"
#include "rasp/EncoderCounts.h"

using namespace std;

// Channel A pins (yellow)
int Apins[4] = {27, 19, 16, 6};
// Channel B pins (green)
int Bpins[4] = {22, 26, 20, 13};

// Encoder Counts
vector<int16_t> counts(4, 0);
vector<re_decoder*> decoders;

void callback(int way, int gpio){
    for (int i = 0; i < 4; i++){
        if(gpio == Apins[i]){
            counts[i] += way;
            break;
        }
    }
}

int main(int argc, char **agrv){

    if (gpioInitialise() < 0) return 1;

    ROS_INFO("Encoder Node: Initializing four decoders");
    for (int i = 0; i < 4; i++){
        int Apin = Apins[i];
        int Bpin = Bpins[i];
        decoders.push_back(new re_decoder(Apin, Bpin, callback));
        printf("    Initializing decoder with Apin = %02d and Bpin = %02d.\n", decoders[i]->mygpioA, decoders[i]->mygpioB);
    }
    printf("%d decoders are ready.\n", decoders.size());


    ros::init(argc, agrv, "encodernode");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<rasp::EncoderCounts>("/motor_status/encoder_counts", 10);

    ros::Rate rate(1000);
    printf("Starting publishing loop\n");
    while(ros::ok()){
        rasp::EncoderCounts msg;
        msg.counts = counts;
        msg.header.stamp = ros::Time::now();
        
        pub.publish(msg);
        rate.sleep();
    }

    ROS_INFO("Closing four decoders\n");
    for (int i = 0; i < 4; i++){
        decoders[i]->re_cancel();
    }

    gpioTerminate();
    return 0;
}
