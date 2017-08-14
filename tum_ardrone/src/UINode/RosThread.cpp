 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "RosThread.h"
#include <unistd.h>
#include "cvd/thread.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "tum_ardrone_gui.h"
#include "stdio.h"
#include "std_msgs/Empty.h"
#include "cmath"


pthread_mutex_t RosThread::send_CS = PTHREAD_MUTEX_INITIALIZER;
RosThread::RosThread():it_(nh_)
{
	gui = NULL;
	navdataCount = velCount = dronePoseCount = joyCount = velCount100ms = 0;
	keepRunning = true;
    keepTracking = false;
	lastJoyControlSent = ControlCommand(0,0,0,0);
    lastL1Pressed = lastR1Pressed = false;
    pid_fb.kp = 0.0006;
    pid_fb.kd = 0.0015;
    pid_fb.ki = 0.0001;
    pid_fb.prev_error = 0;
    pid_fb.int_error = 0;
}

RosThread::~RosThread(void)
{

}

void RosThread::startSystem()
{
	keepRunning = true;
	start();
}

void RosThread::stopSystem()
{
	keepRunning = false;
	join();
}

void RosThread::landCb(std_msgs::EmptyConstPtr)
{
	gui->addLogLine("sent: LAND");
}
void RosThread::toggleStateCb(std_msgs::EmptyConstPtr)
{
	gui->addLogLine("sent: ToggleState");
}
void RosThread::takeoffCb(std_msgs::EmptyConstPtr)
{
	gui->addLogLine("sent: Takeoff");
}

void RosThread::droneposeCb(const tum_ardrone::filter_stateConstPtr statePtr)
{
	dronePoseCount++;
}
void RosThread::velCb(const geometry_msgs::TwistConstPtr vel)
{
    velCount++;
    velCount100ms++;
}
void RosThread::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	if(navdataCount%10==0)
	{
		char buf[200];
		snprintf(buf,200,"Motors: %f %f %f %f",
				(float)navdataPtr->motor1,
				(float)navdataPtr->motor2,
				(float)navdataPtr->motor3,
				(float)navdataPtr->motor4);
		gui->setMotorSpeeds(std::string(buf));

//        if(navdataPtr->altd <200 && keepTracking){
//            sendControlToDrone(ControlCommand(0,0,0,-1));
//            qDebug(">>>>>>>>>>>>>>>>>>>>>>");
//        }
    }
	navdataCount++;
}

void RosThread::joyCb(const sensor_msgs::JoyConstPtr joy_msg)
{
    joyCount++;

    if (joy_msg->axes.size() < 4) {
        ROS_WARN_ONCE("Error: Non-compatible Joystick!");
        return;
    }

    // Avoid crashes if non-ps3 joystick is being used
    short unsigned int actiavte_index = (joy_msg->buttons.size() > 11) ? 11 : 1;

	// if not controlling: start controlling if sth. is pressed (!)
    bool justStartedControlling = false;
	if(gui->currentControlSource != CONTROL_JOY)
	{
		if(		joy_msg->axes[0] > 0.1 ||  joy_msg->axes[0] < -0.1 ||
				joy_msg->axes[1] > 0.1 ||  joy_msg->axes[1] < -0.1 ||
				joy_msg->axes[2] > 0.1 ||  joy_msg->axes[2] < -0.1 ||
				joy_msg->axes[3] > 0.1 ||  joy_msg->axes[3] < -0.1 ||
                joy_msg->buttons.at(actiavte_index))
		{
			gui->setControlSource(CONTROL_JOY);
			justStartedControlling = true;
		}
	}

	// are we actually controlling with the Joystick?
	if(justStartedControlling || gui->currentControlSource == CONTROL_JOY)
	{
		ControlCommand c;
		c.yaw = -joy_msg->axes[2];
		c.gaz = joy_msg->axes[3];
		c.roll = -joy_msg->axes[0];
		c.pitch = -joy_msg->axes[1];

		sendControlToDrone(c);
		lastJoyControlSent = c;

        if(!lastL1Pressed && joy_msg->buttons.at(actiavte_index - 1))
			sendTakeoff();
        if(lastL1Pressed && !joy_msg->buttons.at(actiavte_index - 1))
			sendLand();

        if(!lastR1Pressed && joy_msg->buttons.at(actiavte_index))
			sendToggleState();

	}
    lastL1Pressed =joy_msg->buttons.at(actiavte_index - 1);
    lastR1Pressed = joy_msg->buttons.at(actiavte_index);
}


void RosThread::comCb(const std_msgs::StringConstPtr str)
{
	if(str->data.substr(0,2) == "u ")
	{
		if(str->data.substr(0,4) == "u l ")
			gui->addLogLine(str->data.substr(4,str->data.length()-4));

		else if(str->data.substr(0,4) == "u c ")
			gui->setAutopilotInfo(str->data.substr(4,str->data.length()-4));

		else if(str->data.substr(0,4) == "u s ")
			gui->setStateestimationInfo(str->data.substr(4,str->data.length()-4));
	}
}




void RosThread::run()
{
	std::cout << "Starting ROS Thread" << std::endl;

    vel_pub	   = nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("cmd_vel"),1);
    vel_sub	   = nh_.subscribe(nh_.resolveName("cmd_vel"),50, &RosThread::velCb, this);

    tum_ardrone_pub	   = nh_.advertise<std_msgs::String>(nh_.resolveName("tum_ardrone/com"),50);
    tum_ardrone_sub	   = nh_.subscribe(nh_.resolveName("tum_ardrone/com"),50, &RosThread::comCb, this);


    dronepose_sub	   = nh_.subscribe(nh_.resolveName("ardrone/predictedPose"),50, &RosThread::droneposeCb, this);
    navdata_sub	   = nh_.subscribe(nh_.resolveName("ardrone/navdata"),50, &RosThread::navdataCb, this);
    joy_sub	   = nh_.subscribe(nh_.resolveName("joy"),50, &RosThread::joyCb, this);

    takeoff_pub	   = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/takeoff"),1);
    land_pub	   = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/land"),1);
    toggleState_pub	   = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/reset"),1);

    takeoff_sub	   = nh_.subscribe(nh_.resolveName("ardrone/takeoff"),1, &RosThread::takeoffCb, this);
    land_sub	   = nh_.subscribe(nh_.resolveName("ardrone/land"),1, &RosThread::landCb, this);
    toggleState_sub	   = nh_.subscribe(nh_.resolveName("ardrone/reset"),1, &RosThread::toggleStateCb, this);

    toggleCam_srv        = nh_.serviceClient<std_srvs::Empty>(nh_.resolveName("ardrone/togglecam"),1);
    flattrim_srv         = nh_.serviceClient<std_srvs::Empty>(nh_.resolveName("ardrone/flattrim"),1);

    getPath_cli = nh_.serviceClient<droneTest::Num_srv>("/chatter_svr", 1);
    getPath_sub = nh_.subscribe("/chatter", 1, &RosThread::path_tracking, this);

	ros::Time last = ros::Time::now();
	ros::Time lastHz = ros::Time::now();
	while(keepRunning && nh_.ok())
	{
		// spin for 100ms
		while((ros::Time::now() - last) < ros::Duration(0.1))
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1 - (ros::Time::now() - last).toSec()));
		last = ros::Time::now();

        // if nothing on /cmd_vel, repeat!
		if(velCount100ms == 0)
			switch(gui->currentControlSource)
			{
			case CONTROL_AUTO:
				sendControlToDrone(ControlCommand(0,0,0,0));
				break;
			case CONTROL_JOY:
				sendControlToDrone(lastJoyControlSent);
				break;
			case CONTROL_KB:
				sendControlToDrone(gui->calcKBControl());
				break;
			case CONTROL_NONE:
				sendControlToDrone(ControlCommand(0,0,0,0));
				break;
			}
		velCount100ms = 0;

		// if 1s passed: update Hz values
		if((ros::Time::now() - lastHz) > ros::Duration(1.0))
		{
			gui->setCounts(navdataCount, velCount, dronePoseCount, joyCount);
			navdataCount = velCount = dronePoseCount = joyCount = 0;
			lastHz = ros::Time::now();
		}
	}

	gui->closeWindow();
	if(nh_.ok()) ros::shutdown();
	std::cout << "Exiting ROS Thread (ros::shutdown() has been called)" << std::endl;
}


void RosThread::publishCommand(std::string c)
{
	std_msgs::String s;
	s.data = c.c_str();
	pthread_mutex_lock(&send_CS);
	tum_ardrone_pub.publish(s);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::sendControlToDrone(ControlCommand cmd)
{
	// TODO: check converstion (!)
	geometry_msgs::Twist cmdT;
	cmdT.angular.z = -cmd.yaw;
	cmdT.linear.z = cmd.gaz;
	cmdT.linear.x = -cmd.pitch;
	cmdT.linear.y = -cmd.roll;

	cmdT.angular.x = cmdT.angular.y = gui->useHovering ? 0 : 1;
    qDebug(">>>> %f\n", cmd.yaw);

	pthread_mutex_lock(&send_CS);
	vel_pub.publish(cmdT);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::sendLand()
{
	pthread_mutex_lock(&send_CS);
	land_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}
void RosThread::sendTakeoff()
{
	pthread_mutex_lock(&send_CS);
	takeoff_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}
void RosThread::sendToggleState()
{
	pthread_mutex_lock(&send_CS);
	toggleState_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}
void RosThread::sendToggleCam()
{
	pthread_mutex_lock(&send_CS);
	toggleCam_srv.call(toggleCam_srv_srvs);
	pthread_mutex_unlock(&send_CS);
}
void RosThread::sendFlattrim()
{
	pthread_mutex_lock(&send_CS);
	flattrim_srv.call(flattrim_srv_srvs);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::startTracking()
{
    keepTracking = true;
}

void RosThread::path_tracking(const droneTest::Num info)
{
    if(keepTracking)
    {
        if(fabs(info.theta) < 3.1415926/5.0)
        {
            //qDebug(">>>> distance: %f >>> theta: %f", info.distance, info.theta);
            droneTest::Num_srv stv;
            double delta = info.distance;
            double error = 0 -  delta;
            double offset = pid_update(&pid_fb, error);
            //if (offset < UMIN_N) offset = UMIN_N;
            offset *= 1.2;
            double offset_y = cos(info.theta) * offset;
            double offset_x = sin(info.theta) * offset;
            if(offset_y == 0 && offset_x == 0)
            {
                vx_now = -0.035*offset_x/sqrt(offset_x*offset_x + offset_y * offset_y) + vx_now * 0.04;
                vy_now = 0.035*offset_y/sqrt(offset_x*offset_x + offset_y * offset_y) + vy_now * 0.04;
                qDebug("offset %f distance %f theta %f offset_x %f offset_y %f vx %f vy %f\n",offset , info.distance , info.theta, offset_x, offset_y, vx_now, vy_now);
                if(fabs(delta) > 50 ){
                        sendControlToDrone(ControlCommand(vy_now, vx_now,0,0));
                } else{
                        sendControlToDrone(ControlCommand(-0.02, 0, 0, 0));
                        qDebug("vy -0.02");
                }
            }else{
                sendControlToDrone(ControlCommand(-0.02, 0, 0, 0));
                qDebug("vy -0.02");
            }

        } else{
            sendControlToDrone(ControlCommand(0, 0, 1.0 * fabs(info.theta)/3.1415926 * info.theta/fabs(info.theta), 0));
            qDebug("da %f\n", 1.0 * fabs(info.theta)/3.1415926 * info.theta/fabs(info.theta));
        }
    }
}

double RosThread::pid_update(struct pid *pid, double input)
{
    double error = input;
    pid->int_error += error;
    double derivate = error - pid->prev_error;
    pid->prev_error = error;
    double output = pid->kp * error + pid->kd * derivate + pid->ki * pid->int_error;
    return output;
}
