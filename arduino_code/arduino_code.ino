#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <mrm_description/Floats_array.h>

ros::NodeHandle  nh;
float readservo0=0,readservo1=0,readservo2=0, readservo3=0, readservo4=0, readservo5=0;

void callback(const mrm_description::Floats_array::Request & req, mrm_description::Floats_array::Response & res)
{
  // Simulate function running for a non-deterministic amount of time
  

  res.res_length=6;
  
  readservo0=(random(175) + 5.0);
  readservo1=(random(180) + 5.0);
  readservo2=(random(180) + 5.0);
  readservo3=(random(180) + 5.0);
  readservo4=(random(180) + 5.0);
  readservo5=(random(180) + 5.0);
  
  res.res[0]=readservo0;
  res.res[1]=readservo1;
  res.res[2]=readservo2;
  res.res[3]=readservo3;
  res.res[4]=readservo4;
  res.res[5]=readservo5;
  
  return;
  
}


//ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino", servo_cb);
ros::ServiceServer<mrm_description::Floats_array::Request, mrm_description::Floats_array::Response> server("/read_joint_state",&callback);

void setup() {
  // put your setup code here, to run once:
  randomSeed(analogRead(0));
  
  nh.initNode();
  //nh.subscribe(sub);
  nh.advertiseService(server);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}
