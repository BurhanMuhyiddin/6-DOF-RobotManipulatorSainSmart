#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <mrm_description/Floats_array.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
float readservo0=0,readservo1=0,readservo2=0, readservo3=0, readservo4=0, readservo5=0;
std_msgs::String str_msg;

ros::Publisher chatter("chatter", &str_msg);

void callback(const mrm_description::Floats_array::Request & req, mrm_description::Floats_array::Response & res)
{
  // Simulate function running for a non-deterministic amount of time

  str_msg.data = "Callback started successfully";
  chatter.publish( &str_msg );

  res.res_length=5;
  
  readservo0=(random(175) + 5.0);
  readservo1=(random(180) + 5.0);
  readservo2=(random(180) + 5.0);
  readservo3=(random(180) + 5.0);
  readservo4=(random(180) + 5.0);

  res.res[0]=readservo0;
  res.res[1]=readservo1;
  res.res[2]=readservo2;
  res.res[3]=readservo3;
  res.res[4]=readservo4;

  str_msg.data = "Callback finished successfully";
  chatter.publish( &str_msg );
  

  //res.res[0]=-80.00000000000000000000000;
  //res.res[1]=-70.00000000000000000000000;
  //res.res[2]=-20.00000000000000000000000;
  //res.res[3]=-10.00000000000000000000000;
  //res.res[4]=-50.00000000000000000000000;

  str_msg.data = "Callback finished successfully";
  chatter.publish( &str_msg );
  
}


//ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino", servo_cb);

ros::ServiceServer<mrm_description::Floats_array::Request, mrm_description::Floats_array::Response> server("/read_joint_state",&callback);

void setup() {
  // put your setup code here, to run once:
  str_msg.data = "Hello";
  randomSeed(analogRead(0));
  
  nh.initNode();
  //nh.subscribe(sub);
  nh.advertiseService(server);
  nh.advertise(chatter);
  str_msg.data = "Service created successfully";
  chatter.publish( &str_msg );
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}
