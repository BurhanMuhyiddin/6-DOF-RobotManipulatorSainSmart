/*
sudo chmod a+rw /dev/ttyACM0
rosrun rosserial_python serial_node.py /dev/ttyACM0
*/

#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <mrm_description/Floats_array.h>
#include <Servo.h> 

ros::NodeHandle  nh;

int cur_pos[5]={0, 0 ,0, 0, 0};

Servo servo0, servo1, servo2, servo3, servo4;

float readservo0=0,readservo1=0,readservo2=0, readservo3=0, readservo4=0, readservo5=0;

void rotate_servo(int servo,int new_pos,int cur_pos,int dir)
{
  int pos = 0;
  
  if (servo==0)
  {
    if (new_pos < 0)
      new_pos = 0;
    else if(new_pos > 180)
      new_pos = 180;
    if (dir == 1)
    {
      for(pos=cur_pos;pos<= new_pos;pos += 1)
      {
        servo0.write(pos);
        delay(10);
      }      
    }
    else if(dir == -1)
    {
      for(pos=cur_pos;pos>= new_pos;pos -= 1)
      {
        servo0.write(pos);
        delay(10);
      }      
    }
  }
  if (servo==1)
  {
    if (new_pos<25)
      new_pos=25;
    else if(new_pos > 155)
    {
      //nh.loginfo("Limiting Servo2 to 155");
      new_pos=155;
    }
    
    if (dir == 1)
    {
      for(pos=cur_pos;pos<= new_pos;pos += 1)
      {
        servo1.write(pos);
        delay(10);
      }      
    }
    else if(dir == -1)
    {
      for(pos=cur_pos;pos>= new_pos;pos -= 1)
      {
        servo1.write(pos);
        delay(10);
      }      
    }
  }
  if (servo==2)
  {
    if (new_pos<15)
      new_pos=15;
    else if (new_pos > 85)
      new_pos=85;
      
    if (dir == 1)
    {
      for(pos=cur_pos;pos<= new_pos;pos += 1)
      {
        servo2.write(pos);
        delay(10);
      }      
    }
    else if(dir == -1)
    {
      for(pos=cur_pos;pos>= new_pos;pos -= 1)
      {
        servo2.write(pos);
        delay(10);
      }      
    }
  }
  if(servo == 3)
  {
    if(new_pos < 0)
      new_pos = 0;
    else if(new_pos > 180)
      new_pos = 180;

    if (dir == 1)
    {
      for(pos=cur_pos;pos<= new_pos;pos += 1)
      {
        servo3.write(pos);
        delay(10);
      }      
    }
    else if(dir == -1)
    {
      for(pos=cur_pos;pos>= new_pos;pos -= 1)
      {
        servo3.write(pos);
        delay(10);
      }      
    }
  }

  if(servo == 4)
  {
    if(new_pos < 0)
      new_pos = 0;
    else if(new_pos > 140)
      new_pos = 140;

    if (dir == 1)
    {
      for(pos=cur_pos;pos<= new_pos;pos += 1)
      {
        servo4.write(pos);
        delay(10);
      }      
    }
    else if(dir == -1)
    {
      for(pos=cur_pos;pos>= new_pos;pos -= 1)
      {
        servo4.write(pos);
        delay(10);
      }      
    }
  }
}

void servo_cb( const rospy_tutorials::Floats& cmd_msg){
  //nh.loginfo("Command Received");
  
  int new_pos[5]={cmd_msg.data[0],cmd_msg.data[1],cmd_msg.data[2], cmd_msg.data[3], cmd_msg.data[4]};
  int i=0;
  
  for(i=0;i<5;i++)
  {
    if (new_pos[i]>cur_pos[i])
    {
      rotate_servo(i,new_pos[i],cur_pos[i],1);
      cur_pos[i]=new_pos[i];
    }
    else if(new_pos[i]<cur_pos[i])
    {
      rotate_servo(i,new_pos[i],cur_pos[i],-1);
      cur_pos[i]=new_pos[i];
    } 
  }
}

/*
void callback(const mrm_description::Floats_array::Request & req, mrm_description::Floats_array::Response & res)
{
  // Simulate function running for a non-deterministic amount of time
  res.res_length=5;
  
  readservo0=0;
  readservo1=25;
  readservo2=25;
  readservo3=25;
  readservo4=25;

  res.res[0]=readservo0;
  res.res[1]=readservo1;
  res.res[2]=readservo2;
  res.res[3]=readservo3;
  res.res[4]=readservo4;
}
*/

ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino", servo_cb);
//ros::ServiceServer<mrm_description::Floats_array::Request, mrm_description::Floats_array::Response> server("/read_joint_state",&callback);

void setup() {
  // put your setup code here, to run once:
  //str_msg.data = "Hello";
  randomSeed(analogRead(0));
  
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertiseService(server);
  //nh.advertise(chatter);
  //str_msg.data = "Service created successfully";
  //chatter.publish( &str_msg );

  servo0.attach(8);
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  servo4.attach(12);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}
