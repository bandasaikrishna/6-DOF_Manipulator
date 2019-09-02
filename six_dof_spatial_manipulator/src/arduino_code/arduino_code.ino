#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <Servo.h> 

ros::NodeHandle  nh;


int global_cur_pos[6]={0,0,0,0,0,0};

Servo servo1,servo2,servo3,servo4,servo5,servo6;


void rotate_servo(int servo,int new_pos,int cur_pos,int dir)
{
  int pos = 0;
  
  if (servo==0)
  {
    //nh.loginfo("Servo1");
    new_pos=new_pos-5;
    if (new_pos<(33))
      new_pos=33;
    if (new_pos>130)
      new_pos=130;
      
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
    //servo1.write(new_pos);
  }
  if (servo==1)
  {
    //nh.loginfo("Servo2"); //16=0
    new_pos=new_pos+5;
    if (new_pos < 16)
      new_pos=16;
    else if(new_pos > 158)
      new_pos=158;
    
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
    //servo2.write(new_pos);
  }
  if (servo==2)
  {
    //nh.loginfo("Servo3");
    new_pos=new_pos+10;
    if (new_pos<70)
      new_pos=70;
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
    //servo3.write(new_pos);
  }
  
  if (servo==3)
  {
    //nh.loginfo("Servo4");
    if (new_pos<10)
      new_pos=10;
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
    //servo4.write(new_pos);
  }
  if (servo==4)
  {
    //nh.loginfo("Servo5");
    if (new_pos<6)
      new_pos=6;
    if (dir == 1)
    {
      for(pos=cur_pos;pos<= new_pos;pos += 1)
      {
        servo5.write(pos);
        delay(10);
      }      
    }
    else if(dir == -1)
    {
      for(pos=cur_pos;pos>= new_pos;pos -= 1)
      {
        servo5.write(pos);
        delay(10);
      }      
    }
    //servo5.write(new_pos);
  }
  
  if (servo==5)
  {
    //nh.loginfo("Servo6");
    if (new_pos<6)
      new_pos=6;
    if (dir == 1)
    {
      for(pos=cur_pos;pos<= new_pos;pos += 1)
      {
        servo6.write(pos);
        delay(10);
      }      
    }
    else if(dir == -1)
    {
      for(pos=cur_pos;pos>= new_pos;pos -= 1)
      {
        servo6.write(pos);
        delay(10);
      }      
    }
    servo6.write(new_pos);
  }
  
  global_cur_pos[servo]=pos;
  
}


void servo_cb( const rospy_tutorials::Floats& cmd_msg){
  //nh.loginfo("Command Received ");
  
  int new_pos[6]={cmd_msg.data[0],cmd_msg.data[1],cmd_msg.data[2],cmd_msg.data[3],cmd_msg.data[4],cmd_msg.data[5]};
  int i=0;
  
  for(i=0;i<6;i++)
  {
    if (new_pos[i]>global_cur_pos[i])
    {
      rotate_servo(i,new_pos[i],global_cur_pos[i],1);
      //global_cur_pos[i]=new_pos[i];
        
    }
    else if(new_pos[i]<global_cur_pos[i])
    {
      rotate_servo(i,new_pos[i],global_cur_pos[i],-1);
      //global_cur_pos[i]=new_pos[i];
    }
    //rotate_servo(i,new_pos[i],global_cur_pos[i],1);
    //delay(100);
  }
  
}





ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino",servo_cb);


void setup(){

  nh.initNode();
  nh.subscribe(sub);
  
  servo1.attach(3); //attach it to pin 9
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9); //attach it to pin 9
  servo5.attach(10);
  servo6.attach(11);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(10);
  servo5.write(90);
  servo6.write(90);
  
  
}

void loop(){

  nh.spinOnce();
}

