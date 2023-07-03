#include <ros/ros.h>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <thread>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

#define MaxLimit(a,max) ((a)>(max)?(max):(a))

#define MinLimit(a,min) ((a)<(min)?(min):(a))

#define LIMIT_VAL(a,min,max) ((a)<(min)?(min):((a)>(max)?(max):(a)))

class KeyboardReader
{
public:
  KeyboardReader()
#ifndef _WIN32
    : kfd(0)
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
#endif
  }
  void readOne(char * c)
  {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#else
    for(;;)
    {
      HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
      INPUT_RECORD buffer;
      DWORD events;
      PeekConsoleInput(handle, &buffer, 1, &events);
      if(events > 0)
      {
        ReadConsoleInput(handle, &buffer, 1, &events);
        if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
        {
          *c = KEYCODE_LEFT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
        {
          *c = KEYCODE_UP;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
        {
          *c = KEYCODE_RIGHT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
        {
          *c = KEYCODE_DOWN;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x42)
        {
          *c = KEYCODE_B;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43)
        {
          *c = KEYCODE_C;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44)
        {
          *c = KEYCODE_D;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45)
        {
          *c = KEYCODE_E;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46)
        {
          *c = KEYCODE_F;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x47)
        {
          *c = KEYCODE_G;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51)
        {
          *c = KEYCODE_Q;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52)
        {
          *c = KEYCODE_R;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54)
        {
          *c = KEYCODE_T;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x56)
        {
          *c = KEYCODE_V;
          return;
        }
      }
    }
#endif
  }
  void shutdown()
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }
private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

KeyboardReader input;

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();
  void lension();

private:

  
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
   char cc;
   bool getc;
  
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  //twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/chassis/ctrl_motion", 1);
  linear_=angular_=0;
  getc = false;
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}

void TeleopTurtle::lension()
{
  char c;
  bool dirty=false;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle. 'q' to quit.");

  ros::Rate r(50);
  while(ros::ok())
  {
    if(getc == true)
    {
       switch(cc)
      {
        case KEYCODE_LEFT:
          ROS_DEBUG("LEFT");
          // angular_ = 1.0;
          //angular_ ++;
          angular_ = angular_+0.2;
          dirty = true;
          break;
        case KEYCODE_RIGHT:
          ROS_DEBUG("RIGHT");
          //angular_ = -1.0;
          // angular_ --;
          angular_ =angular_-0.2;
          dirty = true;
          break;
        case KEYCODE_UP:
          ROS_DEBUG("UP");
          //linear_ = 1.0;
          // linear_ ++;
          linear_ =linear_+0.2;
          dirty = true;
          break;
        case KEYCODE_DOWN:
          ROS_DEBUG("DOWN");
          //linear_ = -1.0;
          // linear_ --;
          linear_ =linear_-0.2;
          dirty = true;
          dirty = true;
          break;
        case KEYCODE_Q:
          ROS_DEBUG("quit");
          return;
      }

      getc = false;
    }
   
    angular_ = LIMIT_VAL(angular_,-1,1);
    linear_ = LIMIT_VAL(linear_,-1,1);

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    // if(dirty ==true)
     if(1)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }



    r.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);
  std::thread  loop(&TeleopTurtle::lension,&teleop_turtle);
  teleop_turtle.keyLoop();
 
  loop.join();
  quit(0);
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
  // char c;
  bool dirty=false;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle. 'q' to quit.");


  for(;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&cc);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return;
    }

    // linear_=angular_=0;
    printf("value: 0x%02X\n", cc);
    getc =  true;
  
  }


  return;
}



