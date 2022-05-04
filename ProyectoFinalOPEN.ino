/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
* 
0.020, 0.032, 0.083, -0.035, 0.000
-0.545, 0.621, -0.520, 1.385, 0.000
-0.525, 0.624, -0.523, 1.394, -0.006
0.091, -0.216, -0.457, 1.649, -0.006
1.431, -0.518, 0.561, 1.436, -0.006
1.486, -0.430, 0.834, 1.091, -0.006
1.485, -0.393, 0.828, 1.131, 0.004
0.106, -0.061, 0.006, 0.034, 0.006

*******************************************************************************/

/* Authors: Will Son */

#include <open_manipulator_libs.h>
#include "Arduino.h"

OpenManipulator open_manipulator;
double control_time = 0.010;  //default control frequency (100Hz)
double present_time = 0.0;
double previous_time = 0.0;
bool platform_state = true;
bool start_motion_flag = false;
bool stop_motion_flag = false;
bool teaching_mode_flag = false;
uint8_t motion_cnt[] = {0};
uint8_t motion_index = 0;
uint8_t motion_number = 0;

std::vector<JointValue> present_position;
std::vector<JointValue> gripper_position;
std::vector<JointValue> saved_teaching_pose;

void setup()
{
  Serial.begin(57600);
  while (!Serial);  // Wait until Serial is Opened

  open_manipulator.setOpenManipulatorCustomJointId(11, 12, 13, 14, 15); // ID 11, 12, 13, 14, 15 is default
  open_manipulator.initOpenManipulator(platform_state);
  Serial.println("OpenManipulator Init Begin");

  initDemo();
  
  Serial.println("Press [SW1] to start programmed motion. Press [SW2] to start teaching mode.");
  while (true)
  {
    digitalWrite(BDPIN_LED_USER_1, LOW);
    if(digitalRead(BDPIN_PUSH_SW_1))
    {
      Serial.println("Programmed motion mode begin.");
      teaching_mode_flag = false;
      warning();
      break;
    }
    else if(digitalRead(BDPIN_PUSH_SW_2))
    {
      Serial.println("Teaching mode begin.");
      teaching_mode_flag = true;
      break;
    }
  }
  digitalWrite(BDPIN_LED_USER_1, HIGH);
}

void loop()
{
  if (teaching_mode_flag)
  {
    open_manipulator.disableAllActuator();
    delay_ms(1000);
    Serial.println("Press [SW2] to append Current Pose. Press [SW1] to finish appending pose and play taught pose");

    while (true)
    {
      digitalWrite(BDPIN_LED_USER_2, LOW);
      if (digitalRead(BDPIN_PUSH_SW_2))
      {
        // Append(Teach) the current Pose to the present_position data
        uint8_t pos_vector_index = motion_index;
        present_position = open_manipulator.receiveAllJointActuatorValue();
        gripper_position = open_manipulator.receiveAllToolActuatorValue();
        for(uint8_t joint_index = 0; joint_index < 4; joint_index++ )
        {
          saved_teaching_pose.push_back(present_position.at(joint_index));
        }
        saved_teaching_pose.push_back(gripper_position.at(0));

        // Display saved Pose Joint angles in Radian
        Serial.print("[Pose ");
        Serial.print(motion_index/5 + 1);
        Serial.print("] : ");
        Serial.print(saved_teaching_pose[pos_vector_index++].position, 3);
        Serial.print(", ");
        Serial.print(saved_teaching_pose[pos_vector_index++].position, 3);
        Serial.print(", ");
        Serial.print(saved_teaching_pose[pos_vector_index++].position, 3);
        Serial.print(", ");
        Serial.print(saved_teaching_pose[pos_vector_index++].position, 3);
        Serial.print(", ");
        Serial.println(saved_teaching_pose[pos_vector_index++].position, 3);
        motion_index = motion_index + 5;
        break;
      }
      if (digitalRead(BDPIN_PUSH_SW_1))
      {
        digitalWrite(BDPIN_LED_USER_3, LOW);
        teaching_mode_flag = false;

        // Display the list of saved(taught) Pose
        for(uint8_t index = 0; index < motion_index ; index++)
        {
          if(index % 5 == 0)
          {
            Serial.println();
          }
          else
          {
            Serial.print(", ");
          }
          Serial.print(saved_teaching_pose[index].position, 3);
        }

        warning();
        break;
      }
    }
    digitalWrite(BDPIN_LED_USER_2, HIGH);
  }
  else
  {
    present_time = millis()/1000.0;

    // Trajectory following movement occurs here
    if(present_time-previous_time >= control_time)
    {
      open_manipulator.processOpenManipulator(millis()/1000.0);
      previous_time = millis()/1000.0;
    }

    // Read the next Pose to move
    if(!saved_teaching_pose.empty())
    {
      runTeachingMotion(&open_manipulator);
    }
    else
    {
      runDemo(&open_manipulator);
    }
  }
}

// Output warning before starting motion
void warning()
{
  Serial.println();
  Serial.println("WARNING!!! OpenMANIPULATOR-X operates in 5 seconds.");
  delay_ms(1000);
  Serial.println("WARNING!!! OpenMANIPULATOR-X operates in 4 seconds.");
  delay_ms(1000);
  Serial.println("WARNING!!! OpenMANIPULATOR-X operates in 3 seconds.");
  delay_ms(1000);
  Serial.println("WARNING!!! OpenMANIPULATOR-X operates in 2 seconds.");
  delay_ms(1000);
  Serial.println("WARNING!!! OpenMANIPULATOR-X operates in 1 seconds.");
  open_manipulator.receiveAllJointActuatorValue();
  open_manipulator.receiveAllToolActuatorValue();
  open_manipulator.enableAllActuator();
  delay_ms(1000);
}

// Move in Joint Space 
void moveJS(OpenManipulator *open_manipulator, double j1, double j2, double j3, double j4, double gripper_pos, double time)
{
  static std::vector <double> target_angle;
  target_angle.clear();
  target_angle.push_back(j1);
  target_angle.push_back(j2);
  target_angle.push_back(j3);
  target_angle.push_back(j4);
  open_manipulator->makeJointTrajectory(target_angle, time);
  open_manipulator->makeToolTrajectory("gripper", gripper_pos);
}

/*****************************************************************************
** Initialize Demo
*****************************************************************************/
void initDemo()
{
  start_motion_flag = false;
  motion_cnt[0] = 0;

  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);
  pinMode(BDPIN_GPIO_1, OUTPUT);
  digitalWrite(BDPIN_GPIO_1, LOW);
  Serial.println("OpenManipulator Init Completed");
}

/*****************************************************************************
** Play Demo Motion
*****************************************************************************/
void runDemo(OpenManipulator *open_manipulator)
{
  if(!start_motion_flag && !stop_motion_flag)
  {
    startMotion();
  }
  if(!start_motion_flag && stop_motion_flag)  // Restart DEMO
  {
    if(digitalRead(BDPIN_PUSH_SW_1))
    {
      startMotion();
    }
  }
  if(digitalRead(BDPIN_PUSH_SW_2) && !stop_motion_flag)  // Stop DEMO
  {
    stopMotion(open_manipulator);
  }

  if (open_manipulator->getMovingState())
  {
    return;
  }
  else 
  {
    if (start_motion_flag)
    {
      switch(motion_cnt[0])
      {
        case 0:
          moveJS(open_manipulator,0.040, 0.239, -0.117, 1.310, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 1:
          moveJS(open_manipulator, -0.015, -0.296, -0.221, 1.405, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 2:
          moveJS(open_manipulator, -1.465, -0.219, -0.169, 1.396, -0.000, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 3:
          moveJS(open_manipulator, -1.422, 0.357, -0.541, 1.362, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 4:
          moveJS(open_manipulator, -1.427, 0.632, -0.509, 1.370, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 5:
          moveJS(open_manipulator, -1.436, 0.638, -0.509, 1.364, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 6:
          moveJS(open_manipulator, -1.417, 0.048, -0.506, 1.640, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 7:
          moveJS(open_manipulator,-0.150, -0.015, -0.502, 1.717, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 8:
          moveJS(open_manipulator, 1.572, -0.356, -0.152, 1.430, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 9:
          moveJS(open_manipulator, 1.589, 0.230, -0.282, 1.416, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 10:
          moveJS(open_manipulator, 1.586, 0.379, -0.505, 1.637, -0.004, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 11:
          moveJS(open_manipulator, 1.568, 0.206, -0.939, 2.075, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 12:
          moveJS(open_manipulator, -0.025, 0.350, -0.923, 2.115, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 13:
          moveJS(open_manipulator, -1.440, 0.282, -0.635, 1.898, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 14:
          moveJS(open_manipulator, -1.430, 0.201, 0.207, 1.083, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 15:
          moveJS(open_manipulator, -1.420, 0.195, 0.199, 1.100, -0.007, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 16:
          moveJS(open_manipulator, -1.414, 0.098, -0.824, 2.168, -0.007, 2.0); 
          motion_cnt[0] ++; 
          break;        
        case 17:
          moveJS(open_manipulator, -0.095, 0.044, -0.525, 1.566, -0.007, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 18:
          moveJS(open_manipulator, 1.272, -0.250, -0.091, 1.308, -0.007, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 19:
          moveJS(open_manipulator, 1.276, 0.216, -0.153, 1.224, -0.007, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 20:
          moveJS(open_manipulator, 1.282, 0.233, -0.175, 1.236, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 21:
          moveJS(open_manipulator, 1.264, 0.201, -0.824, 1.865, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 22:
          moveJS(open_manipulator, 0.091, 0.290, -0.824, 1.879, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 23:
          moveJS(open_manipulator, -1.150, 0.161, -0.824, 1.898, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;          
        case 24:
          moveJS(open_manipulator, -1.147, 0.663, -1.224, 1.987, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;  
        case 25:
          moveJS(open_manipulator, -1.144, 0.997, -1.226, 1.747, -0.001, 2.0); 
          motion_cnt[0] ++; 
          break;   
        case 26:
          moveJS(open_manipulator, -1.152, 0.997, -1.229, 1.738, -0.008, 2.0); 
          motion_cnt[0] ++; 
          break; 
        case 27:
          moveJS(open_manipulator, -1.147, 0.307, -1.229, 2.166, -0.008, 2.0); 
          motion_cnt[0] ++; 
          break; 
        case 28:
          moveJS(open_manipulator, -0.044, 0.405, -1.229, 2.102, -0.007, 2.0); 
          motion_cnt[0] ++; 
          break; 
        case 29:
          moveJS(open_manipulator, 1.614, -0.492, 0.318, 1.341, -0.007, 2.0); 
          motion_cnt[0] ++; 
          break;           
        case 30:
          moveJS(open_manipulator, 1.598, -0.408, 0.345, 1.321, -0.004, 2.0); 
          motion_cnt[0] ++; 
          break;  
        case 31:
          moveJS(open_manipulator, 1.598, -0.436, 0.008, 1.687, -0.004, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 32:
          moveJS(open_manipulator, 0.035, -0.396, -0.040, 1.537, -0.004, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 33:
          moveJS(open_manipulator, -1.114, -0.020, -0.374, 1.571, -0.004, 2.0); 
          motion_cnt[0] ++; 
          break;          
        case 34:
          moveJS(open_manipulator, -1.062, 0.413, -0.230, 1.351, -0.004, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 35:
          moveJS(open_manipulator, -1.039, 0.419, -0.156, 1.253, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;  
        case 36:
          moveJS(open_manipulator, -1.062, -0.107, -0.259, 1.582, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 37:
          moveJS(open_manipulator, 0.087, -0.103, -0.267, 1.528, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 38:
          moveJS(open_manipulator, 1.198, -0.069, -0.098, 1.568, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;          
        case 39:
          moveJS(open_manipulator, 1.204, 0.132, -0.095, 1.519, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 40:
          moveJS(open_manipulator, 1.198, 0.101, -0.094, 1.571, 0.004, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 41:
          moveJS(open_manipulator, 1.158, -0.026, -0.954, 2.168, 0.001, 2.0); 
          motion_cnt[0] ++; 
          break;          
        case 42:
          moveJS(open_manipulator, -0.095, -0.255, -0.624, 1.396, -0.005, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 43:
          moveJS(open_manipulator, -0.074, 0.192, -0.408, 1.727, -0.005, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 44:
          moveJS(open_manipulator, -0.083, 0.199, -0.405, 1.726, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;          
        case 45:
          moveJS(open_manipulator, 0.084, -0.382, -0.529, 1.605, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 46:
          moveJS(open_manipulator, 1.424, -0.557, -0.313, 1.634, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 47:
          moveJS(open_manipulator, 1.348, -0.219, -0.160, 1.623, -0.009, 2.0); 
          motion_cnt[0] ++; 
          break;
        case 48:
          moveJS(open_manipulator, 1.302, -0.144, -0.221, 1.575, -0.004, 2.0); 
          motion_cnt[0] ++; 
          break;          
        case 49:
          moveJS(open_manipulator, 1.287, -0.683, -0.462, 1.382, -0.004, 2.0); 
          motion_cnt[0] ++; 
          break;   
        case 50:
          moveJS(open_manipulator, -0.166, -0.545, 0.167, 0.953, -0.004, 2.0); 
          motion_cnt[0] ++; 
          break;   
        default:
          motion_cnt[0] = 0;
          break;
      }
    }
  }

  
}

/*****************************************************************************
** Play Teaching Motion
*****************************************************************************/
void runTeachingMotion(OpenManipulator *open_manipulator)
{
  if(!start_motion_flag && !stop_motion_flag) // Start Teach Motion
  {
    startMotion();
  }
  if(!start_motion_flag && stop_motion_flag)  // Restart Teach Motion
  {
    if(digitalRead(BDPIN_PUSH_SW_1))
    {
      startMotion();
    }
  }
  if(digitalRead(BDPIN_PUSH_SW_2) && !stop_motion_flag)  // Stop Teach Motion
  {
    stopMotion(open_manipulator);
    digitalWrite(BDPIN_LED_USER_1, HIGH); // Turn off USER1 LED
    digitalWrite(BDPIN_LED_USER_2, HIGH); // Turn off USER2 LED
    digitalWrite(BDPIN_LED_USER_3, HIGH); // Turn off USER3 LED
    digitalWrite(BDPIN_LED_USER_4, HIGH); // Turn off USER4 LED
  }

  if (open_manipulator->getMovingState()) // Check if OpenMANIPULATOR is moving along the trajectory
  {
    // If OpenMANIPULATOR is still moving along the trajectory, then do not get the next Pose and return to main loop
    // Digital Output HIGH(3.3V) on OpenCR GPIO pin #3 (http://emanual.robotis.com/docs/en/parts/controller/opencr10/#gpio)
    digitalWrite(BDPIN_GPIO_1, HIGH);
    return;
  }
  else
  {
    // Digital Output LOW(0V) on OpenCR GPIO pin #3 (http://emanual.robotis.com/docs/en/parts/controller/opencr10/#gpio)
    digitalWrite(BDPIN_GPIO_1, LOW);
    if (start_motion_flag)
    {
      if (motion_cnt[0] < motion_index)
      {
        uint8_t joint1_index = motion_cnt[0]++;
        uint8_t joint2_index = motion_cnt[0]++;
        uint8_t joint3_index = motion_cnt[0]++;
        uint8_t joint4_index = motion_cnt[0]++;
        uint8_t gripper_index = motion_cnt[0]++;
        
        // Play each motion during 3.0 seconds
        moveJS(open_manipulator, saved_teaching_pose[joint1_index].position, saved_teaching_pose[joint2_index].position, saved_teaching_pose[joint3_index].position, saved_teaching_pose[joint4_index].position, saved_teaching_pose[gripper_index].position, 3.0);
      }
      else
      {
        motion_cnt[0] = 0;
      }
    }
  }
}

/*****************************************************************************
** Start Motion
*****************************************************************************/
void startMotion()
{
  Serial.println("Press [SW2] to Stop Motion");
  // Start the motion
  start_motion_flag = true;
  stop_motion_flag = false;
  
  motion_cnt[0] = 0;
}

/*****************************************************************************
** Stop Motion
*****************************************************************************/
void stopMotion(OpenManipulator *open_manipulator)
{
  Serial.println("Press [SW1] to Start Motion");
  // Stop the motion
  start_motion_flag = false;
  stop_motion_flag = true;

  // Go to the Init Pose(Right angle pose).
  moveJS(open_manipulator, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0); 
  motion_cnt[0] = 0;
}
