STAGE 1

Using WiLoR on Macbook CPU takes a lot of time and processing is lenghty.
I used source code from WiLoR demo.py to change the pipeline for video format - visualize_pose_videos.py and added a plotting layer - visualize_pose.py

WiLoR output per frame should be read in this manner:

Wrist - index 0 
Thumb - indices (joint numbers) 1-4;
Fingertip - 4

Index - indices (joint numbers) 5-8;
Fingertip - 8

Middle - indices (joint numbers) 9-12;
Fingertip - 12

Ring - indices (joint numbers) 13-16;
Fingertip - 16

Pinky - indices (joint numbers) 17-20;
Fingertip - 20

To position the wrist location in space, camera_translation needs to be applied to the hand position

STAGE 2 
After adjusting the output format to only joints position (vertices deleted) and confidence score - relevant for robotic translation, the JSON output should be analysed (broken down into singular important phases of the captured activity). My notes about the frame analysis where:
Frame 000000- 000030 - positioning the needle in between the fingertips 

Frame 000030- 00058 - pushing the needle through one layer of fabric with right hand - left hand holds the fabric. Needle position changes to vertical to stab through the fabric layer

 Re-directing the needle end to re-pierce the fabric. Starting position - Needle still held by the fingers from top fabric layer (20% of the needle outside, 80% of the needle is through\on the other side of the fabric).Hand turns the needle holding that 20% to pierce the fabric and extract the needle from its tip. The middle and the ring finger help redirect the needle (move it from vertical to horizontal in relation to the fabric or table position)

Frame 000059 - 000069 FLAG - the hand releases the needle end and travels to grab the pointy end
Frame 000070 - hand attempts to grab the pointy end
Frame 000071 - 000100 needle is pulled outwards, to pull the thread through a previously marked track

Frames 000100 - 000195
Ignore - user reaches for thread, irrelevant to the gesture


Frames 000195 - 000211
The needle is repositioned against the fabric layer at a steep angle to pierce through

Frames 000212 - 000223
The needle is inserted into fabric and pushed horizontally. Fabric is manipulated by the left hand to conform to the needle

Frames 000224 - 000320

Needle is being pushed through the fabric to stick out on the other side, while the tip is being held. The hand turns the needle to a vertical, and then a horizontal position again - to pierce it through the fabric in a direction opposite to the previous. Force is applied to insert the needle in between two holes - insertion point and extraction point.


Frames 000321 - 000330 FLAG - needle is released. The hand travels to the pointy end to extract it

Frame 000331 - 000332the hand arrived at the extraction point, it grasps the tip.

Frame 000333 - 000339
The needle is being pulled by the tip to be extracted from under the fabric. The other hand hold the fabric down to accelerate the step.


After understanding the division and form of the output, the JSON files are globbed using glob_json.py to produce a three-dimensional NumPy array in this format: npz = [n_frames, 21, 3], in which npz[0] tells you which frame it is, npz[1] which one of the 21 joints coordinates you're looking at, and npz[2] lists the x,y,z coordinates for the respective joint. I didn't come up with that method of analysis on my own, I inserted the WiLoR repo + my analysis into Claude to understand how to do it. All the code edits were done by myself, as I only relied on AI for gaining process knowledge.

WiLoR analysis is not very good for estimating wrist position - the wrist position is portrayed in relation to the finger joints, which means it doesn't indicate bend or rotation.

WiLoR also sometimes flips the left hand on the drawing. That was fixed in glob_json but during globbing, I remembered that for the first demo i just need the right hand tracking - so I removed that part until later. I will probably add another globbing file to get the readible output of both hands for the future development. 

Retargeting to servos uses simple kinematics in relation to local frame P = wrist for mcp flexion angle and P = mcp for pip and tip flexion, treating each segment as a vector with start and end coordinates at each joint, f.e. for thumb tip-to-pip segment, vector t, coordinates in the index 4 from the json frame is the start of the vector, and index 3 is the end. 
The angle in between vectors a and b is derived from dot product of a and b divided by the dot product of their lenght.

The tendon architecture of the robot is designed with singe tendon per finger but the script can be easily modified to include double tendon architecture.

Specific notes for servos in my wiring setup - irrelevant if you're wiring them yourself:
Channel 0 - finger 5 - pinkie | max straightening - 2000us | max flex - 500us |
Channel 1 - finger 1 - thumb (adduction)| max backwards motion - 2500 us | max adduction to palm - 1000us |
Channel 2 - finger 3 - middle | max straightening - 1500us | max flex - 2400us |
Channel 3 - finger 2 - index | max straightening - 2000us | max flex - 500us |
Channel 4 - finger 4 - ring  | max straightening - 1500us | max flex - 500us |
Channel 5 - finger 1 - thumb | max straightening - 2400us | max flex - 500us |


STAGE 2 - Wrist setup

Demo's own comments: speed × 0.732 = rpm, and acceleration × 8.7 = deg/s².

Wrist tilt - ID 2 | max straightening - 3000 | max flex - 2200 |
wrist rotation - ID 1 | range from 2000 - 4095 |
acceleration
speed


What needs to be added is a tendon displacement leeway for wrist tilt. It's a natural occurence in human anatomy, called tendosis. Because finger tendons are attached to bones below the wrist, tilting the wrist stretched/relaxes the tendons. I recommend reading more on that on Wikipedia.

To acknlowledge the difference that needs to be applied to the finger position if the wrist moves, an equation ...
k = ?