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
