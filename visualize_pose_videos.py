from pathlib import Path
import torch
import argparse
import os
import cv2
import numpy as np
import json
from typing import Dict, Optional
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "WiLoR"))


from wilor.models import WiLoR, load_wilor
from wilor.utils import recursive_to
from wilor.datasets.vitdet_dataset import ViTDetDataset, DEFAULT_MEAN, DEFAULT_STD
from wilor.utils.renderer import cam_crop_to_full
from ultralytics import YOLO 



def main():
    parser = argparse.ArgumentParser(description='WiLoR demo for robot arm - no rendering, video input')
    parser.add_argument('--video', type=str, default='/Users/katarzynadlugosz/Downloads/WhatsApp Video 2026-05-22 at 15.31.27 (1).mp4', help='Path to input video')
    parser.add_argument('--out_folder', type=str, default='out_demo', help='Output folder to save pose data')
    parser.add_argument('--save_mesh', dest='save_mesh', action='store_true', default=False, help='If set, save meshes to disk')
    parser.add_argument('--rescale_factor', type=float, default=2.0, help='Factor for padding the bbox')
    parser.add_argument('--file_type', nargs='+', default=['*.jpg', '*.png', '*.jpeg'], help='List of file extensions to consider')
    parser.add_argument('--fast',   dest='fast', action='store_true', default=False, help='Use FP16 and layer dropping to accelerate inference')
    args = parser.parse_args()

    # model load - start of wilor copy
    print("Loading WiLoR model...")
    model, model_cfg = load_wilor(checkpoint_path='./pretrained_models/wilor_final.ckpt', cfg_path='./pretrained_models/model_config.yaml')
    if args.fast:     
        torch.set_float32_matmul_precision('high')
        model = model.half()
        model.backbone = torch.compile(model.backbone)
        model.backbone.skip_blocks = True 
        
    print("Loading hand detector...")
    detector = YOLO('./pretrained_models/detector.pt')
    
    device   = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    model    = model.to(device)
    detector = detector.to(device)
    model.eval()
    # end of copy

    # Make output directory if it does not exist
    os.makedirs(args.out_folder, exist_ok=True)

    # getting the video properties
    cap = cv2.VideoCapture(args.video)
    fps    = cap.get(cv2.CAP_PROP_FPS)
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"Found {total} frames")
    # start of inference
    # Iterate over all frames
    for frame_idx in range(total):
        ret, img_cv2 = cap.read()
        if not ret:
            print(f"  Error: Could not read frame {frame_idx}")
            continue

        # Detect hands
        print(f"  Detecting hands...")
        detections = detector(img_cv2, conf=0.3, verbose=False)[0]
        
        bboxes = []
        is_right = []
        for det in detections: 
            Bbox = det.boxes.data.cpu().detach().squeeze().numpy()
            is_right.append(det.boxes.cls.cpu().detach().squeeze().item())
            bboxes.append(Bbox[:4].tolist())
        
        if len(bboxes) == 0:
            print(f"  No hands detected")
            continue
        
        print(f"  Found {len(bboxes)} hand(s)")
        
        boxes = np.stack(bboxes)
        right = np.stack(is_right)
        
        # Prepare dataset
        dataset = ViTDetDataset(model_cfg, img_cv2, boxes, right, rescale_factor=args.rescale_factor, fp16=args.fast)
        dataloader = torch.utils.data.DataLoader(dataset, batch_size=16, shuffle=False, num_workers=0)

        # Process each batch
        print(f"  Estimating hand pose...")
        for batch in dataloader: 
            batch = recursive_to(batch, device)
    
            with torch.no_grad():
                out = model(batch) 
                
            multiplier    = (2*batch['right']-1)
            pred_cam      = out['pred_cam']
            pred_cam[:,1] = multiplier*pred_cam[:,1]
            box_center    = batch["box_center"].float()
            box_size      = batch["box_size"].float()
            img_size      = batch["img_size"].float()
            scaled_focal_length = model_cfg.EXTRA.FOCAL_LENGTH / model_cfg.MODEL.IMAGE_SIZE * img_size.max()
            pred_cam_t_full = cam_crop_to_full(pred_cam, box_center, box_size, img_size, scaled_focal_length).detach().cpu().numpy()

            # Extract and save hand pose data
            batch_size = batch['img'].shape[0]
            for n in range(batch_size):
                img_fn, _ = f"frame{frame_idx:06d}", None  # Use frame index as filename
                
                # Get hand data
                verts = out['pred_vertices'][n].detach().cpu().numpy()  # 778 hand vertices
                joints = out['pred_keypoints_3d'][n].detach().cpu().numpy()  # 21 hand joints
                
                is_right_hand = batch['right'][n].cpu().numpy()
                # Mirror x-axis if right hand
                verts[:,0]  = (2*is_right_hand-1)*verts[:,0]
                joints[:,0] = (2*is_right_hand-1)*joints[:,0]
                
                cam_t = pred_cam_t_full[n]  # Camera translation [tx, ty, tz]
                
                # Save as JSON for easy loading in robot code
                pose_data = {
                    'image': str(args.video),
                    'hand_index': n,
                    'is_right_hand': bool(is_right_hand),
                    'camera_translation': cam_t.tolist(),  # [tx, ty, tz]
                    'focal_length': float(scaled_focal_length.cpu().numpy()),
                    'hand_vertices': verts.tolist(),  # 778x3 - hand mesh vertices
                    'hand_joints': joints.tolist(),   # 21x3 - hand keypoints
                }
                #end of wilor copy
                pose_file = os.path.join(args.out_folder, f'{img_fn}_hand{n}.json')
                with open(pose_file, 'w') as f:
                    json.dump(pose_data, f, indent=2)
                print(f"    Saved pose to {os.path.basename(pose_file)}")
                
                
                # Optionally save mesh
                if args.save_mesh:
                    import trimesh
                    mesh_file = os.path.join(args.out_folder, f'{img_fn}_hand{n}.obj')
                    
                    # Create mesh using MANO faces
                    faces = model.mano.faces.cpu().numpy()
                    mesh = trimesh.Trimesh(vertices=verts, faces=faces)
                    mesh.export(mesh_file)
                    print(f"    Saved mesh to {os.path.basename(mesh_file)}")

    print("\nDone!")
    cap.release()

if __name__ == '__main__':
    main()
