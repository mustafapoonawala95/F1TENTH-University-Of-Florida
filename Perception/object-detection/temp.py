# -*- coding: utf-8 -*-
"""
Created on Sun Aug 22 14:27:03 2021

@author: krish
"""
import torch
from torch import sqrt

def iou(preds, targets, btype):
        
        if btype == 'box':
            box1_x1 = preds[..., 0]
            box1_y1 = preds[..., 1]
            box1_x2 = preds[..., 2]
            box1_y2 = preds[..., 3]
            box2_x1 = targets[..., 0]
            box2_y1 = targets[..., 1]
            box2_x1 = targets[..., 2]
            box2_y1 = targets[..., 3]
            
        elif btype == 'midpoint':
            box1_x1 = preds[..., 0:1] - preds[..., 2:3] / 2
            box1_y1 = preds[..., 1:2] - preds[..., 3:4] / 2
            box1_x2 = preds[..., 0:1] + preds[..., 2:3] / 2
            box1_y2 = preds[..., 1:2] + preds[..., 3:4] / 2
            box2_x1 = targets[..., 0:1] - targets[..., 2:3] / 2
            box2_y1 = targets[..., 1:2] - targets[..., 3:4] / 2
            box2_x2 = targets[..., 0:1] + targets[..., 2:3] / 2
            box2_y2 = targets[..., 1:2] + targets[..., 3:4] / 2
            
        
        x1 = torch.max(box1_x1, box2_x1)
        y1 = torch.max(box1_y1, box2_y1)
        x2 = torch.min(box1_x2, box2_x2)
        y2 = torch.min(box1_y2, box2_y2)
        
        
        # .clamp(0) is for the case when they do not intersect
        intersection = (x2 - x1).clamp(0) * (y2 - y1).clamp(0)

        box1_area = abs((box1_x2 - box1_x1) * (box1_y2 - box1_y1))
        box2_area = abs((box2_x2 - box2_x1) * (box2_y2 - box2_y1))

        return intersection / (box1_area + box2_area - intersection + 1e-6)  

preds   = torch.rand((10,7,7,30))
targets = torch.rand((10,7,7,30))

iobj_i = torch.round(torch.abs(targets[..., 20])).unsqueeze(3)

B = 2
iou_list = []
for i in range(B):
    iou_temp = iou(preds[..., 21+(i*5):25+(i*5)], targets[..., 21:25], 'midpoint')
    iou_list.append(iou_temp.unsqueeze(0))

iou_list_cat = torch.cat(iou_list, dim=0)

iou_maxes, bbox_idxs = torch.max(iou_list_cat, dim=0)
iobj_ij = iobj_i * (bbox_idxs+1)

lambda_obj    = 5
lambda_no_obj = .5
batch_size = iobj_i.shape[0]
losses = torch.zeros((batch_size, 7, 7, 1))
for batch in range(batch_size):
    
    # print(batch)
    
    row = -1
    for i in range(7*7):
        if i%7==0:
            row += 1
        col = i%7
        
        bbox_idx = int(iobj_ij[batch, row, col, 0])
        
        # print(batch, row, col, bbox_idx)
        
        # if the box exists
        if bbox_idx:
            bbox_idx -= 1
            # Midpoint loss
            x_loss = (targets[batch, row, col, 21] - preds[batch, row, col, 21+(bbox_idx*5)]) ** 2
            y_loss = (targets[batch, row, col, 21] - preds[batch, row, col, 22+(bbox_idx*5)]) ** 2
            losses[batch, row, col, 0] =  lambda_obj*(x_loss + y_loss)
            
            # Width-Height loss
            w_loss = ( sqrt(targets[batch, row, col, 23])-sqrt(preds[batch, row, col, 23+(bbox_idx*5)]) ) ** 2
            h_loss = ( sqrt(targets[batch, row, col, 24])-sqrt(preds[batch, row, col, 24+(bbox_idx*5)]) ) ** 2
            losses[batch, row, col, 0] += lambda_obj*(w_loss + h_loss)
            
            # loss for probability of existance of object
            losses[batch, row, col, 0] += (targets[batch, row, col, 20] - preds[batch, row, col, 20+(bbox_idx*5)]) ** 2
            
            # Class loss - if obj present
            combined_loss = torch.sum( (targets[batch, row, col, 0:20] - preds[batch, row, col, 0:20])**2 )
            losses[batch, row, col, 0] += combined_loss
            
        
        # if the box does NOT exist
        else:
            # loss for probability of Non-existance of object
            for bbox in range(B):
                losses[batch, row, col, 0] += (targets[batch, row, col, 20] - preds[batch, row, col, 20+(bbox*5)]) ** 2
            
            
loss = torch.sum(losses)        
    
    
  


