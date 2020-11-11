# -*- coding: utf-8 -*-
"""
Created on Sat Jun 27 12:03:29 2020

@author: Thomas
"""

import numpy as np                                 # (pip install numpy)
import json
import scipy.io
import os
import cv2


SimDataPath = './SimulationData0910_Train/'
trainvalname = 'train2017'

uniformBoxes = True
writeJPGs = True
drawBoxes = False


def generate_annotation(TargetID, label, image_id, annotation_id):
    # label = {TargetID;  [targetR, targetV, azi, egoMotion, xPos, yPos, width, 
    #                       length, heading, obstruction]}
    
    # Define RD map
    fmcwL = 256 #velbins/chirps 
    fmcwK = 320 #2*rangebins
    sweepBw = 1e9 #Bw in Hz
    chirpInterval = 64e-6 #in sec
    c_0 = 299792458 #Speed of Light
    f0 = 76.5e9 #Radar Freq
    fmcwdR = c_0/(2*sweepBw) #Range resolution
    fmcwdV = 1/(fmcwL *chirpInterval) * c_0 /(2*f0) #Velocity resolution
    #fmcwrangeBins = range(fmcwK/2)*fmcwdR;
    #fmcwvelBins = (range(fmcwL)-fmcwL/2)  *fmcwdV;
    
    # Calculate simple bounding box coords in R-vd
    azi = label[2]
    heading = label[8]
    DOA = (heading-azi)%180
  
    corner1 = [label[4]+ np.cos(label[8] * np.pi/180)*label[7]/2 + np.cos((label[8]+90) * np.pi/180)* label[6]/2, label[5]+ np.sin(label[8] * np.pi/180)*label[7]/2+ np.sin((label[8]+90) * np.pi/180)* label[6]/2]
    corner2 = [label[4]+ np.cos(label[8] * np.pi/180)*label[7]/2 + np.cos((label[8]-90) * np.pi/180)* label[6]/2, label[5]+ np.sin(label[8] * np.pi/180)*label[7]/2+ np.sin((label[8]-90) * np.pi/180)* label[6]/2]
    corner3 = [label[4]- np.cos(label[8] * np.pi/180)*label[7]/2 - np.cos((label[8]+90) * np.pi/180)* label[6]/2, label[5]- np.sin(label[8] * np.pi/180)*label[7]/2- np.sin((label[8]+90) * np.pi/180)* label[6]/2]
    corner4 = [label[4]- np.cos(label[8] * np.pi/180)*label[7]/2 - np.cos((label[8]-90) * np.pi/180)* label[6]/2, label[5]- np.sin(label[8] * np.pi/180)*label[7]/2- np.sin((label[8]-90) * np.pi/180)* label[6]/2]
    
    corners = np.array([corner1, corner2, corner3, corner4])
    Range = np.sqrt(np.multiply(corners[:,0],corners[:,0])+np.multiply(corners[:,1],corners[:,1]))
    dR = 0.5*(max(Range)-min(Range))      
    dV = abs(label[1]/2)
    

    # Box width/height: min [0,0] max [256,160]
    minR = int(min(Range)/fmcwdR)
    maxR = int(max(Range)/fmcwdR)
    minV = int((label[1]-dV)/fmcwdV)
    maxV = int((label[1]+dV)/fmcwdV)
    
    if uniformBoxes:
        if 'Ped' in TargetID:
            # Doppler Spread 2 m/s
            maxR = int(minR+1.2/fmcwdR)
            minV = int((label[1] - 1.2)/fmcwdV)
            maxV = int((label[1] + 1.2)/fmcwdV)
        elif 'Bic' in TargetID:
            # Doppler Spread 6 m/s
            if (maxR-minR) < 1.5/fmcwdR:
                maxR = int(minR+1.5/fmcwdR)
            minV = int((label[1] - 3)/fmcwdV)
            maxV = int((label[1] + 3)/fmcwdV)
        else: # 'Veh' in TargetID:
            # Doppler Spread 6 m/s
            if (maxR-minR) < 2.5/fmcwdR:
                maxR = int(minR+2.5/fmcwdR)
            minV = int((label[1] - 3)/fmcwdV)
            maxV = int((label[1] + 3)/fmcwdV)
        
    
    outofRange = 0 # Check if in Range of Radar
    if label[0]<0 or (label[0]/fmcwdR)>(fmcwK/2) or abs(label[1]/fmcwdV)>(fmcwL/2):
        outofRange = 1 #center out of range
    if minR<0:
        minR = 0
    if minR>(fmcwK/2):
        minR = int(fmcwK/2)-1
        outofRange = 1
    if maxR<0:
        maxR = 1
        outofRange = 1
    if maxR>(fmcwK/2):
        maxR = int(fmcwK/2)
    if minV < -(fmcwL/2):
        minV = -int(fmcwL/2)
    if minV > (fmcwL/2):
        minV = int(fmcwL/2)-1
        outofRange = 1
    if maxV < -(fmcwL/2):
        maxV = -int(fmcwL/2)+1
        outofRange = 1
    if maxV > (fmcwL/2):
        maxV = int(fmcwL/2)
    # Box width/height: min [0,0] max [256,160]
    topl = (int(fmcwL/2+minV),int(maxR))
    botr = (int(fmcwL/2+maxV),int(minR))
    center = (int(fmcwL/2+(maxV+minV)/2),int((maxR+minR)/2))
    
    if abs(minV-maxV) <3 and not outofRange:
        print('Small width of window (w <3 )')
    if abs(minR-maxR) <3 and not outofRange:
        print('Small height of window (h <3 )')
    
    # Check box position for errors
    if (center[0]/fmcwL) >=1 and not outofRange:
        print('V >= 1')
    elif (center[0]/fmcwL) <=0 and not outofRange:
        print('V <= 0')
    if (2*center[1]/fmcwK) >=1 and not outofRange:
        print('R >= 1')
    elif (2*center[1]/fmcwK) <=0 and not outofRange:
        print('R <= 0')
    
    
    # Find simple contours, set bounding box (x,y) coordinates 
    segmentations = []
    segmentations.append(topl[0]) #bottom left x
    segmentations.append(botr[1]) #bottom left y
    segmentations.append(topl[0]) #top left x
    segmentations.append(topl[1]) #top left y
    segmentations.append(botr[0]) #top right x
    segmentations.append(topl[1]) #top right y
    segmentations.append(botr[0]) #bottom left x
    segmentations.append(botr[1]) #bottom right y
    segmentations.append(topl[0]) #bottom left x
    segmentations.append(botr[1]) #bottom left y
    
    
    # calculate the bounding box and area
    # bbox = [x,y,width,height]
    x = topl[0] #center[0]
    y = botr[1] #center[1]
    w = abs(maxV-minV)
    h = abs(maxR-minR)
    bbox = (int(x), int(y), int(w), int(h))
    area = w*h
    
    is_crowd = 0
    
    #Category label
    if 'Veh' in TargetID:
        category = 0
    elif 'Ped' in TargetID:
        category = 1
    elif 'Bic' in TargetID:
        category = 2
    else:
        category = 0
    
    if not outofRange:
        annotation = {
            'segmentation': [segmentations],
            'iscrowd': is_crowd,
            'image_id': image_id,
            'category_id': category,
            'id': annotation_id,
            'bbox': bbox,
            'area': area
        }
    else:
        annotation = {}

    return annotation



def writeJPG(RDrgb, image_id, trainvalname):
    # Save RD as 2d grayscale jpg
    #Reduce 3rd dimension
    RGBimg = []
    for RD in RDrgb:
        RDmap = np.amax(RD,2)
        #Set limits
        RDmap[RDmap < -185] = -185
        RDmap[RDmap > -120] = -120
        RDmap = (RDmap+185)/(185-120) *255
        #Normalize and convert to grayscale
        RGBimg.append(RDmap)
    RGBimg = np.dstack(RGBimg)
    RDmap = cv2.UMat(RGBimg)
    
    #Save JPG
    folder = './COCO/images/'+ trainvalname + '/'
    cv2.imwrite(folder+str(image_id)+'.jpg', RDmap)
    
    return



def plotBoundingBoxes(RD, labels):
    import matplotlib.pyplot as plt
    
    # Define RD map
    fmcwL = 256 #velbins/chirps 
    fmcwK = 320 #2*rangebins
    sweepBw = 1e9 #Bw in Hz
    chirpInterval = 64e-6 #in sec
    c_0 = 299792458 #Speed of Light
    f0 = 76.5e9 #Radar Freq
    fmcwdR = c_0/(2*sweepBw) #Range resolution
    fmcwdV = 1/(fmcwL *chirpInterval) * c_0 /(2*f0) #Velocity resolution
    rangeBins = np.arange(int(fmcwK/2)) *fmcwdR;
    velBins = (np.arange(fmcwL)-fmcwL/2)  *fmcwdV;
    
    #Reduce 3rd dimension
    RDmap = np.amax(RD,2)
    #Set limits
    RDmap[RDmap < -185] = -185
    RDmap[RDmap > -120] = -120
    #Normalize and convert to grayscale
    RDmap = cv2.UMat((RDmap+185)/(185-120) *255)
    #Flip ud for correct plot
    #RDmap = cv2.flip(RDmap, 0)
    
    for Label in labels:
        #Plot Bounding Boxes
        # Calculate simple bounding box coords in R-vd
        TargetID = str(np.squeeze(Label[0]))
        label = np.squeeze(Label[1])
        azi = label[2]
        heading = label[8]
        DOA = (heading-azi)%180
      
        corner1 = [label[4]+ np.cos(label[8] * np.pi/180)*label[7]/2 + np.cos((label[8]+90) * np.pi/180)* label[6]/2, label[5]+ np.sin(label[8] * np.pi/180)*label[7]/2+ np.sin((label[8]+90) * np.pi/180)* label[6]/2]
        corner2 = [label[4]+ np.cos(label[8] * np.pi/180)*label[7]/2 + np.cos((label[8]-90) * np.pi/180)* label[6]/2, label[5]+ np.sin(label[8] * np.pi/180)*label[7]/2+ np.sin((label[8]-90) * np.pi/180)* label[6]/2]
        corner3 = [label[4]- np.cos(label[8] * np.pi/180)*label[7]/2 - np.cos((label[8]+90) * np.pi/180)* label[6]/2, label[5]- np.sin(label[8] * np.pi/180)*label[7]/2- np.sin((label[8]+90) * np.pi/180)* label[6]/2]
        corner4 = [label[4]- np.cos(label[8] * np.pi/180)*label[7]/2 - np.cos((label[8]-90) * np.pi/180)* label[6]/2, label[5]- np.sin(label[8] * np.pi/180)*label[7]/2- np.sin((label[8]-90) * np.pi/180)* label[6]/2]
        
        corners = np.array([corner1, corner2, corner3, corner4])
        Range = np.sqrt(np.multiply(corners[:,0],corners[:,0])+np.multiply(corners[:,1],corners[:,1]))
        dR = 0.5*(max(Range)-min(Range)) 
        dV = abs(label[1]/2)
        
        minR = int(min(Range)/fmcwdR)
        maxR = int(max(Range)/fmcwdR)
        minV = int((label[1]-dV)/fmcwdV)
        maxV = int((label[1]+dV)/fmcwdV)
        
        if uniformBoxes:
            if 'Ped' in TargetID:
                # Doppler Spread 2 m/s
                maxR = int(minR+1.2/fmcwdR)
                minV = int((label[1] - 1.2)/fmcwdV)
                maxV = int((label[1] + 1.2)/fmcwdV)
            elif 'Bic' in TargetID:
                # Doppler Spread 6 m/s
                if (maxR-minR) < 1.5/fmcwdR:
                    maxR = int(minR+1.5/fmcwdR)
                minV = int((label[1] - 3)/fmcwdV)
                maxV = int((label[1] + 3)/fmcwdV)
            else: # 'Veh' in TargetID:
                # Doppler Spread 6 m/s
                if (maxR-minR) < 2.5/fmcwdR:
                    maxR = int(minR+2.5/fmcwdR)
                minV = int((label[1] - 3)/fmcwdV)
                maxV = int((label[1] + 3)/fmcwdV)
            
        outofRange = 0 # Check if in Range of Radar
        if minR<0:
            minR = 0
        if minR>(fmcwK/2):
            minR = int(fmcwK/2)-1
            outofRange = 1
        if maxR<0:
            maxR = 1
            outofRange = 1
        if maxR>(fmcwK/2):
            maxR = int(fmcwK/2)
        if minV < -(fmcwL/2):
            minV = -int(fmcwL/2)
        if minV > (fmcwL/2):
            minV = int(fmcwL/2)-1
            outofRange = 1
        if maxV < -(fmcwL/2):
            maxV = -int(fmcwL/2)+1
            outofRange = 1
        if maxV > (fmcwL/2):
            maxV = int(fmcwL/2)
        # Box width/height: min [0,0] max [256,160]
        topl = (int(fmcwL/2+minV),int(maxR))
        botr = (int(fmcwL/2+maxV),int(minR))
        center = (int(fmcwL/2+(maxV+minV)/2),int((maxR+minR)/2))        
        
        x = center[0]
        y = center[1]
        w = abs(maxV-minV)
        h = abs(maxR-minR)
        
        if 'Veh' in TargetID:
            color = [0,255,255] #green
        elif 'Ped' in TargetID:
            color = [255,0,0] #red
        elif 'Bic' in TargetID:
            color = [0,0,255] #blue
        else:
            color = [0,255,255] #green
        color = 255
        #boundingBox = patches.Rectangle((fmcwL+minV,fmcwK/2-maxR),abs(2*dV/fmcwdV),2*dR/fmcwdR, edgecolor=color, facecolor="none")
        #boundingBox= patches.Rectangle((100,50),30,50, linewidth=10, edgecolor=color, facecolor="r") 
        #ax.add_patch(boundingBox)
        tl = (int(center[0]-w/2), int(center[1]+h/2))
        br = (int(center[0]+w/2), int(center[1]-h/2))
        RDmap = cv2.rectangle(RDmap, tl, br, color, 2)
        #RDmap = cv2.rectangle(RDmap, topl, botr, color, 3)
        RDmap = cv2.circle(RDmap, center, radius=1, color= 255, thickness=2)
        print('LABEL:  '+TargetID)
        print(label)
        print('Format: [targetR, targetV, azi, egoMotion, xPos, yPos, width, length, heading, obstruction]\n')
    
    cv2.imwrite('Labelled_RDmap.png', RDmap)
    img = cv2.imread('Labelled_RDmap.png')
    # PLOT
    plt.figure(1, figsize=(12,8))
    extent = [velBins[0], velBins[-1], rangeBins[-1], rangeBins[0]]
    plt.imshow(img, extent= extent)
    plt.gca().invert_yaxis()
    plt.xlabel('Velocity (m/s)')
    plt.ylabel('Range (m)')
    plt.show()
    return
    

def writeJSONfile(SimDataPath, writeJPGs, trainvalname, drawBoxes):
    # SimDataPath = 'SimulationDataTrain/'
    dirs = os.listdir(SimDataPath)
    
    simpleFiles = True
    if simpleFiles:
        NumSzenarios = len(dirs)
    else:
        # find number of Szenarios
        NumSzenarios = 0;
        for folder in dirs:
            if 'Szenario' in folder:
                i = str.find('o')
                if i >= 0:
                    i = int(folder[i:])
                    if i>NumSzenarios:
                        NumSzenarios = i
                else:
                    exit('Error: Undefined Szenario Number...')
     
    ## INFO
    info = {
        'description': 'Simulated Radar Dataset',
        'url': 'https://github.com/thomaswengerter',
        'version': '1.0',
        'year': 2020,
        'contributor': 'thomaswengerter',
        'date_created': '28/06/2020'
    }   
    
    ## LICENSES
    licenses = {
        'url': 'https://opensource.org/licenses/MIT',
        'id': 1,
        'name': 'MIT License'
            }
    
    ## IMAGES
    images = []
    
    
    ## ANNOTATIONS
    # Collect data from all scenarios and write to JSON
    rgb_id = 0
    annotation_id = 0
    annotations = []
    for szenario in range(1,NumSzenarios+1):
        NumMeas = len(os.listdir(SimDataPath+ 'Szenario'+ str(szenario)+ '/'))/2
        
        for meas in range(1,1+int(NumMeas)):
            # IMAGE
            if meas%3==0:
                rgb_id += 1
                Labels = scipy.io.loadmat(SimDataPath+ 'Szenario'+ str(szenario)+ '/'+ 'Szenario'+ str(szenario)+ '_Label_'+str(meas)+'.mat')
                if writeJPGs:
                    RD1 = scipy.io.loadmat(SimDataPath+ 'Szenario'+ str(szenario)+ '/'+ 'Szenario'+ str(szenario)+ '_'+str(meas-2)+'.mat')
                    RD2 = scipy.io.loadmat(SimDataPath+ 'Szenario'+ str(szenario)+ '/'+ 'Szenario'+ str(szenario)+ '_'+str(meas-1)+'.mat')
                    RD3 = scipy.io.loadmat(SimDataPath+ 'Szenario'+ str(szenario)+ '/'+ 'Szenario'+ str(szenario)+ '_'+str(meas)+'.mat')
                    RDrgb = [RD1['RD'], RD2['RD'], RD3['RD']]
                    writeJPG(RDrgb, rgb_id, trainvalname)
                filename = str(rgb_id)+'.jpg'
                image = {
                    'license': 1,
                    'file_name': filename,
                    'coco_url': 'https://github.com/thomaswengerter',
                    'height': 160, #160
                    'width': 256, #256
                    'date_captured': '21/06/2020',
                    'flickr_url': 'https://github.com/thomaswengerter',
                    'id': rgb_id
                        }
                images.append(image)
                for Label in Labels['label']:
                    # ANNOTATION
                    annotation_id += 1
                    TargetID = str(np.squeeze(Label[0]))
                    label = np.squeeze(Label[1])
                    annotation = generate_annotation(TargetID, label, rgb_id, annotation_id)
                    if annotation != {}:
                        annotations.append(annotation) # collect annotations in long list
                
                
                #DEBUG
                if drawBoxes:
                    RD = scipy.io.loadmat(SimDataPath+ 'Szenario'+ str(szenario)+ '/'+  'Szenario'+ str(szenario)+ '_'+str(meas)+'.mat')
                    plotBoundingBoxes(RD['RD'], Labels['label'])
                    
    
    # CATEGORIES
    categories = []
    category = {'supercategory': 'Road Users',
                'id': 0,
                'name': 'Vehicle'}
    categories.append(category)
    category = {'supercategory': 'Road Users',
                'id': 1,
                'name': 'Pedestrian'}
    categories.append(category)
    category = {'supercategory': 'Road Users',
                'id': 2,
                'name': 'Bicycle'}
    categories.append(category)
    
    
    # Build main frame
    mainFrame = {
        'info': info,
        'licenses': licenses,
        'images': images,
        'annotations': annotations,
        'categories': categories
    }   
    
    JSON = json.dumps(mainFrame)
    # Writing to sample.json 
    with open("./COCO/annotations/instances_"+ trainvalname +".json", "w") as outfile: 
        outfile.write(JSON) 
    print('Writing JSON file complete!')
    if writeJPGs:
        print('Writing JPG images complete!')
    print('Generated '+ str(annotation_id)+ ' annotations for '+ str(rgb_id)+ ' RD images.')
    return
  
    

# MAIN     


if not os.path.exists('./COCO'):
    stat = os.mkdir('./COCO')
    stat = os.mkdir('./COCO/annotations')
    
if writeJPGs and not os.path.exists('./COCO/images/'):
    stat = os.mkdir('./COCO/images')
stat = os.mkdir('./COCO/images/'+trainvalname)

writeJSONfile(SimDataPath, writeJPGs, trainvalname, drawBoxes)

