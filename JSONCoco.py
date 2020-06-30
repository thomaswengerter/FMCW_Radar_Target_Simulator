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
    DOA = (azi+180+heading)%180
    if 'Ped' in TargetID:
        label[7] = 0.5
    thres = np.rad2deg(np.arctan(label[6]/(label[7]*2)))
        
    if DOA<thres or (DOA>180-thres):
        #Illuminated from front/back
        dR = abs(label[7]/(2*np.cos(np.deg2rad(DOA))))
    else:
        #Illuminated from sides
        dR = abs(label[6]/(2*np.cos(np.deg2rad(DOA-90))))
    dV = label[1]/2
    
    minR = label[0]-dR
    maxR = label[0]+dR
    minV = label[1]-dV
    maxV = label[1]+dV
    
    # Find simple contours, set bounding box coordinates 
    segmentations = []
    segmentations.append(int(minR/fmcwdR))
    segmentations.append(int(minV/fmcwdV))
    segmentations.append(int(maxR/fmcwdR))
    segmentations.append(int(minV/fmcwdV))
    segmentations.append(int(maxR/fmcwdR))
    segmentations.append(int(maxV/fmcwdV))
    segmentations.append(int(minR/fmcwdR))
    segmentations.append(int(maxV/fmcwdV))
    segmentations.append(int(minR/fmcwdR))
    segmentations.append(int(minV/fmcwdV))
    
    
    # calculate the bounding box and area
    # bbox = [x,y,width,height]
    bbox = (fmcwL/2+int(label[1]/fmcwdV), int(label[0]/fmcwdR), 2*abs(dV)/fmcwdV, 2*dR/fmcwdR)
    area = dR*2/fmcwdR * dV*2/fmcwdV

    is_crowd = 0
    
    #Category label
    if 'Veh' in TargetID:
        category = 1
    elif 'Ped' in TargetID:
        category = 2
    elif 'Bic' in TargetID:
        category = 3
    else:
        category = 1

    annotation = {
        'segmentation': segmentations,
        'iscrowd': is_crowd,
        'image_id': image_id,
        'category_id': category,
        'id': annotation_id,
        'bbox': bbox,
        'area': area
    }

    return annotation



def writeJPG(RD, image_id, trainvalname):
    # Save RD as 2d grayscale jpg
    #Reduce 3rd dimension
    RDmap = np.amax(RD,2)
    #Set limits
    RDmap[RDmap < -185] = -185
    RDmap[RDmap > -120] = -120
    #Normalize and convert to grayscale
    RDmap = cv2.UMat((RDmap+185)/(185-120) *255)
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
    
    for Label in labels:
        #Plot Bounding Boxes
        # Calculate simple bounding box coords in R-vd
        TargetID = str(np.squeeze(Label[0]))
        label = np.squeeze(Label[1])
        azi = label[2]
        heading = label[8]
        DOA = (azi+180+heading)%180
        if 'Ped' in TargetID:
            label[7] = 0.5
        thres = np.rad2deg(np.arctan(label[6]/(label[7]*2)))
            
        if DOA<thres or (DOA>180-thres):
            #Illuminated from front/back
            dR = abs(label[7]/(2*np.cos(np.deg2rad(DOA))))
        else:
            #Illuminated from sides
            dR = abs(label[6]/(2*np.cos(np.deg2rad(DOA-90))))
        dV = label[1]/2
    
        minR = int((label[0]-dR)/fmcwdR)
        maxR = int((label[0]+dR)/fmcwdR)
        minV = int((label[1]-dV)/fmcwdV)
        maxV = int((label[1]+dV)/fmcwdV)
        topl = (int(fmcwL/2+minV),int(maxR))
        botr = (int(fmcwL/2+maxV),int(minR))
        
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
        RDmap = cv2.rectangle(RDmap, topl, botr, color, 5)
        print('LABEL:  ')
        print(label)
        print('Format: [targetR, targetV, azi, egoMotion, xPos, yPos, width, length, heading, obstruction]')
    
    cv2.imwrite('Labelled_RDmap.png', RDmap)
    img = cv2.imread('Labelled_RDmap.png')
    # PLOT
    plt.figure(1, figsize=(12,8))
    extent = [velBins[0], velBins[-1], rangeBins[-1], rangeBins[0]]
    plt.imshow(img, extent= extent)
    plt.gca().invert_yaxis()
    plt.xlabel('Velocity / m/s')
    plt.ylabel('Range / m')
    plt.show()
    return
    

def writeJSONfile(SimDataPath, writeJPGs, trainvalname):
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
    image_id = 0
    annotation_id = 0
    annotations = []
    for szenario in range(1,NumSzenarios+1):
        NumMeas = len(os.listdir(SimDataPath+ 'Szenario'+ str(szenario)+ '/'))/2
        for meas in range(int(NumMeas)):
            # IMAGE
            image_id += 1
            Labels = scipy.io.loadmat(SimDataPath+ 'Szenario'+ str(szenario)+ '/'+ 'Szenario'+ str(szenario)+ '_Label_'+str(meas)+'.mat')
            if writeJPGs:
                RD = scipy.io.loadmat(SimDataPath+ 'Szenario'+ str(szenario)+ '/'+ 'Szenario'+ str(szenario)+ '_'+str(meas)+'.mat')
                writeJPG(RD['RD'],image_id, trainvalname)
            filename = str(image_id)+'.jpg'
            image = {
                'license': 1,
                'file_name': filename,
                'coco_url': 'https://github.com/thomaswengerter',
                'height': 160,
                'width': 256,
                'date_captured': '21/06/2020',
                'flickr_url': 'https://github.com/thomaswengerter',
                'id': image_id
                    }
            images.append(image)
            for Label in Labels['label']:
                # ANNOTATION
                annotation_id += 1
                TargetID = str(np.squeeze(Label[0]))
                label = np.squeeze(Label[1])
                annotation = generate_annotation(TargetID, label, image_id, annotation_id)
                annotations.append(annotation) # collect annotations in long list
            
            drawBoxesinPlot = True #DEBUG
            if drawBoxesinPlot:
                RD = scipy.io.loadmat(SimDataPath+ 'Szenario'+ str(szenario)+ '/'+  'Szenario'+ str(szenario)+ '_'+str(meas)+'.mat')
                plotBoundingBoxes(RD['RD'], Labels['label'])
                
    
    # CATEGORIES
    categories = []
    category = {'supercategory': 'Road Users',
                'id': 1,
                'name': 'Vehicle'}
    categories.append(category)
    category = {'supercategory': 'Road Users',
                'id': 2,
                'name': 'Pedestrian'}
    categories.append(category)
    category = {'supercategory': 'Road Users',
                'id': 3,
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
    print('Generated '+ str(annotation_id)+ ' annotations for '+ str(image_id)+ ' RD images.')
    return
  
    

# MAIN     

SimDataPath = './SimulationDataTrain/'
trainvalname = 'train2017'

writeJPGs = True


if not os.path.exists('./COCO'):
    stat = os.mkdir('./COCO')
    stat = os.mkdir('./COCO/annotations')
    
if writeJPGs and not os.path.exists('./COCO/images'):
    stat = os.mkdir('./COCO/images')
    
writeJSONfile(SimDataPath, writeJPGs, trainvalname)

