
clc; clear;

close all; 
clear all;
clc
objects = imaqfind %find video input objects in memory

delete(objects) %delete a video input object from memory
[y, Fs] = audioread('alarm.mp3'); %this loads the alarm sound
player = audioplayer(y, Fs);
play(player) %we preload the alarm to pevent delay immediately the object cannot be detected
vid=videoinput('winvideo',1,'YUY2_640x480'); 
set(vid,'FramesPerTrigger',Inf);
set(vid,'ReturnedColorspace','rgb')
vid.FrameGrabInterval=3;  %this will make the camera a little bit slower but its better to avoid crash
a = arduino;  %loading the arduino

start(vid);

while(vid.FramesAcquired<=1000)
    
    data=getsnapshot(vid); %as the camera livestream it takes shots
    
    diff_im=imsubtract(data(:,:,1),rgb2gray(data)); %here we select only the red objects
    diff_im=medfilt2(diff_im,[3,3]);  %we use filter to remove any noise in the snapshot
    
    level = graythresh(diff_im); %we use this code to automatically calculate the gray threshold for the snapshot taken
    diff_im=im2bw(diff_im, level); %we cover the snapshot to black and white with all pixels below the threshold black
    %and all pixel greater than the threshold white
    diff_im=bwareaopen(diff_im,300); %this is used for connecting the connected label
    
    bw=bwlabel(diff_im,8);
    
    stats=regionprops(bw,'BoundingBox','Centroid','Solidity'); %we state the properties in which we want to take note of
    imshow(data);
    
    hold on
    
    for object=1:length(stats)
        if (stats(object).Solidity >= 0.75) && (stats(object).BoundingBox(3) >= 70) && (stats(object).BoundingBox(4) <= 200)
            %the if statement above is used to make sure the camera do not
            %just detect irrelevant red object. It must make sure the
            %solidity is above 0.75 and the width of the bounding box is
            %above 70 while the height is below 200
            
            bb=stats(object).BoundingBox;
            bc=stats(object).Centroid;
            
            disp(double(bc));
            rectangle('Position',bb,'EdgeColor','r','LineWidth',2)

            plot(bb(1),bb(2))

            ad=text(bb(1)+15,bb(2),strcat('X:',num2str(round(bb(1))),'Y',num2str(round(bb(2)))));%x and y coordinate of centroid

            set(ad,'fontName','Arial','FontWeight','bold','FontSize',12,'Color','yellow');
            
            ratio = double(bc(1)/bc(2)) %we take the ratio of the x and y centroid of the block
            pause(player);
            if ratio >= 0.67 && ratio <= 0.81 %this is important as if the centroid ratio is within this range even if the centroid
                %may not be where we want to be it is closer to the center
                %so we tell the arduino not to do anything by unsetting its
                %pin
                
                configurePin(a,'D22','Unset'); %unsetting arduino pin
                configurePin(a,'D28','Unset'); %unsetting arduino pin
            
            else %if the ratio isn't in that region we do the following
                
                if bc(1)>330 %move the robot to the right if centroid x axis greater than 330
                
                    writeDigitalPin(a,'D28',1)
                    writePWMVoltage(a, 'D3', 1.7) %this voltage set the speeds from 0 - 5V
                
                
                elseif bc(1)<260 
                
                    writeDigitalPin(a,'D28',0) %move robot to left if centroid x axis less than 260
                    writePWMVoltage(a, 'D3', 1.7) %this voltage set the speeds from 0 - 5V
            
                else  %if centroid x -axis is between 260 and 330 we make sure the robot don't move in the x axis
                    configurePin(a,'D28','Unset');
                end 
    
           
                if bc(2)>260 %move robot down if centroid Y-axis greater than 260
                    writePWMVoltage(a, 'D6', 1.7)
                
                    writeDigitalPin(a, 'D22',0)
        
                elseif bc(2)<200 %move robot up if centroid Y-axis less than 200
            
                    writePWMVoltage(a,'D6', 2)
                    writeDigitalPin(a,'D22',1)
           
                else
                    configurePin(a,'D22','Unset');
                end
           end
        else %if we can't detect the object, don't move robot and start the alarm
                resume(player) 
                configurePin(a,'D22','Unset');
                configurePin(a,'D28','Unset');
            
        end
    
        end
    
    hold off
    
end

stop(vid);
flushdata(vid);
clear all;
close all;