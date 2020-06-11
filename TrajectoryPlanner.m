classdef TrajectoryPlanner
    % Plan movement of targets for each timestep (measurement)
    %   PROPERTIES
    %   heading:        heading of each target for every single tstep
    %   trajectory:     xy Position of each target for every single tstep
    %   velocity:       velocity of each target for every single tstep
    %
    
    properties
        heading = [];       %heading
        trajectory = [];    %position
        velocity = [];      %velocity
        plotTraj = true;    %bool for live plot
    end
    
    methods
        function obj = init_TrajectoryPlanner(obj, tstep, Szduration, Targets, fmcw)
            % Initialize the for all traj time steps
            %   Accelerate target randomly
            %   Draw random curves for each target
            %   Avoid "collisions"
            %   Plot the trajectories in a live animation
            %
            
            names = fieldnames(Targets); %names of all targets
            
            %initialize traj time steps
            acc = zeros(numel(names), floor(Szduration/tstep));         % acceleration
            dheading = zeros(numel(names), floor(Szduration/tstep));    % changes in heading
            v = zeros(numel(names), floor(Szduration/tstep));           % velocities
            mapPos = zeros(numel(names), floor(Szduration/tstep), 2);   % xy positions
            dir = zeros(numel(names), floor(Szduration/tstep));         % direction angle
            
            
            for i = 1:numel(names)
                if strcmp(names{i}(1:3),'Veh')
                    %% Plan Vehicle
                    mv = fmcw.velBins(end); %15m/s max velocity
                    eval(['v(i,:) = ones(1,size(v,2))* Targets.',names{i}, '.vel;']);
                    eventduration = 4; % acceleration of a vehicle takes ~4s
                    %Vehicle acceleration
                    events = floor(rand()*Szduration/eventduration+0.3); %number of events possible in Szene
                    eventTidx = sort(ceil(abs(randn(events,1))/2 * size(acc,2)));   
                    eventTidx(eventTidx>size(acc,2)) = sort(ceil(rand(events,1) * size(acc,2)));
                    for event = 1:length(eventTidx)
                        idx = eventTidx(event);
                        if (mv-v(i,idx))<=0.3*mv
                            %high velocity -> decelerate
                            rndeventduration = abs(randn()+ eventduration + v(i,idx)/mv * eventduration *rand()); %random time to decelerate
                            randacc = -rand()* v(i,idx)/rndeventduration; %random acceleration
                            acc(i, idx:idx+ceil(rndeventduration/tstep)) = randacc; % acceleration to trajectory
                            v(i,idx:idx+ceil(rndeventduration/tstep)) = v(i,idx)+(0:ceil(rndeventduration/tstep))*tstep*randacc; % iterative update speed
                            v(i,idx+ceil(rndeventduration/tstep)+1:end) = v(i,idx+ceil(rndeventduration/tstep)); % update speed after acceleration
                            if rand()>0.3
                                %include change of heading for deceleration
                                rndturnangle = 180*(rand()-0.5); %turn +/- 90°
                                dheading(i,idx:idx+ceil(rndeventduration/tstep)) = rndturnangle/ceil(rndeventduration/tstep); % iterative update heading
                            end
                        elseif (mv-v(i,idx))>=0.7*mv
                            %low velocity -> accelerate
                            rndeventduration = abs(randn()+ eventduration + v(i,idx)/mv * eventduration *rand());
                            randacc = +rand()* (mv-v(i,idx))/rndeventduration;
                            acc(i, idx:idx+rndeventduration/tstep) = randacc;
                            v(i,idx:idx+ceil(rndeventduration/tstep)) = v(i,idx)+(0:ceil(rndeventduration/tstep))*tstep*randacc; % update speed
                            v(i,idx+ceil(rndeventduration/tstep)+1:end) = v(i,idx+ceil(rndeventduration/tstep)); % update speed after acceleration
                            if rand()>0.5
                                %include change of heading for deceleration
                                rndturnangle = 180*(rand()-0.5); %turn +/- 90°
                                dheading(i,idx:idx+ceil(rndeventduration/tstep)) = rndturnangle/ceil(rndeventduration/tstep); % iterative update heading
                            end
                        else
                            %medium velocity -> random acceleration/deceleration
                            rndeventduration = abs(randn()+ eventduration + v(i,idx)/mv * eventduration *rand());
                            if rand() > 0.5
                                randacc = +rand()* (mv-v(i,idx))/rndeventduration;
                            else 
                                randacc = -rand()* v(i,idx)/rndeventduration;
                            end
                            acc(i, idx:idx+rndeventduration/tstep) = randacc;
                            v(i,idx:idx+ceil(rndeventduration/tstep)) = v(i,idx)+(0:ceil(rndeventduration/tstep))*tstep*randacc; % update speed
                            v(i,idx+ceil(rndeventduration/tstep)+1:end) = v(i,idx+ceil(rndeventduration/tstep)); % update speed after acceleration
                            if rand()>0.5
                                %include change of heading for acceleration/deceleration
                                rndturnangle = 180*(rand()-0.5); %turn +/- 90°
                                dheading(i,idx:idx+ceil(rndeventduration/tstep)) = rndturnangle/ceil(rndeventduration/tstep); % iterative update heading
                            end
                        end                        
                    end
                    %Vehicle long turn
                    events = floor(events/2);
                    %eventTidx = sort(ceil(rand(events,1) * size(dheading,2)));
                    eventTidx = sort(ceil(abs(randn(events,1))/2 * size(acc,2)));   
                    eventTidx(eventTidx>size(acc,2)) = sort(ceil(rand(events,1) * size(acc,2)));
                    for event = 1:length(eventTidx)
                        idx = eventTidx(event);
                        rndeventduration = abs(eventduration + (v(i,idx)/mv-0.5) * rand());
                        rndturnangle = 80*(rand()-0.5); %turn +/- 40°
                        dheading(i,idx:idx+ceil(rndeventduration/tstep)) = rndturnangle/ceil(rndeventduration/tstep); % iterative update heading
                    end 
                    
                elseif strcmp(names{i}(1:3),'Bic')
                    %% Plan Bicycle
                    mv = 10; %10m/s max velocity
                    eval(['v(i,:) = ones(1,size(v,2))* Targets.',names{i}, '.Speed;']);
                    eventduration = 4; % acceleration of a bicycle takes ~6s
                    %Bicycle acceleration
                    events = floor(rand()*Szduration/eventduration+0.3); %number of events possible in Szene
                    eventTidx = sort(ceil(abs(randn(events,1))/2 * size(acc,2)));   
                    eventTidx(eventTidx>size(acc,2)) = sort(ceil(rand(events,1) * size(acc,2)));           
                    for event = 1:length(eventTidx)
                        idx = eventTidx(event);
                        if (mv-v(i,idx))<0.3*mv
                            %high velocity -> decelerate to 0
                            rndeventduration = abs(randn()+ eventduration + v(i,idx)/mv * eventduration *rand()); %random time to decelerate
                            randacc = -rand()* v(i,idx)/rndeventduration; %random acceleration
                            acc(i, idx:idx+ceil(rndeventduration/tstep)) = randacc; % acceleration to trajectory
                            v(i,idx:idx+ceil(rndeventduration/tstep)) = v(i,idx)+(0:ceil(rndeventduration/tstep))*tstep*randacc; % iterative update speed
                            v(i,idx+ceil(rndeventduration/tstep)+1:end) = v(i,idx+ceil(rndeventduration/tstep)); % update speed after acceleration
                            if rand()>0.3
                                %include change of heading for deceleration
                                rndturnangle = 180*(rand()-0.5); %turn +/- 90°
                                dheading(i,idx:idx+ceil(rndeventduration/tstep)) = rndturnangle/ceil(rndeventduration/tstep); % iterative update heading
                            end
                        elseif (mv-v(i,idx))>=0.7*mv
                            %low velocity -> accelerate to mv=10
                            rndeventduration = abs(randn()+ eventduration + v(i,idx)/mv * eventduration *rand());
                            randacc = +rand()* (mv-v(i,idx))/rndeventduration;
                            acc(i, idx:idx+rndeventduration/tstep) = randacc;
                            v(i,idx:idx+ceil(rndeventduration/tstep)) = v(i,idx)+(0:ceil(rndeventduration/tstep))*tstep*randacc; % update speed
                            v(i,idx+ceil(rndeventduration/tstep)+1:end) = v(i,idx+ceil(rndeventduration/tstep)); % update speed after acceleration
                            if rand()>0.5
                                %include change of heading for deceleration
                                rndturnangle = 180*(rand()-0.5); %turn +/- 90°
                                dheading(i,idx:idx+ceil(rndeventduration/tstep)) = rndturnangle/ceil(rndeventduration/tstep); % iterative update heading
                            end
                        else
                            %medium velocity -> random acceleration/deceleration
                            rndeventduration = abs(randn()+ eventduration + v(i,idx)/mv * eventduration *rand());
                            if rand() > 0.5
                                randacc = +rand()* (mv-v(i,idx))/rndeventduration;
                            else 
                                randacc = -rand()* v(i,idx)/rndeventduration;
                            end
                            acc(i, idx:idx+rndeventduration/tstep) = randacc;
                            v(i,idx:idx+ceil(rndeventduration/tstep)) = v(i,idx)+(0:ceil(rndeventduration/tstep))*tstep*randacc; % update speed
                            v(i,idx+ceil(rndeventduration/tstep)+1:end) = v(i,idx+ceil(rndeventduration/tstep)); % update speed after acceleration
                            if rand()>0.5
                                %include change of heading for acceleration/deceleration
                                rndturnangle = 180*(rand()-0.5); %turn +/- 90°
                                dheading(i,idx:idx+ceil(rndeventduration/tstep)) = rndturnangle/ceil(rndeventduration/tstep); % iterative update heading
                            end
                        end                        
                    end
                    %Bicycle long turn
                    events = floor(events/2);
                    %eventTidx = sort(ceil(rand(events,1) * size(dheading,2)));
                    eventTidx = sort(ceil(abs(randn(events,1))/2 * size(acc,2)));   
                    eventTidx(eventTidx>size(acc,2)) = sort(ceil(rand(events,1) * size(acc,2)));
                    for event = 1:length(eventTidx)
                        idx = eventTidx(event);
                        rndeventduration = abs(eventduration + (v(i,idx)/mv-0.5) * rand());
                        rndturnangle = 80*(rand()-0.5); %turn +/- 40°
                        dheading(i,idx:idx+ceil(rndeventduration/tstep)) = rndturnangle/ceil(rndeventduration/tstep); % iterative update heading
                    end
                    
                elseif strcmp(names{i}(1:3), 'Ped')
                    %% Plan Pedestrian
                    eval(['mv = 1.4*Targets.',names{i}, '.Height;']);
                    eval(['v(i,:) = ones(1,size(v,2))* Targets.',names{i}, '.WalkingSpeed;']);
                    eventduration = 2; % acceleration of a pedestrian takes ~2s
                    %Pedestrian acceleration
                    events = floor(rand()*Szduration/eventduration+0.2); %number of events possible in Szene
                    eventTidx = sort(ceil(rand(events,1) * size(acc,2)));            
                    for event = 1:length(eventTidx)
                        idx = eventTidx(event);
                        if (mv-v(i,idx))<0.5*mv
                            %high velocity -> decelerate to 0
                            rndeventduration = abs(eventduration + randn()); %random time to decelerate
                            randacc = -rand()* v(i,idx)/rndeventduration; %random acceleration
                            acc(i, idx:idx+ceil(rndeventduration/tstep)) = randacc; % acceleration to trajectory
                            v(i,idx:idx+ceil(rndeventduration/tstep)) = v(i,idx)+(0:ceil(rndeventduration/tstep))*tstep*randacc; % iterative update speed
                            v(i,idx+ceil(rndeventduration/tstep)+1:end) = v(i,idx+ceil(rndeventduration/tstep)); % update speed after acceleration
                            if rand()>0.3
                                %include change of heading for deceleration
                                rndturnangle = 180*(rand()-0.5); %turn +/- 90°
                                dheading(i,idx:idx+ceil(rndeventduration/tstep)) = rndturnangle/ceil(rndeventduration/tstep); % iterative update heading
                            end
                        elseif (mv-v(i,idx))>=0.5*mv
                            %low velocity -> accelerate to mv=1.4*ped.Height
                            rndeventduration = abs(eventduration + randn());
                            randacc = +rand()* v(i,idx)/rndeventduration;
                            acc(i, idx:idx+ceil(rndeventduration/tstep)) = randacc;
                            v(i,idx:idx+ceil(rndeventduration/tstep)) = v(i,idx)+(0:ceil(rndeventduration/tstep))*tstep*randacc; % update speed
                            v(i,idx+ceil(rndeventduration/tstep)+1:end) = v(i,idx+ceil(rndeventduration/tstep)); % update speed after acceleration
                        end                        
                    end
                    %Pedestrian long turn
                    events = floor(events/2);
                    eventTidx = sort(ceil(rand(events,1) * size(dheading,2)));
                    for event = 1:length(eventTidx)
                        idx = eventTidx(event);
                        rndeventduration = abs(eventduration + (v(i,idx)/mv-0.5) * rand());
                        rndturnangle = 60*(rand()-0.5); %turn +/- 30°
                        dheading(i,idx:idx+ceil(rndeventduration/tstep)) = rndturnangle/ceil(rndeventduration/tstep); % iterative update heading
                    end
                       
                end
                
                
                %% Map trajectory to xy
                % Init first position
                if strcmp(names{i}(1:3),'Veh')
                    eval(['mapPos(i,1,:) = [Targets.', names{i},'.xPos, Targets.', names{i},'.yPos];']);
                    eval(['dir(i,:) = Targets.', names{i},'.heading + dheading(i,1);']);
                elseif strcmp(names{i}(1:3),'Bic')
                    eval(['mapPos(i,1,:) = Targets.', names{i},'.InitialPosition(1:2);']);
                    eval(['dir(i,:) = Targets.', names{i},'.InitialHeading + dheading(i,1);']);
                elseif strcmp(names{i}(1:3), 'Ped')
                    eval(['mapPos(i,1,:) = Targets.', names{i},'.InitialPosition(1:2);']); 
                    eval(['dir(i,:) = Targets.', names{i},'.InitialHeading + dheading(i,1);']);
                end

                for t = 2:floor(Szduration/tstep)
                    % Update Position for each timestep
                    dir(i,t) = dir(i,t-1)+dheading(i,t);
                    mapPos(i,t,1) = mapPos(i,t-1,1)+ (v(i,t-1)*tstep + 0.5*acc(i,t-1)*tstep^2)* cosd(dir(i,t-1)); % xPos
                    mapPos(i,t,2) = mapPos(i,t-1,2)+ (v(i,t-1)*tstep + 0.5*acc(i,t-1)*tstep^2)* sind(dir(i,t-1)); % yPos

                end
            end
            
            
            
            
            %% Write to trajectory object
            obj.velocity = v;
            obj.heading = dir;
            obj.trajectory = mapPos;
            
            
            
            
            if obj.plotTraj && ~isempty(names)
                %% plot Trajectory
                % PLOT
                figure
                xlim([0,fmcw.rangeBins(end)]);
                ylim([-fmcw.rangeBins(end)/2, fmcw.rangeBins(end)/2]);
                colors = {'k'; 'b'; 'g'; 'r'; 'c'; 'm'; 'y'};
                
                for t = 1:floor(Szduration/tstep)
                    %x = fmcw.rangeBins;
                    %y = fmcw.rangeBins-fmcw.rangeBins(end)/2;
                    xlim([0,fmcw.rangeBins(end)]);
                    ylim([-fmcw.rangeBins(end), fmcw.rangeBins(end)]);
                    scatter(mapPos(:,t,1), mapPos(:,t,2), colors{1:length(mapPos(:,t,2))});
                    hold on
                    pause(tstep/4)
                    
                end
                ylabel('y in metres');
                xlabel('x in metres');
                title('Movement of targets relative to radar at (0,0)')
                legend(names{:});
            end
        end
        
        
        
        function [Targets] = move_TrajectoryPlanner(obj, t, Targets)
            % Update target objects' positions and headings for each timestep
            %   obj.trajectory:         xy Positions for each time step t
            %   obj.heading:            heading angle for each time step t
            %   obj.velocity:           velocity for each time step t
            %   Targets:                Targets to update
            %   t:                      time step
            %   

            names = fieldnames(Targets);

            for i = 1:numel(names)
                % Update target position to next sample
                if strcmp(names{i}(1:3), 'Ped') 
                    eval(['Targets.', names{i},'.InitialPosition(1:2) = obj.trajectory(i,t,:);']);
                    eval(['Targets.', names{i},'.InitialHeading = obj.heading(i,t);']);
                    eval(['Targets.',names{i}, '.WalkingSpeed = obj.velocity(i,t);']);
                elseif strcmp(names{i}(1:3),'Bic')
                    eval(['Targets.', names{i},'.InitialPosition(1:2) = obj.trajectory(i,t,:);']); 
                    eval(['Targets.', names{i},'.InitialHeading = obj.heading(i,t);']);
                    eval(['Targets.',names{i}, '.Speed = obj.velocity(i,t);']);
                elseif strcmp(names{i}(1:3),'Veh')
                    eval(['Targets.', names{i},'.xPos = obj.trajectory(i,t,1);']); 
                    eval(['Targets.', names{i},'.xPos = obj.trajectory(i,t,2);']); 
                    eval(['Targets.', names{i},'.heading = obj.heading(i,t);']);
                    eval(['Targets.',names{i}, '.vel = obj.velocity(i,t);']);
                end

            end

        end
        
        
    end
end

