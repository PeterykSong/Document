clear all

% initialize random theta 1 and theta 2 values for training and testing
NTrain = 100;
NTest = 100;
theta1_train = rand(1,NTrain);
theta2_train = rand(1,NTrain);

theta1_test = rand(1,NTest);
theta2_test = rand(1,NTest);

% send to simulink model to get the end effector results

pTrain = zeros(3,NTest); % end effector position [training]
qTrain = zeros(4,NTest) ; % end effector quaternion [training]

pTest = zeros(3,NTest); % end effector position [testing]
qTest = zeros(4,NTest) ; % end effector quaternion [testing]

% training data
for i=1:NTrain
    simIn = Simulink.SimulationInput("robotModel_blackbox"); %create object
    theta1 = theta1_train(i);
    theta2 = theta2_train(i);
    
    out = sim(simIn); %run simulation, all results returned in "out"
    % end effector position
    pTrain(:,i) = out.p.data(:,:,end);
    % end effector quaternion
    qTrain(:,i) = out.q.data(:,:,end);
end

% testing data
for i=1:NTest
    simIn = Simulink.SimulationInput("robotModel_blackbox"); %create object

    theta1 = theta1_test(i);
    theta2 = theta2_test(i);
    
    out = sim(simIn); %run simulation, all results returned in "out"
    % end effector position
    pTest(:,i) = out.p.data(:,:,end);
    % end effector quaternion
    qTest(:,i) = out.q.data(:,:,end);
end

