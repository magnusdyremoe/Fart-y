%% Simple script producing gust wind using the dryden model

% Assuming that the direction cosine matrix of the aircraft is the identity
% matrix. 
attitude = Rzyx(0,0,0);
height = 10000;
speed = 25;

sim ('dryden', 100)

dryden = ans;

dryden.plot

