ik = inverseKinematics('RigidBodyTree',robot);

body1 = rigidBody('body1');

jnt1 = rigidBodyJoint('jnt1','prismatic');
tform1 = trvec2tform([0, 100, 0]); % User defined
setFixedTransform(jnt1,tform1);

body1.Joint = jnt1;
robot = rigidBodyTree;
addBody(robot,body1,'base')

body2 = rigidBody("body2");

jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = 0;
tform2 = trvec2tform([0,150,0]);
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1');

endeff = rigidBody('endeff');
jnt3 = rigidBodyJoint('jnt3','prismatic');
tform3 = trvec2tform([0,150,0]);
setFixedTransform(jnt3,tform3);
endeff.Joint = jnt3;
addBody(robot,endeff,'body2')

ik.RigidBodyTree = robot;

% for th = 1:31
%     x = R*cos(th/10) +500;
%     y = R*sin(th/10) +100;
%     z = 0;
%     pose = [x y z];
%     poseTF = trvec2tform(pose);
%     weights = [0 0 0 1 1 1];
%     initialguess = homeConfiguration(robot);
%     [configSoln,solnInfo] = ik("endeff",poseTF,weights,initialguess);
%     disp(th)
%     show(robot,configSoln,PreservePlot=false);
% end

x = 500;
y = 350;
z = 0;
pose = [x y z];
poseTF = trvec2tform(pose);
weights = [0 0 0 1 1 1];
initialguess = homeConfiguration(robot);
[configSoln,solnInfo] = ik("endeff",poseTF,weights,initialguess);
show(robot,configSoln,PreservePlot=true);
title("End-Effector Target Position Achieved")
